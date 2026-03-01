/*
 * sensor_task.c
 * 
 * FreeRTOS task to read VL53L0X ToF sensor via I2C
 * Runs at fixed 20ms interval
 * Sends distance data to queue for other tasks
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "sensor_task.h"
#include <stdint.h>
#include <stdio.h>

/* External I2C handle (configured by STM32CubeMX) */
extern I2C_HandleTypeDef hi2c1;

/* VL53L0X I2C Address */
#define VL53L0X_I2C_ADDR (0x29 << 1)  /* 7-bit address shifted left */

/* VL53L0X Register Map */
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID    0xC0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID 0xC2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD 0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START 0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS 0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS 0x14
#define VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN 0xBC
#define VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN 0xC0
#define VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF 0xD0
#define VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_REF 0xD4
#define VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF 0xB6

/* Sensor state machine */
typedef enum {
    SENSOR_STATE_INIT,
    SENSOR_STATE_IDLE,
    SENSOR_STATE_MEASURING,
    SENSOR_STATE_ERROR
} SensorState_t;

/* Global queue for sensor data (created in main.c) */
QueueHandle_t sensorDataQueue = NULL;

/* ----- Helper Functions ----- */

/**
 * @brief Read byte from VL53L0X register
 */
static HAL_StatusTypeDef VL53L0X_ReadByte(uint8_t reg, uint8_t *value) {
    return HAL_I2C_Mem_Read(&hi2c1, VL53L0X_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value, 1, 100);
}

/**
 * @brief Write byte to VL53L0X register
 */
static HAL_StatusTypeDef VL53L0X_WriteByte(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&hi2c1, VL53L0X_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

/**
 * @brief Read word (2 bytes) from VL53L0X register
 */
static HAL_StatusTypeDef VL53L0X_ReadWord(uint8_t reg, uint16_t *value) {
    uint8_t data[2];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, VL53L0X_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
    if (status == HAL_OK) {
        *value = (data[0] << 8) | data[1];  /* Big-endian */
    }
    return status;
}

/**
 * @brief Initialize VL53L0X sensor
 */
static SensorState_t VL53L0X_Init(void) {
    uint8_t model_id;
    
    /* Read model ID to verify sensor presence */
    if (VL53L0X_ReadByte(VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model_id) != HAL_OK) {
        printf("[SENSOR] ERROR: Failed to read model ID\r\n");
        return SENSOR_STATE_ERROR;
    }
    
    /* Expected model ID for VL53L0X is 0xEE */
    if (model_id != 0xEE) {
        printf("[SENSOR] ERROR: Invalid model ID: 0x%02X (expected 0xEE)\r\n", model_id);
        return SENSOR_STATE_ERROR;
    }
    
    printf("[SENSOR] Model ID verified: 0x%02X\r\n", model_id);
    
    /* Set ranging mode (continuous, non-blocking measurement) */
    if (VL53L0X_WriteByte(VL53L0X_REG_SYSRANGE_START, 0x01) != HAL_OK) {
        printf("[SENSOR] ERROR: Failed to start ranging\r\n");
        return SENSOR_STATE_ERROR;
    }
    
    printf("[SENSOR] Ranging started\r\n");
    
    return SENSOR_STATE_IDLE;
}

/**
 * @brief Read distance from sensor
 */
static HAL_StatusTypeDef VL53L0X_ReadDistance(uint16_t *distance_mm) {
    uint8_t status_byte;
    uint16_t raw_distance;
    
    /* Check measurement completion */
    if (VL53L0X_ReadByte(VL53L0X_REG_RESULT_INTERRUPT_STATUS, &status_byte) != HAL_OK) {
        return HAL_ERROR;
    }
    
    /* Check if data is ready (bit 0 set = data ready) */
    if ((status_byte & 0x01) == 0) {
        return HAL_BUSY;  /* Not ready yet */
    }
    
    /* Read raw distance */
    if (VL53L0X_ReadWord(VL53L0X_REG_RESULT_RANGE_STATUS + 10, &raw_distance) != HAL_OK) {
        return HAL_ERROR;
    }
    
    *distance_mm = raw_distance;
    
    return HAL_OK;
}

/* ----- FreeRTOS Task ----- */

/**
 * @brief Sensor acquisition task
 * 
 * Periodically reads VL53L0X and sends data to queue.
 * Priority: High (real-time critical)
 * Stack: 256 words
 */
void SensorTask(void *argument) {
    SensorState_t sensor_state = SENSOR_STATE_INIT;
    DistanceReading_t reading;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20);  /* 20ms sampling */
    
    uint32_t error_count = 0;
    const uint32_t MAX_ERRORS = 10;
    
    printf("[SENSOR] Task starting...\r\n");
    
    /* Initialize sensor */
    sensor_state = VL53L0X_Init();
    if (sensor_state == SENSOR_STATE_ERROR) {
        printf("[SENSOR] Initialization failed - halting task\r\n");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    sensor_state = SENSOR_STATE_IDLE;
    printf("[SENSOR] Task ready - sampling at 20ms intervals\r\n");
    
    while (1) {
        /* Wake up every 20ms */
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        
        /* Attempt to read distance */
        HAL_StatusTypeDef read_status = VL53L0X_ReadDistance(&reading.distance_mm);
        
        reading.timestamp = xTaskGetTickCount();
        
        if (read_status == HAL_OK) {
            reading.status = 0;  /* Success */
            error_count = 0;
            
            /* Send to queue (non-blocking, 0 timeout) */
            if (xQueueSend(sensorDataQueue, &reading, 0) != pdPASS) {
                /* Queue full - drop oldest data */
                xQueueReceive(sensorDataQueue, &reading, 0);
                xQueueSend(sensorDataQueue, &reading, 0);
            }
            
            /* Print debug info via UART every 5th reading (100ms) */
            static uint32_t print_count = 0;
            if (++print_count >= 5) {
                printf("[SENSOR] Distance: %d mm | Status: %d | Time: %lu ms\r\n",
                       reading.distance_mm, reading.status, reading.timestamp);
                print_count = 0;
            }
            
        } else if (read_status == HAL_BUSY) {
            /* Data not ready, skip this cycle */
            continue;
            
        } else {
            /* I2C error */
            reading.status = 1;  /* Error */
            error_count++;
            
            if (error_count > MAX_ERRORS) {
                /* Too many errors - sensor timeout */
                printf("[SENSOR] ERROR: Too many consecutive errors (%lu) - entering fault state\r\n", error_count);
                sensor_state = SENSOR_STATE_ERROR;
            } else {
                printf("[SENSOR] ERROR: Read failed (error %lu/%lu)\r\n", error_count, MAX_ERRORS);
            }
        }
    }
}