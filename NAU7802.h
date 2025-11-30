/**
 * @file NAU7802.h
 * @brief NAU7802 24-bit ADC driver for STM32
 * @author Adapted for STM32
 */

#ifndef NAU7802_H
#define NAU7802_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// NAU7802 I2C Address
#define NAU7802_I2C_ADDR            0x2A << 1  // 7-bit adres sola kaydırılır

// Register Addresses
#define NAU7802_PU_CTRL             0x00
#define NAU7802_CTRL1               0x01
#define NAU7802_CTRL2               0x02
#define NAU7802_ADCO_B2             0x12
#define NAU7802_ADC                 0x15
#define NAU7802_PGA                 0x1B
#define NAU7802_POWER               0x1C
#define NAU7802_REVISION_ID         0x1F

// LDO Voltage Options
typedef enum {
    NAU7802_LDO_4V5 = 0,
    NAU7802_LDO_4V2,
    NAU7802_LDO_3V9,
    NAU7802_LDO_3V6,
    NAU7802_LDO_3V3,
    NAU7802_LDO_3V0,
    NAU7802_LDO_2V7,
    NAU7802_LDO_2V4,
    NAU7802_LDO_EXTERNAL
} NAU7802_LDOVoltage;

// Gain Options
typedef enum {
    NAU7802_GAIN_1 = 0,
    NAU7802_GAIN_2,
    NAU7802_GAIN_4,
    NAU7802_GAIN_8,
    NAU7802_GAIN_16,
    NAU7802_GAIN_32,
    NAU7802_GAIN_64,
    NAU7802_GAIN_128
} NAU7802_Gain;

// Sample Rate Options
typedef enum {
    NAU7802_RATE_10SPS = 0,
    NAU7802_RATE_20SPS = 1,
    NAU7802_RATE_40SPS = 2,
    NAU7802_RATE_80SPS = 3,
    NAU7802_RATE_320SPS = 7
} NAU7802_SampleRate;

// Calibration Modes
typedef enum {
    NAU7802_CALMOD_INTERNAL = 0,
    NAU7802_CALMOD_OFFSET = 2,
    NAU7802_CALMOD_GAIN = 3
} NAU7802_Calibration;

// NAU7802 Handle Structure
typedef struct {
    I2C_HandleTypeDef *hi2c;
    float calibration_factor;
    int32_t zero_offset;
} NAU7802_Handle;

// Function Prototypes
bool NAU7802_Init(NAU7802_Handle *handle, I2C_HandleTypeDef *hi2c);
bool NAU7802_Reset(NAU7802_Handle *handle);
bool NAU7802_Enable(NAU7802_Handle *handle, bool enable);
bool NAU7802_Available(NAU7802_Handle *handle);
int32_t NAU7802_Read(NAU7802_Handle *handle);
bool NAU7802_SetChannel(NAU7802_Handle *handle, uint8_t channel);
bool NAU7802_SetLDO(NAU7802_Handle *handle, NAU7802_LDOVoltage voltage);
bool NAU7802_SetGain(NAU7802_Handle *handle, NAU7802_Gain gain);
bool NAU7802_SetRate(NAU7802_Handle *handle, NAU7802_SampleRate rate);
bool NAU7802_Calibrate(NAU7802_Handle *handle, NAU7802_Calibration mode);

// Weight Measurement Functions
bool NAU7802_Tare(NAU7802_Handle *handle, uint8_t samples);
float NAU7802_GetWeight(NAU7802_Handle *handle, uint8_t samples);
void NAU7802_SetCalibrationFactor(NAU7802_Handle *handle, float factor);
int32_t NAU7802_ReadAverage(NAU7802_Handle *handle, uint8_t samples);

#endif // NAU7802_H