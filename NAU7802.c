/**
 * @file NAU7802.c
 * @brief NAU7802 24-bit ADC driver implementation for STM32
 */

#include "NAU7802.h"

#define I2C_TIMEOUT 100

// Helper Functions
static bool NAU7802_WriteRegister(NAU7802_Handle *handle, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(handle->hi2c, NAU7802_I2C_ADDR, data, 2, I2C_TIMEOUT) == HAL_OK;
}

static bool NAU7802_ReadRegister(NAU7802_Handle *handle, uint8_t reg, uint8_t *value) {
    if (HAL_I2C_Master_Transmit(handle->hi2c, NAU7802_I2C_ADDR, &reg, 1, I2C_TIMEOUT) != HAL_OK)
        return false;
    return HAL_I2C_Master_Receive(handle->hi2c, NAU7802_I2C_ADDR, value, 1, I2C_TIMEOUT) == HAL_OK;
}

static bool NAU7802_SetBits(NAU7802_Handle *handle, uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t reg_value;
    if (!NAU7802_ReadRegister(handle, reg, &reg_value))
        return false;
    
    reg_value &= ~mask;
    reg_value |= (value & mask);
    
    return NAU7802_WriteRegister(handle, reg, reg_value);
}

static bool NAU7802_GetBit(NAU7802_Handle *handle, uint8_t reg, uint8_t bit) {
    uint8_t value;
    if (!NAU7802_ReadRegister(handle, reg, &value))
        return false;
    return (value & (1 << bit)) != 0;
}

/**
 * @brief Initialize NAU7802 sensor
 */
bool NAU7802_Init(NAU7802_Handle *handle, I2C_HandleTypeDef *hi2c) {
    handle->hi2c = hi2c;
    handle->calibration_factor = 1.0f;
    handle->zero_offset = 0;
    
    // Reset the device
    if (!NAU7802_Reset(handle))
        return false;
    
    // Enable the device
    if (!NAU7802_Enable(handle, true))
        return false;
    
    // Check revision ID (low nibble should be 0xF)
    uint8_t rev;
    if (!NAU7802_ReadRegister(handle, NAU7802_REVISION_ID, &rev))
        return false;
    if ((rev & 0x0F) != 0x0F)
        return false;
    
    // Configure device
    if (!NAU7802_SetLDO(handle, NAU7802_LDO_3V0))
        return false;
    if (!NAU7802_SetGain(handle, NAU7802_GAIN_128))
        return false;
    if (!NAU7802_SetRate(handle, NAU7802_RATE_10SPS))
        return false;
    
    // Disable ADC chopper clock
    if (!NAU7802_SetBits(handle, NAU7802_ADC, 0x30, 0x30))
        return false;
    
    // Use low ESR caps
    if (!NAU7802_SetBits(handle, NAU7802_PGA, 0x40, 0x00))
        return false;
    
    return true;
}

/**
 * @brief Reset NAU7802
 */
bool NAU7802_Reset(NAU7802_Handle *handle) {
    // Set RR bit to 1
    if (!NAU7802_SetBits(handle, NAU7802_PU_CTRL, 0x01, 0x01))
        return false;
    HAL_Delay(10);
    
    // Set RR bit to 0 and PUD bit to 1
    if (!NAU7802_SetBits(handle, NAU7802_PU_CTRL, 0x01, 0x00))
        return false;
    if (!NAU7802_SetBits(handle, NAU7802_PU_CTRL, 0x02, 0x02))
        return false;
    
    HAL_Delay(1);
    
    // Check if device is ready
    return NAU7802_GetBit(handle, NAU7802_PU_CTRL, 3);
}

/**
 * @brief Enable or disable NAU7802
 */
bool NAU7802_Enable(NAU7802_Handle *handle, bool enable) {
    if (!enable) {
        // Power down
        if (!NAU7802_SetBits(handle, NAU7802_PU_CTRL, 0x04, 0x00))
            return false;
        if (!NAU7802_SetBits(handle, NAU7802_PU_CTRL, 0x02, 0x00))
            return false;
        return true;
    }
    
    // Power up
    if (!NAU7802_SetBits(handle, NAU7802_PU_CTRL, 0x02, 0x02))
        return false;
    if (!NAU7802_SetBits(handle, NAU7802_PU_CTRL, 0x04, 0x04))
        return false;
    
    HAL_Delay(600); // Wait for analog to stabilize
    
    if (!NAU7802_SetBits(handle, NAU7802_PU_CTRL, 0x10, 0x10))
        return false;
    
    return NAU7802_GetBit(handle, NAU7802_PU_CTRL, 3);
}

/**
 * @brief Check if new data is available
 */
bool NAU7802_Available(NAU7802_Handle *handle) {
    return NAU7802_GetBit(handle, NAU7802_PU_CTRL, 5);
}

/**
 * @brief Read 24-bit ADC value
 */
int32_t NAU7802_Read(NAU7802_Handle *handle) {
    uint8_t data[3];
    uint8_t reg = NAU7802_ADCO_B2;
    
    if (HAL_I2C_Master_Transmit(handle->hi2c, NAU7802_I2C_ADDR, &reg, 1, I2C_TIMEOUT) != HAL_OK)
        return 0;
    if (HAL_I2C_Master_Receive(handle->hi2c, NAU7802_I2C_ADDR, data, 3, I2C_TIMEOUT) != HAL_OK)
        return 0;
    
    uint32_t value = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    
    // Sign extend from 24-bit to 32-bit
    if (value & 0x800000) {
        value |= 0xFF000000;
    }
    
    return (int32_t)value;
}

/**
 * @brief Set ADC channel (0 or 1)
 */
bool NAU7802_SetChannel(NAU7802_Handle *handle, uint8_t channel) {
    if (channel > 1)
        channel = 1;
    return NAU7802_SetBits(handle, NAU7802_CTRL2, 0x80, channel ? 0x80 : 0x00);
}

/**
 * @brief Set LDO voltage
 */
bool NAU7802_SetLDO(NAU7802_Handle *handle, NAU7802_LDOVoltage voltage) {
    if (voltage == NAU7802_LDO_EXTERNAL) {
        return NAU7802_SetBits(handle, NAU7802_PU_CTRL, 0x80, 0x00);
    }
    
    // Enable internal LDO
    if (!NAU7802_SetBits(handle, NAU7802_PU_CTRL, 0x80, 0x80))
        return false;
    
    return NAU7802_SetBits(handle, NAU7802_CTRL1, 0x38, (voltage << 3));
}

/**
 * @brief Set gain
 */
bool NAU7802_SetGain(NAU7802_Handle *handle, NAU7802_Gain gain) {
    return NAU7802_SetBits(handle, NAU7802_CTRL1, 0x07, gain);
}

/**
 * @brief Set sample rate
 */
bool NAU7802_SetRate(NAU7802_Handle *handle, NAU7802_SampleRate rate) {
    return NAU7802_SetBits(handle, NAU7802_CTRL2, 0x70, (rate << 4));
}

/**
 * @brief Perform calibration
 */
bool NAU7802_Calibrate(NAU7802_Handle *handle, NAU7802_Calibration mode) {
    // Set calibration mode
    if (!NAU7802_SetBits(handle, NAU7802_CTRL2, 0x03, mode))
        return false;
    
    // Start calibration
    if (!NAU7802_SetBits(handle, NAU7802_CTRL2, 0x04, 0x04))
        return false;
    
    // Wait for calibration to complete
    uint32_t timeout = HAL_GetTick() + 2000;
    while (!NAU7802_GetBit(handle, NAU7802_CTRL2, 2)) {
        if (HAL_GetTick() > timeout)
            return false;
        HAL_Delay(10);
    }
    
    // Check for calibration error
    return !NAU7802_GetBit(handle, NAU7802_CTRL2, 3);
}

/**
 * @brief Read average of multiple samples
 */
int32_t NAU7802_ReadAverage(NAU7802_Handle *handle, uint8_t samples) {
    int64_t sum = 0;
    
    for (uint8_t i = 0; i < samples; i++) {
        while (!NAU7802_Available(handle)) {
            HAL_Delay(1);
        }
        sum += NAU7802_Read(handle);
    }
    
    return (int32_t)(sum / samples);
}

/**
 * @brief Tare the scale (zero offset)
 */
bool NAU7802_Tare(NAU7802_Handle *handle, uint8_t samples) {
    handle->zero_offset = NAU7802_ReadAverage(handle, samples);
    return true;
}

/**
 * @brief Get weight in calibrated units
 */
float NAU7802_GetWeight(NAU7802_Handle *handle, uint8_t samples) {
    int32_t raw = NAU7802_ReadAverage(handle, samples);
    return (float)(raw - handle->zero_offset) / handle->calibration_factor;
}

/**
 * @brief Set calibration factor
 */
void NAU7802_SetCalibrationFactor(NAU7802_Handle *handle, float factor) {
    handle->calibration_factor = factor;
}