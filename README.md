# NAU7802 STM32 HAL Library
STM32 HAL-based driver library for NAU7802 24-bit ADC and load cell amplifier.

## Description
This project provides a complete solution for interfacing with the NAU7802 precision 24-bit ADC load cell amplifier using STM32 microcontrollers with HAL libraries. The library includes calibration, tare 

functionality, and weight measurement features.

## Features
NAU7802_Init() - Device initialization and configuration

NAU7802_Calibrate() - Internal calibration process

NAU7802_Tare() - Zero calibration (tare) function

NAU7802_Read() - Raw ADC value reading

NAU7802_GetWeight() - Calibrated weight calculation

NAU7802_SetCalibrationFactor() - Calibration factor setting

NAU7802_Available() - Data ready check

## Requirements
STM32CubeIDE

STM32 HAL Library

I2C peripheral

UART for debugging output

## Hardware Configuration

**I2C Configuration (STM32CubeMX)**

- Clock Speed: 100 kHz
- Duty Cycle: 2
- Addressing Mode: 7-bit
- No Stretch Mode: Disabled

**UART Configuration (STM32CubeMX)**

- Baud Rate: 115200
- Word Length: 8-bit
- Stop Bits: 1
- Parity: None
- Flow Control: None

## Installation
Add NAU7802.h and NAU7802.c files to your project

Configure I2C1 and USART1 in STM32CubeMX with above settings

Include the following in your main.c:

#include "NAU7802.h"

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

NAU7802_Handle nau7802;

## Usage Example

// Initialize NAU7802
if (!NAU7802_Init(&nau7802, &hi2c1)) {
    printf("ERROR: NAU7802 initialization failed!\r\n");
}

// Perform internal calibration
if (!NAU7802_Calibrate(&nau7802, NAU7802_CALMOD_INTERNAL)) {
    printf("WARNING: Internal calibration failed!\r\n");
}

// Tare (zero) the scale
if (!NAU7802_Tare(&nau7802, 10)) {
    printf("ERROR: Tare failed!\r\n");
}

// Set calibration factor (adjust based on known weight)
NAU7802_SetCalibrationFactor(&nau7802, -6800.0f);

// Read weight continuously
if (NAU7802_Available(&nau7802)) {
    int32_t raw_value = NAU7802_Read(&nau7802);
    float weight = NAU7802_GetWeight(&nau7802, 10);
    printf("Raw: %ld\tWeight: %.3f kg\r\n", raw_value, weight);
}

## Key Functions
Initialization
bool NAU7802_Init(NAU7802_Handle *handle, I2C_HandleTypeDef *hi2c)
Initializes the NAU7802 device with default settings.

Calibration
bool NAU7802_Calibrate(NAU7802_Handle *handle, NAU7802_CalibrationMode mode)
Performs internal calibration of the ADC.

Tare Function
bool NAU7802_Tare(NAU7802_Handle *handle, uint8_t samples)
Sets the current reading as zero point (tare).

Weight Reading
float NAU7802_GetWeight(NAU7802_Handle *handle, uint8_t samples)
Returns the calibrated weight value averaged over specified samples.

## Main Program Flow
System Initialization - Clock, GPIO, I2C, UART
NAU7802 Initialization - Device setup and check
Calibration - Internal calibration process
Tare Operation - Zero point setting
Continuous Reading - Real-time weight measurement
LED Indicator - Status LED blinking

## Configuration Notes
Calibration Factor: The value -6800.0f should be adjusted based on your specific load cell and known weight
Sample Rate: Default is 10 samples per second
Gain: Default gain is 128
LDO Voltage: Default is 3.3V

## Error Handling
Initialization failure triggers LED blinking and error messages
Calibration warnings are displayed but don't halt execution
UART printf redirection for debugging output

## Output Format
The program outputs both raw ADC values and converted weight:
text
Raw: 12345    Weight: 0.250 kg    (250.0 g)
