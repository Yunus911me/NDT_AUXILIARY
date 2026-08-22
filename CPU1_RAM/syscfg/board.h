/*
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef BOARD_H
#define BOARD_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//
// Included Files
//

#include "driverlib.h"
#include "device.h"

//*****************************************************************************
//
// PinMux Configurations
//
//*****************************************************************************

//
// ANALOG -> myANALOGPinMux0 Pinmux
//
//
// GPIO0 - GPIO Settings
//
#define CH5_B_MCU_GPIO_PIN_CONFIG GPIO_0_GPIO0
//
// GPIO1 - GPIO Settings
//
#define CH5_A_MCU_GPIO_PIN_CONFIG GPIO_1_GPIO1
//
// GPIO5 - GPIO Settings
//
#define CH4_A_MCU_GPIO_PIN_CONFIG GPIO_5_GPIO5
//
// GPIO45 - GPIO Settings
//
#define CH4_B_MCU_GPIO_PIN_CONFIG GPIO_45_GPIO45
//
// GPIO6 - GPIO Settings
//
#define CH1_A_MCU_GPIO_PIN_CONFIG GPIO_6_GPIO6
//
// GPIO14 - GPIO Settings
//
#define CH1_B_MCU_GPIO_PIN_CONFIG GPIO_14_GPIO14
//
// GPIO15 - GPIO Settings
//
#define CH2_A_MCU_GPIO_PIN_CONFIG GPIO_15_GPIO15
//
// GPIO34 - GPIO Settings
//
#define CH2_B_MCU_GPIO_PIN_CONFIG GPIO_34_GPIO34
//
// GPIO23 - GPIO Settings
//
#define CH6_B_MCU_GPIO_PIN_CONFIG GPIO_23_GPIO23
//
// GPIO40 - GPIO Settings
//
#define CH6_A_MCU_GPIO_PIN_CONFIG GPIO_40_GPIO40
//
// GPIO41 - GPIO Settings
//
#define CH7_A_MCU_GPIO_PIN_CONFIG GPIO_41_GPIO41
//
// GPIO22 - GPIO Settings
//
#define CH7_B_MCU_GPIO_PIN_CONFIG GPIO_22_GPIO22
//
// GPIO7 - GPIO Settings
//
#define CH8_A_MCU_GPIO_PIN_CONFIG GPIO_7_GPIO7
//
// GPIO9 - GPIO Settings
//
#define CH3_B_MCU_GPIO_PIN_CONFIG GPIO_9_GPIO9
//
// GPIO10 - GPIO Settings
//
#define CH3_A_MCU_GPIO_PIN_CONFIG GPIO_10_GPIO10
//
// GPIO44 - GPIO Settings
//
#define CH8_B_MCU_GPIO_PIN_CONFIG GPIO_44_GPIO44
//
// GPIO25 - GPIO Settings
//
#define Pulser_EN_GPIO_PIN_CONFIG GPIO_25_GPIO25
//
// GPIO17 - GPIO Settings
//
#define Analog_EN_GPIO_PIN_CONFIG GPIO_17_GPIO17
//
// GPIO39 - GPIO Settings
//
#define MODE0_GPIO_PIN_CONFIG GPIO_39_GPIO39
//
// GPIO42 - GPIO Settings
//
#define MODE1_GPIO_PIN_CONFIG GPIO_42_GPIO42
//
// GPIO16 - GPIO Settings
//
#define MCU_LED_GPIO_PIN_CONFIG GPIO_16_GPIO16
//
// GPIO31 - GPIO Settings
//
#define AFE_GAIN_A1_GPIO_PIN_CONFIG GPIO_31_GPIO31
//
// GPIO30 - GPIO Settings
//
#define AFE_GAIN_A2_GPIO_PIN_CONFIG GPIO_30_GPIO30
//
// GPIO4 - GPIO Settings
//
#define AFE_GAIN_B1_GPIO_PIN_CONFIG GPIO_4_GPIO4
//
// GPIO8 - GPIO Settings
//
#define AFE_GAIN_B2_GPIO_PIN_CONFIG GPIO_8_GPIO8
//
// GPIO13 - GPIO Settings
//
#define THP_GPIO_PIN_CONFIG GPIO_13_GPIO13
//
// GPIO24 - GPIO Settings
//
#define BOOT0_GPIO_PIN_CONFIG GPIO_24_GPIO24
//
// GPIO32 - GPIO Settings
//
#define BOOT1_GPIO_PIN_CONFIG GPIO_32_GPIO32
//
// GPIO33 - GPIO Settings
//
#define EEPROM_WC_GPIO_PIN_CONFIG GPIO_33_GPIO33

//
// I2CA -> myI2CA Pinmux
//
//
// I2CA_SDA - GPIO Settings
//
#define GPIO_PIN_I2CA_SDA 26
#define myI2CA_I2CSDA_GPIO 26
#define myI2CA_I2CSDA_PIN_CONFIG GPIO_26_I2CA_SDA
//
// I2CA_SCL - GPIO Settings
//
#define GPIO_PIN_I2CA_SCL 27
#define myI2CA_I2CSCL_GPIO 27
#define myI2CA_I2CSCL_PIN_CONFIG GPIO_27_I2CA_SCL

//
// I2CB -> myI2CB Pinmux
//
//
// I2CB_SDA - GPIO Settings
//
#define GPIO_PIN_I2CB_SDA 2
#define myI2CB_I2CSDA_GPIO 2
#define myI2CB_I2CSDA_PIN_CONFIG GPIO_2_I2CB_SDA
//
// I2CB_SCL - GPIO Settings
//
#define GPIO_PIN_I2CB_SCL 3
#define myI2CB_I2CSCL_GPIO 3
#define myI2CB_I2CSCL_PIN_CONFIG GPIO_3_I2CB_SCL

//*****************************************************************************
//
// ADC Configurations
//
//*****************************************************************************
#define myADCA_BASE ADCA_BASE
#define myADCA_RESULT_BASE ADCARESULT_BASE
#define myADCA_SOC0 ADC_SOC_NUMBER0
#define myADCA_FORCE_SOC0 ADC_FORCE_SOC0
#define myADCA_SAMPLE_WINDOW_SOC0 150
#define myADCA_TRIGGER_SOURCE_SOC0 ADC_TRIGGER_EPWM1_SOCA
#define myADCA_CHANNEL_SOC0 ADC_CH_ADCIN6
#define myADCA_SOC1 ADC_SOC_NUMBER1
#define myADCA_FORCE_SOC1 ADC_FORCE_SOC1
#define myADCA_SAMPLE_WINDOW_SOC1 150
#define myADCA_TRIGGER_SOURCE_SOC1 ADC_TRIGGER_EPWM1_SOCA
#define myADCA_CHANNEL_SOC1 ADC_CH_ADCIN3
#define myADCA_SOC2 ADC_SOC_NUMBER2
#define myADCA_FORCE_SOC2 ADC_FORCE_SOC2
#define myADCA_SAMPLE_WINDOW_SOC2 150
#define myADCA_TRIGGER_SOURCE_SOC2 ADC_TRIGGER_EPWM1_SOCA
#define myADCA_CHANNEL_SOC2 ADC_CH_ADCIN2
#define myADCA_SOC3 ADC_SOC_NUMBER3
#define myADCA_FORCE_SOC3 ADC_FORCE_SOC3
#define myADCA_SAMPLE_WINDOW_SOC3 150
#define myADCA_TRIGGER_SOURCE_SOC3 ADC_TRIGGER_EPWM1_SOCA
#define myADCA_CHANNEL_SOC3 ADC_CH_ADCIN9
#define myADCA_SOC4 ADC_SOC_NUMBER4
#define myADCA_FORCE_SOC4 ADC_FORCE_SOC4
#define myADCA_SAMPLE_WINDOW_SOC4 640
#define myADCA_TRIGGER_SOURCE_SOC4 ADC_TRIGGER_SW_ONLY
#define myADCA_CHANNEL_SOC4 ADC_CH_ADCIN12
#define myADCA_SOC5 ADC_SOC_NUMBER5
#define myADCA_FORCE_SOC5 ADC_FORCE_SOC5
#define myADCA_SAMPLE_WINDOW_SOC5 640
#define myADCA_TRIGGER_SOURCE_SOC5 ADC_TRIGGER_SW_ONLY
#define myADCA_CHANNEL_SOC5 ADC_CH_ADCIN5
#define myADCA_SOC6 ADC_SOC_NUMBER6
#define myADCA_FORCE_SOC6 ADC_FORCE_SOC6
#define myADCA_SAMPLE_WINDOW_SOC6 640
#define myADCA_TRIGGER_SOURCE_SOC6 ADC_TRIGGER_SW_ONLY
#define myADCA_CHANNEL_SOC6 ADC_CH_ADCIN1
#define myADCA_SOC7 ADC_SOC_NUMBER7
#define myADCA_FORCE_SOC7 ADC_FORCE_SOC7
#define myADCA_SAMPLE_WINDOW_SOC7 640
#define myADCA_TRIGGER_SOURCE_SOC7 ADC_TRIGGER_SW_ONLY
#define myADCA_CHANNEL_SOC7 ADC_CH_ADCIN11
#define myADCA_SOC8 ADC_SOC_NUMBER8
#define myADCA_FORCE_SOC8 ADC_FORCE_SOC8
#define myADCA_SAMPLE_WINDOW_SOC8 640
#define myADCA_TRIGGER_SOURCE_SOC8 ADC_TRIGGER_SW_ONLY
#define myADCA_CHANNEL_SOC8 ADC_CH_ADCIN0
void myADCA_init();

#define myADCC_BASE ADCC_BASE
#define myADCC_RESULT_BASE ADCCRESULT_BASE
#define myADCC_SOC0 ADC_SOC_NUMBER0
#define myADCC_FORCE_SOC0 ADC_FORCE_SOC0
#define myADCC_SAMPLE_WINDOW_SOC0 150
#define myADCC_TRIGGER_SOURCE_SOC0 ADC_TRIGGER_EPWM1_SOCA
#define myADCC_CHANNEL_SOC0 ADC_CH_ADCIN6
#define myADCC_SOC1 ADC_SOC_NUMBER1
#define myADCC_FORCE_SOC1 ADC_FORCE_SOC1
#define myADCC_SAMPLE_WINDOW_SOC1 150
#define myADCC_TRIGGER_SOURCE_SOC1 ADC_TRIGGER_EPWM1_SOCA
#define myADCC_CHANNEL_SOC1 ADC_CH_ADCIN14
#define myADCC_SOC2 ADC_SOC_NUMBER2
#define myADCC_FORCE_SOC2 ADC_FORCE_SOC2
#define myADCC_SAMPLE_WINDOW_SOC2 150
#define myADCC_TRIGGER_SOURCE_SOC2 ADC_TRIGGER_EPWM1_SOCA
#define myADCC_CHANNEL_SOC2 ADC_CH_ADCIN11
#define myADCC_SOC3 ADC_SOC_NUMBER3
#define myADCC_FORCE_SOC3 ADC_FORCE_SOC3
#define myADCC_SAMPLE_WINDOW_SOC3 150
#define myADCC_TRIGGER_SOURCE_SOC3 ADC_TRIGGER_EPWM1_SOCA
#define myADCC_CHANNEL_SOC3 ADC_CH_ADCIN10
void myADCC_init();


//*****************************************************************************
//
// ASYSCTL Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// GPIO Configurations
//
//*****************************************************************************
#define CH5_B_MCU 0
void CH5_B_MCU_init();
#define CH5_A_MCU 1
void CH5_A_MCU_init();
#define CH4_A_MCU 5
void CH4_A_MCU_init();
#define CH4_B_MCU 45
void CH4_B_MCU_init();
#define CH1_A_MCU 6
void CH1_A_MCU_init();
#define CH1_B_MCU 14
void CH1_B_MCU_init();
#define CH2_A_MCU 15
void CH2_A_MCU_init();
#define CH2_B_MCU 34
void CH2_B_MCU_init();
#define CH6_B_MCU 23
void CH6_B_MCU_init();
#define CH6_A_MCU 40
void CH6_A_MCU_init();
#define CH7_A_MCU 41
void CH7_A_MCU_init();
#define CH7_B_MCU 22
void CH7_B_MCU_init();
#define CH8_A_MCU 7
void CH8_A_MCU_init();
#define CH3_B_MCU 9
void CH3_B_MCU_init();
#define CH3_A_MCU 10
void CH3_A_MCU_init();
#define CH8_B_MCU 44
void CH8_B_MCU_init();
#define Pulser_EN 25
void Pulser_EN_init();
#define Analog_EN 17
void Analog_EN_init();
#define MODE0 39
void MODE0_init();
#define MODE1 42
void MODE1_init();
#define MCU_LED 16
void MCU_LED_init();
#define AFE_GAIN_A1 31
void AFE_GAIN_A1_init();
#define AFE_GAIN_A2 30
void AFE_GAIN_A2_init();
#define AFE_GAIN_B1 4
void AFE_GAIN_B1_init();
#define AFE_GAIN_B2 8
void AFE_GAIN_B2_init();
#define THP 13
void THP_init();
#define BOOT0 24
void BOOT0_init();
#define BOOT1 32
void BOOT1_init();
#define EEPROM_WC 33
void EEPROM_WC_init();

//*****************************************************************************
//
// I2C Configurations
//
//*****************************************************************************
#define myI2CA_BASE I2CA_BASE
#define myI2CA_BITRATE 400000
#define myI2CA_TARGET_ADDRESS 0
#define myI2CA_OWN_ADDRESS 0
#define myI2CA_MODULE_CLOCK_FREQUENCY 10000000
void myI2CA_init();
#define myI2CB_BASE I2CB_BASE
#define myI2CB_BITRATE 400000
#define myI2CB_TARGET_ADDRESS 33
#define myI2CB_OWN_ADDRESS 33
#define myI2CB_MODULE_CLOCK_FREQUENCY 10000000
void myI2CB_init();

//*****************************************************************************
//
// INTERRUPT Configurations
//
//*****************************************************************************

// Interrupt Settings for INT_myI2CB
// ISR need to be defined for the registered interrupts
#define INT_myI2CB INT_I2CB
#define INT_myI2CB_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP8
extern __interrupt void INT_myI2CB_ISR(void);

//*****************************************************************************
//
// SYSCTL Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// Board Configurations
//
//*****************************************************************************
void	Board_init();
void	ADC_init();
void	ASYSCTL_init();
void	GPIO_init();
void	I2C_init();
void	INTERRUPT_init();
void	SYSCTL_init();
void	PinMux_init();

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif  // end of BOARD_H definition
