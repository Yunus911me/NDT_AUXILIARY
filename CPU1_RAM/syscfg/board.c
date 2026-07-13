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

#include "board.h"

//*****************************************************************************
//
// Board Configurations
// Initializes the rest of the modules.
// Call this function in your application if you wish to do all module
// initialization.
// If you wish to not use some of the initializations, instead of the
// Board_init use the individual Module_inits
//
//*****************************************************************************
void Board_init()
{
	EALLOW;

	PinMux_init();
	ASYSCTL_init();
	ADC_init();
	GPIO_init();
	I2C_init();
	INTERRUPT_init();

	EDIS;
}

//*****************************************************************************
//
// PINMUX Configurations
//
//*****************************************************************************
void PinMux_init()
{
	//
	// PinMux for modules assigned to CPU1
	//

	//
	// ANALOG -> myANALOGPinMux0 Pinmux
	//
	// Analog PinMux for A0/C15
	GPIO_setPinConfig(GPIO_231_GPIO231);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(231, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A1
	GPIO_setPinConfig(GPIO_232_GPIO232);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(232, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A10/C10
	GPIO_setPinConfig(GPIO_230_GPIO230);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(230, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A11/C0
	GPIO_setPinConfig(GPIO_237_GPIO237);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(237, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A12/C1
	GPIO_setPinConfig(GPIO_238_GPIO238);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(238, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A14/C4
	GPIO_setPinConfig(GPIO_239_GPIO239);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(239, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A15/C7
	GPIO_setPinConfig(GPIO_233_GPIO233);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(233, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A2/C9
	GPIO_setPinConfig(GPIO_224_GPIO224);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(224, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A3/C5/VDAC
	GPIO_setPinConfig(GPIO_242_GPIO242);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(242, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A4/C14
	GPIO_setPinConfig(GPIO_225_GPIO225);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(225, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A5/C2
	GPIO_setPinConfig(GPIO_244_GPIO244);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(244, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A6
	GPIO_setPinConfig(GPIO_228_GPIO228);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(228, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A7/C3
	GPIO_setPinConfig(GPIO_245_GPIO245);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(245, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A8/C11
	GPIO_setPinConfig(GPIO_241_GPIO241);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(241, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A9/C8
	GPIO_setPinConfig(GPIO_227_GPIO227);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(227, GPIO_ANALOG_ENABLED);
	// Analog PinMux for C6
	GPIO_setPinConfig(GPIO_226_GPIO226);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(226, GPIO_ANALOG_ENABLED);
	// GPIO0 -> CH5_B_MCU Pinmux
	GPIO_setPinConfig(GPIO_0_GPIO0);
	// GPIO1 -> CH5_A_MCU Pinmux
	GPIO_setPinConfig(GPIO_1_GPIO1);
	// GPIO5 -> CH4_A_MCU Pinmux
	GPIO_setPinConfig(GPIO_5_GPIO5);
	// GPIO45 -> CH4_B_MCU Pinmux
	GPIO_setPinConfig(GPIO_45_GPIO45);
	// GPIO6 -> CH1_A_MCU Pinmux
	GPIO_setPinConfig(GPIO_6_GPIO6);
	// GPIO14 -> CH1_B_MCU Pinmux
	GPIO_setPinConfig(GPIO_14_GPIO14);
	// GPIO15 -> CH2_A_MCU Pinmux
	GPIO_setPinConfig(GPIO_15_GPIO15);
	// GPIO34 -> CH2_B_MCU Pinmux
	GPIO_setPinConfig(GPIO_34_GPIO34);
	// GPIO23 -> CH6_B_MCU Pinmux
	GPIO_setPinConfig(GPIO_23_GPIO23);
	// GPIO40 -> CH6_A_MCU Pinmux
	GPIO_setPinConfig(GPIO_40_GPIO40);
	// GPIO41 -> CH7_A_MCU Pinmux
	GPIO_setPinConfig(GPIO_41_GPIO41);
	// GPIO22 -> CH7_B_MCU Pinmux
	GPIO_setPinConfig(GPIO_22_GPIO22);
	// GPIO7 -> CH8_A_MCU Pinmux
	GPIO_setPinConfig(GPIO_7_GPIO7);
	// GPIO9 -> CH3_B_MCU Pinmux
	GPIO_setPinConfig(GPIO_9_GPIO9);
	// GPIO10 -> CH3_A_MCU Pinmux
	GPIO_setPinConfig(GPIO_10_GPIO10);
	// GPIO44 -> CH8_B_MCU Pinmux
	GPIO_setPinConfig(GPIO_44_GPIO44);
	// GPIO25 -> Pulser_EN Pinmux
	GPIO_setPinConfig(GPIO_25_GPIO25);
	// GPIO17 -> Analog_EN Pinmux
	GPIO_setPinConfig(GPIO_17_GPIO17);
	// GPIO39 -> MODE0 Pinmux
	GPIO_setPinConfig(GPIO_39_GPIO39);
	// GPIO42 -> MODE1 Pinmux
	GPIO_setPinConfig(GPIO_42_GPIO42);
	// GPIO16 -> MCU_LED Pinmux
	GPIO_setPinConfig(GPIO_16_GPIO16);
	// GPIO31 -> AFE_GAIN_A1 Pinmux
	GPIO_setPinConfig(GPIO_31_GPIO31);
	// GPIO30 -> AFE_GAIN_A2 Pinmux
	GPIO_setPinConfig(GPIO_30_GPIO30);
	// GPIO4 -> AFE_GAIN_B1 Pinmux
	GPIO_setPinConfig(GPIO_4_GPIO4);
	// GPIO8 -> AFE_GAIN_B2 Pinmux
	GPIO_setPinConfig(GPIO_8_GPIO8);
	// GPIO13 -> THP Pinmux
	GPIO_setPinConfig(GPIO_13_GPIO13);
	// GPIO24 -> BOOT0 Pinmux
	GPIO_setPinConfig(GPIO_24_GPIO24);
	// GPIO32 -> BOOT1 Pinmux
	GPIO_setPinConfig(GPIO_32_GPIO32);
	// GPIO33 -> EEPROM_WC Pinmux
	GPIO_setPinConfig(GPIO_33_GPIO33);
	//
	// I2CA -> myI2CA Pinmux
	//
	GPIO_setPinConfig(myI2CA_I2CSDA_PIN_CONFIG);
	GPIO_setPadConfig(myI2CA_I2CSDA_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
	GPIO_setQualificationMode(myI2CA_I2CSDA_GPIO, GPIO_QUAL_ASYNC);

	GPIO_setPinConfig(myI2CA_I2CSCL_PIN_CONFIG);
	GPIO_setPadConfig(myI2CA_I2CSCL_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
	GPIO_setQualificationMode(myI2CA_I2CSCL_GPIO, GPIO_QUAL_ASYNC);

	//
	// I2CB -> myI2CB Pinmux
	//
	GPIO_setPinConfig(myI2CB_I2CSDA_PIN_CONFIG);
	GPIO_setPadConfig(myI2CB_I2CSDA_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
	GPIO_setQualificationMode(myI2CB_I2CSDA_GPIO, GPIO_QUAL_ASYNC);

	GPIO_setPinConfig(myI2CB_I2CSCL_PIN_CONFIG);
	GPIO_setPadConfig(myI2CB_I2CSCL_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
	GPIO_setQualificationMode(myI2CB_I2CSCL_GPIO, GPIO_QUAL_ASYNC);


}

//*****************************************************************************
//
// ADC Configurations
//
//*****************************************************************************
void ADC_init(){
	myADCA_init();
	myADCC_init();
}

void myADCA_init(){
	//
	// ADC Initialization: Write ADC configurations and power up the ADC
	//
	// Set the analog voltage reference selection and ADC module's offset trims.
	// This function sets the analog voltage reference to internal (with the reference voltage of 1.65V or 2.5V) or external for ADC
	// which is same as ASysCtl APIs.
	//
	ADC_setVREF(myADCA_BASE, ADC_REFERENCE_EXTERNAL, ADC_REFERENCE_2_5V);
	//
	// Configures the analog-to-digital converter module prescaler.
	//
	ADC_setPrescaler(myADCA_BASE, ADC_CLK_DIV_2_0);
	//
	// Sets the timing of the end-of-conversion pulse
	//
	ADC_setInterruptPulseMode(myADCA_BASE, ADC_PULSE_END_OF_ACQ_WIN);
	//
	// Sets the timing of early interrupt generation.
	//
	ADC_setInterruptCycleOffset(myADCA_BASE, 0U);
	//
	// Powers up the analog-to-digital converter core.
	//
	ADC_enableConverter(myADCA_BASE);
	//
	// Delay for 1ms to allow ADC time to power up
	//
	DEVICE_DELAY_US(500);
	//
	// SOC Configuration: Setup ADC EPWM channel and trigger settings
	//
	// Disables SOC burst mode.
	//
	ADC_disableBurstMode(myADCA_BASE);
	//
	// Sets the priority mode of the SOCs.
	//
	ADC_setSOCPriority(myADCA_BASE, ADC_PRI_ALL_ROUND_ROBIN);
	//
	// Start of Conversion 0 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 0
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN6
	//	 	Sample Window	: 15 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN6, 15U);
	ADC_setInterruptSOCTrigger(myADCA_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 1 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 1
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN3
	//	 	Sample Window	: 15 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN3, 15U);
	ADC_setInterruptSOCTrigger(myADCA_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 2 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 2
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN2
	//	 	Sample Window	: 15 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN2, 15U);
	ADC_setInterruptSOCTrigger(myADCA_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 3 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 3
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN9
	//	 	Sample Window	: 15 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADCA_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN9, 15U);
	ADC_setInterruptSOCTrigger(myADCA_BASE, ADC_SOC_NUMBER3, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 4 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 4
	//	  	Trigger			: ADC_TRIGGER_SW_ONLY
	//	  	Channel			: ADC_CH_ADCIN12
	//	 	Sample Window	: 64 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADCA_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN12, 64U);
	ADC_setInterruptSOCTrigger(myADCA_BASE, ADC_SOC_NUMBER4, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 5 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 5
	//	  	Trigger			: ADC_TRIGGER_SW_ONLY
	//	  	Channel			: ADC_CH_ADCIN5
	//	 	Sample Window	: 64 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADCA_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN5, 64U);
	ADC_setInterruptSOCTrigger(myADCA_BASE, ADC_SOC_NUMBER5, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 6 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 6
	//	  	Trigger			: ADC_TRIGGER_SW_ONLY
	//	  	Channel			: ADC_CH_ADCIN1
	//	 	Sample Window	: 64 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADCA_BASE, ADC_SOC_NUMBER6, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN1, 64U);
	ADC_setInterruptSOCTrigger(myADCA_BASE, ADC_SOC_NUMBER6, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 7 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 7
	//	  	Trigger			: ADC_TRIGGER_SW_ONLY
	//	  	Channel			: ADC_CH_ADCIN11
	//	 	Sample Window	: 64 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADCA_BASE, ADC_SOC_NUMBER7, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN11, 64U);
	ADC_setInterruptSOCTrigger(myADCA_BASE, ADC_SOC_NUMBER7, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 8 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 8
	//	  	Trigger			: ADC_TRIGGER_SW_ONLY
	//	  	Channel			: ADC_CH_ADCIN0
	//	 	Sample Window	: 64 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADCA_BASE, ADC_SOC_NUMBER8, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 64U);
	ADC_setInterruptSOCTrigger(myADCA_BASE, ADC_SOC_NUMBER8, ADC_INT_SOC_TRIGGER_NONE);
}

void myADCC_init(){
	//
	// ADC Initialization: Write ADC configurations and power up the ADC
	//
	// Set the analog voltage reference selection and ADC module's offset trims.
	// This function sets the analog voltage reference to internal (with the reference voltage of 1.65V or 2.5V) or external for ADC
	// which is same as ASysCtl APIs.
	//
	ADC_setVREF(myADCC_BASE, ADC_REFERENCE_EXTERNAL, ADC_REFERENCE_2_5V);
	//
	// Configures the analog-to-digital converter module prescaler.
	//
	ADC_setPrescaler(myADCC_BASE, ADC_CLK_DIV_2_0);
	//
	// Sets the timing of the end-of-conversion pulse
	//
	ADC_setInterruptPulseMode(myADCC_BASE, ADC_PULSE_END_OF_ACQ_WIN);
	//
	// Sets the timing of early interrupt generation.
	//
	ADC_setInterruptCycleOffset(myADCC_BASE, 0U);
	//
	// Powers up the analog-to-digital converter core.
	//
	ADC_enableConverter(myADCC_BASE);
	//
	// Delay for 1ms to allow ADC time to power up
	//
	DEVICE_DELAY_US(500);
	//
	// SOC Configuration: Setup ADC EPWM channel and trigger settings
	//
	// Disables SOC burst mode.
	//
	ADC_disableBurstMode(myADCC_BASE);
	//
	// Sets the priority mode of the SOCs.
	//
	ADC_setSOCPriority(myADCC_BASE, ADC_PRI_ALL_ROUND_ROBIN);
	//
	// Start of Conversion 0 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 0
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN6
	//	 	Sample Window	: 15 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN6, 15U);
	ADC_setInterruptSOCTrigger(myADCC_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 1 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 1
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN14
	//	 	Sample Window	: 15 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN14, 15U);
	ADC_setInterruptSOCTrigger(myADCC_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 2 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 2
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN11
	//	 	Sample Window	: 15 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADCC_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN11, 15U);
	ADC_setInterruptSOCTrigger(myADCC_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 3 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 3
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN10
	//	 	Sample Window	: 15 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADCC_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN10, 15U);
	ADC_setInterruptSOCTrigger(myADCC_BASE, ADC_SOC_NUMBER3, ADC_INT_SOC_TRIGGER_NONE);
}


//*****************************************************************************
//
// ASYSCTL Configurations
//
//*****************************************************************************
void ASYSCTL_init(){
	//
	// asysctl initialization
	//
	// Disables the temperature sensor output to the ADC.
	//
	ASysCtl_disableTemperatureSensor();
	//
	// Set the analog voltage reference selection to external.
	//
	ASysCtl_setAnalogReferenceExternal( ASYSCTL_VREFHIA | ASYSCTL_VREFHIC );
}

//*****************************************************************************
//
// GPIO Configurations
//
//*****************************************************************************
void GPIO_init(){
	CH5_B_MCU_init();
	CH5_A_MCU_init();
	CH4_A_MCU_init();
	CH4_B_MCU_init();
	CH1_A_MCU_init();
	CH1_B_MCU_init();
	CH2_A_MCU_init();
	CH2_B_MCU_init();
	CH6_B_MCU_init();
	CH6_A_MCU_init();
	CH7_A_MCU_init();
	CH7_B_MCU_init();
	CH8_A_MCU_init();
	CH3_B_MCU_init();
	CH3_A_MCU_init();
	CH8_B_MCU_init();
	Pulser_EN_init();
	Analog_EN_init();
	MODE0_init();
	MODE1_init();
	MCU_LED_init();
	AFE_GAIN_A1_init();
	AFE_GAIN_A2_init();
	AFE_GAIN_B1_init();
	AFE_GAIN_B2_init();
	THP_init();
	BOOT0_init();
	BOOT1_init();
	EEPROM_WC_init();
}

void CH5_B_MCU_init(){
	GPIO_setPadConfig(CH5_B_MCU, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CH5_B_MCU, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CH5_B_MCU, GPIO_DIR_MODE_OUT);
}
void CH5_A_MCU_init(){
	GPIO_setPadConfig(CH5_A_MCU, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CH5_A_MCU, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CH5_A_MCU, GPIO_DIR_MODE_OUT);
}
void CH4_A_MCU_init(){
	GPIO_setPadConfig(CH4_A_MCU, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CH4_A_MCU, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CH4_A_MCU, GPIO_DIR_MODE_OUT);
}
void CH4_B_MCU_init(){
	GPIO_setPadConfig(CH4_B_MCU, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CH4_B_MCU, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CH4_B_MCU, GPIO_DIR_MODE_OUT);
}
void CH1_A_MCU_init(){
	GPIO_setPadConfig(CH1_A_MCU, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CH1_A_MCU, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CH1_A_MCU, GPIO_DIR_MODE_OUT);
}
void CH1_B_MCU_init(){
	GPIO_setPadConfig(CH1_B_MCU, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CH1_B_MCU, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CH1_B_MCU, GPIO_DIR_MODE_OUT);
}
void CH2_A_MCU_init(){
	GPIO_setPadConfig(CH2_A_MCU, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CH2_A_MCU, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CH2_A_MCU, GPIO_DIR_MODE_OUT);
}
void CH2_B_MCU_init(){
	GPIO_setPadConfig(CH2_B_MCU, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CH2_B_MCU, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CH2_B_MCU, GPIO_DIR_MODE_OUT);
}
void CH6_B_MCU_init(){
	GPIO_setPadConfig(CH6_B_MCU, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CH6_B_MCU, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CH6_B_MCU, GPIO_DIR_MODE_OUT);
}
void CH6_A_MCU_init(){
	GPIO_setPadConfig(CH6_A_MCU, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CH6_A_MCU, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CH6_A_MCU, GPIO_DIR_MODE_OUT);
}
void CH7_A_MCU_init(){
	GPIO_setPadConfig(CH7_A_MCU, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CH7_A_MCU, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CH7_A_MCU, GPIO_DIR_MODE_OUT);
}
void CH7_B_MCU_init(){
	GPIO_setPadConfig(CH7_B_MCU, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CH7_B_MCU, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CH7_B_MCU, GPIO_DIR_MODE_OUT);
}
void CH8_A_MCU_init(){
	GPIO_setPadConfig(CH8_A_MCU, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CH8_A_MCU, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CH8_A_MCU, GPIO_DIR_MODE_OUT);
}
void CH3_B_MCU_init(){
	GPIO_setPadConfig(CH3_B_MCU, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CH3_B_MCU, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CH3_B_MCU, GPIO_DIR_MODE_OUT);
}
void CH3_A_MCU_init(){
	GPIO_setPadConfig(CH3_A_MCU, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CH3_A_MCU, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CH3_A_MCU, GPIO_DIR_MODE_OUT);
}
void CH8_B_MCU_init(){
	GPIO_setPadConfig(CH8_B_MCU, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CH8_B_MCU, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CH8_B_MCU, GPIO_DIR_MODE_OUT);
}
void Pulser_EN_init(){
	GPIO_setPadConfig(Pulser_EN, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(Pulser_EN, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(Pulser_EN, GPIO_DIR_MODE_OUT);
}
void Analog_EN_init(){
	GPIO_setPadConfig(Analog_EN, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(Analog_EN, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(Analog_EN, GPIO_DIR_MODE_OUT);
}
void MODE0_init(){
	GPIO_setPadConfig(MODE0, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(MODE0, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(MODE0, GPIO_DIR_MODE_OUT);
}
void MODE1_init(){
	GPIO_setPadConfig(MODE1, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(MODE1, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(MODE1, GPIO_DIR_MODE_OUT);
}
void MCU_LED_init(){
	GPIO_setPadConfig(MCU_LED, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(MCU_LED, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(MCU_LED, GPIO_DIR_MODE_OUT);
}
void AFE_GAIN_A1_init(){
	GPIO_setPadConfig(AFE_GAIN_A1, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(AFE_GAIN_A1, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(AFE_GAIN_A1, GPIO_DIR_MODE_OUT);
}
void AFE_GAIN_A2_init(){
	GPIO_setPadConfig(AFE_GAIN_A2, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(AFE_GAIN_A2, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(AFE_GAIN_A2, GPIO_DIR_MODE_OUT);
}
void AFE_GAIN_B1_init(){
	GPIO_setPadConfig(AFE_GAIN_B1, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(AFE_GAIN_B1, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(AFE_GAIN_B1, GPIO_DIR_MODE_OUT);
}
void AFE_GAIN_B2_init(){
	GPIO_setPadConfig(AFE_GAIN_B2, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(AFE_GAIN_B2, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(AFE_GAIN_B2, GPIO_DIR_MODE_OUT);
}
void THP_init(){
	GPIO_setPadConfig(THP, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(THP, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(THP, GPIO_DIR_MODE_IN);
}
void BOOT0_init(){
	GPIO_setPadConfig(BOOT0, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(BOOT0, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(BOOT0, GPIO_DIR_MODE_IN);
}
void BOOT1_init(){
	GPIO_setPadConfig(BOOT1, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(BOOT1, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(BOOT1, GPIO_DIR_MODE_IN);
}
void EEPROM_WC_init(){
	GPIO_setPadConfig(EEPROM_WC, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(EEPROM_WC, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(EEPROM_WC, GPIO_DIR_MODE_OUT);
}

//*****************************************************************************
//
// I2C Configurations
//
//*****************************************************************************
void I2C_init(){
	myI2CA_init();
	myI2CB_init();
}

void myI2CA_init(){
	I2C_disableModule(myI2CA_BASE);
	I2C_initController(myI2CA_BASE, DEVICE_SYSCLK_FREQ, myI2CA_BITRATE, I2C_DUTYCYCLE_33);
	I2C_setConfig(myI2CA_BASE, I2C_CONTROLLER_SEND_MODE);
	I2C_disableLoopback(myI2CA_BASE);
	I2C_setOwnAddress(myI2CA_BASE, myI2CA_OWN_ADDRESS);
	I2C_setTargetAddress(myI2CA_BASE, myI2CA_TARGET_ADDRESS);
	I2C_setBitCount(myI2CA_BASE, I2C_BITCOUNT_1);
	I2C_setDataCount(myI2CA_BASE, 1);
	I2C_setAddressMode(myI2CA_BASE, I2C_ADDR_MODE_7BITS);
	I2C_enableFIFO(myI2CA_BASE);
	I2C_setFIFOInterruptLevel(myI2CA_BASE, I2C_FIFO_TXEMPTY, I2C_FIFO_RXEMPTY);
	I2C_setEmulationMode(myI2CA_BASE, I2C_EMULATION_STOP_SCL_LOW);
	I2C_enableModule(myI2CA_BASE);
}
void myI2CB_init(){
	I2C_disableModule(myI2CB_BASE);
	I2C_configureModuleFrequency(myI2CB_BASE, DEVICE_SYSCLK_FREQ);
	I2C_setConfig(myI2CB_BASE, I2C_TARGET_RECEIVE_MODE);
	I2C_setOwnAddress(myI2CB_BASE, myI2CB_OWN_ADDRESS);
	I2C_setTargetAddress(myI2CB_BASE, myI2CB_TARGET_ADDRESS);
	I2C_setBitCount(myI2CB_BASE, I2C_BITCOUNT_8);
	I2C_setDataCount(myI2CB_BASE, 1);
	I2C_setAddressMode(myI2CB_BASE, I2C_ADDR_MODE_7BITS);
	I2C_disableFIFO(myI2CB_BASE);
	I2C_clearInterruptStatus(myI2CB_BASE, I2C_INT_ADDR_TARGET | I2C_INT_ARB_LOST | I2C_INT_NO_ACK | I2C_INT_REG_ACCESS_RDY | I2C_INT_RX_DATA_RDY | I2C_INT_STOP_CONDITION | I2C_INT_TX_DATA_RDY);
	I2C_enableInterrupt(myI2CB_BASE, I2C_INT_ADDR_TARGET | I2C_INT_ARB_LOST | I2C_INT_NO_ACK | I2C_INT_REG_ACCESS_RDY | I2C_INT_RX_DATA_RDY | I2C_INT_STOP_CONDITION | I2C_INT_TX_DATA_RDY);
	I2C_setEmulationMode(myI2CB_BASE, I2C_EMULATION_STOP_SCL_LOW);
	I2C_enableModule(myI2CB_BASE);
}

//*****************************************************************************
//
// INTERRUPT Configurations
//
//*****************************************************************************
void INTERRUPT_init(){
	
	// Interrupt Settings for INT_myI2CB
	// ISR need to be defined for the registered interrupts
	Interrupt_register(INT_myI2CB, &INT_myI2CB_ISR);
	Interrupt_enable(INT_myI2CB);
}
