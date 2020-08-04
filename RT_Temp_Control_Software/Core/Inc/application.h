/*
 * application.h
 *
 *  Created on: 21-Jul-2020
 *      Author: Rakesh
 */

#ifndef APPLICATION_USER_APPLICATION_H_
#define APPLICATION_USER_APPLICATION_H_


#include "stdint.h"
#include "stdbool.h"
#include "main.h"
#include "ads114s08.h"
#include "math.h"

/********************************************************************************//**
 *
 * @name Board Resistance
 */
//#define R68 1000
#define R68 33000
#define R70 1000


//#define	isnan(x) (__fp_type_select((x), __isnanf, __isnan, __isnanl))

/********************************************************************************//**
 *
 * @name START Command instead of START pin
 */
#undef START_PIN	// Use START and STOP commands instead of START START_PIN

/********************************************************************************//**
 *
 * @name RESET Command instead of RESET pin
 */
#undef RESET_PIN	// Use START and STOP commands instead of START START_PIN

/********************************************************************************//**
 *
 * @name RTD Connections for 2-Wire (Figure 15 of ADS124S08 EVM User's Guide) on J7
 *
 * AIN0 <- AINN
 * AIN1 <- AIN1
 * AIN5 -> IDAC1
 * AIN6 <- REFP1
 * AIN7 <- REFN1
 *
 */
#define RTD_VBIAS				VBIAS_DEFAULT
#define RTD_TWO_WIRE_INPUT_MUX	ADS_P_AIN1 | ADS_N_AIN0;           							// MuxP = AIN1, MuxN = AIN0
#define RTD_TWO_WIRE_PGA		ADS_PGA_ENABLED | ADS_GAIN_2;      							// PGA enabled, PGA gain = 2X
#define RTD_TWO_WIRE_DATARATE	ADS_CONVMODE_CONT | ADS_FILTERTYPE_LL | ADS_DR_20;   		// Continous conversion, low latency filter, 20 SPS rate
#define RTD_TWO_WIRE_REF_SEL	ADS_REFN_BYP_ENABLE | ADS_REFSEL_P1 | ADS_REFINT_ON_ALWAYS;	// RefN Enabled, RefP1 and RefN1 selected, Int Ref always on
#define RTD_TWO_WIRE_IDACMAG	ADS_IDACMAG_1000;                  							// IDAC Mag 1 mA
#define RTD_TWO_WIRE_IDACMUX	ADS_IDAC2_OFF | ADS_IDAC1_A5;                      			// IDAC1 Mux = Ain5, IDAC2 off
#define RTD_TWO_WIRE_REF_RES	R68
#define RTD_TWO_WIRE_EXT_VREF	0.001 * RTD_TWO_WIRE_REF_RES

/********************************************************************************//**
 *
 * @name RTD Connections for 3-Wire (Figure 14 of ADS124S08 EVM User's Guide) on J7
 *
 * AIN1 <- AINP
 * AIN2 <- AINN
 * AIN3 -> IDAC2
 * AIN5 -> IDAC1
 * AIN6 <- REFP1
 * AIN7 <- REFN1
 *
 */
#define RTD_THREE_WIRE_INPUT_MUX	ADS_P_AIN1 | ADS_N_AIN2;           						 	// MuxP = AIN1, MuxN = AIN2
#define RTD_THREE_WIRE_INPUT_MUX2	ADS_P_AIN2 | ADS_N_AIN3;           						 	// MuxP = AIN2, MuxN = AIN3
#define RTD_THREE_WIRE_PGA			ADS_PGA_ENABLED | ADS_GAIN_2;      						 	// PGA enabled, PGA gain = 2X
#define RTD_THREE_WIRE_DATARATE  	ADS_CONVMODE_CONT | ADS_FILTERTYPE_LL | ADS_DR_20;   		// Continous conversion, low latency filter, 20 SPS rate
#define RTD_THREE_WIRE_REF_SEL   	ADS_REFN_BYP_ENABLE | ADS_REFSEL_P1 | ADS_REFINT_ON_ALWAYS; // RefN Enabled, RefP1 and RefN1 selected, Int Ref always on
#define RTD_THREE_WIRE_IDACMAG   	ADS_IDACMAG_1000;                  						 	// IDAC Mag 1 mA
#define RTD_THREE_WIRE_IDACMUX   	ADS_IDAC2_A3 | ADS_IDAC1_A5;                      		 	// IDAC Mux = AIN5, IDAC2 Mux = AIN3
#define RTD_THREE_WIRE_REF_RES		R68
#define RTD_THREE_WIRE_EXT_VREF		0.001 * RTD_THREE_WIRE_REF_RES


/********************************************************************************//**
 *
 * @name RTD Connections for 4-Wire (Figure 16 of ADS124S08 EVM User's Guide) on J7
 *
 * AIN2 <- AINP
 * AIN4 <- AINN
 * AIN5 -> IDAC1
 * AIN6 <- REFP1
 * AIN7 <- REFN1
 *
 */
#define RTD_FOUR_WIRE_INPUT_MUX	ADS_P_AIN2 | ADS_N_AIN4;           						 	// MuxP = AIN2, MuxN = AIN4
#define RTD_FOUR_WIRE_PGA		ADS_PGA_ENABLED | ADS_GAIN_2;      						 	// PGA enabled, PGA gain = 2X
#define RTD_FOUR_WIRE_DATARATE	ADS_CONVMODE_CONT | ADS_FILTERTYPE_LL | ADS_DR_20;   		// Continous conversion, low latency filter, 20 SPS rate
#define RTD_FOUR_WIRE_REF_SEL	ADS_REFN_BYP_ENABLE | ADS_REFSEL_P1 | ADS_REFINT_ON_ALWAYS; // RefN Enabled, RefP1 and RefN1 selected, Int Ref always on
#define RTD_FOUR_WIRE_IDACMAG	ADS_IDACMAG_1000;                  						 	// IDAC Mag 1 mA
#define RTD_FOUR_WIRE_IDACMUX	ADS_IDAC2_OFF | ADS_IDAC1_A5;                      		 	// IDAC Mux = AIN5, IDAC2 off
#define RTD_FOUR_WIRE_REF_RES	R68
#define RTD_FOUR_WIRE_INT_VREF	INT_VREF


/********************************************************************************//**
 *
 * @name Thermocouple Connections (Figure 17 of ADS124S08 EVM User's Guide)
 *
 * AIN0 <- AINN Cold Junction
 * AIN1 <- AINP Cold Junction
 * AIN2 <- AINP Thermocouple
 * AIN4 <- AINN Thermocouple
 * AIN5 -> IDAC1
 * AIN6 <- REFP1
 * AIN7 <- REFN1
 *
 */
#define TC_VBIAS		VBIAS_DEFAULT
#define TC1_INPUT_MUX	ADS_P_AIN2 | ADS_N_AIN4;           						 	// MuxP = Ain2, MuxN = Ain4 for Thermocouple
#define CJ1_INPUT_MUX	ADS_P_AIN1 | ADS_N_AIN0;           						 	// MuxP = Ain1, MuxN = Ain0 for Cold Junction
#define TC1_PGA     	ADS_PGA_ENABLED | ADS_GAIN_2;      						 	// PGA enabled, PGA gain = 2X
#define TC1_DATARATE  	ADS_CONVMODE_CONT | ADS_FILTERTYPE_LL | ADS_DR_20;   		// Continous conversion, low latency filter, 20 SPS rate
#define TC1_REF_SEL   	ADS_REFN_BYP_ENABLE | ADS_REFSEL_INT | ADS_REFINT_ON_ALWAYS; // RefN Enabled, RefP1 and RefN1 selected, Int Ref always on
#define TC1_IDACMAG   	ADS_IDACMAG_OFF;                  						 	// IDAC Mag off
#define TC1_IDACMUX   	ADS_IDAC2_OFF | ADS_IDAC1_OFF;                      		// IDAC1 off, IDAC2 off
#define TC1_INT_VREF	INT_VREF


/********************************************************************************//**
 *
 * @name Bridge Thermocouple Connections (Figure 18 of ADS124S08 EVM User's Guide)
 *
 * AIN0 <- AINN Cold Junction
 * AIN1 <- AINP Cold Junction
 * AIN2 <- AINP Thermocouple
 * AIN4 <- AINN Thermocouple
 * AIN5 -> IDAC1
 * AIN6 <- REFP1
 * AIN7 <- REFN1
 *
 */
#define TC2_INPUT_MUX	ADS_P_AIN2 | ADS_N_AIN4;           						 	// MuxP = Ain2, MuxN = Ain4 for Thermocouple
#define CJ2_INPUT_MUX	ADS_P_AIN1 | ADS_N_AIN0;           						 	// MuxP = Ain1, MuxN = Ain0 for Cold Junction
#define TC2_PGA     	ADS_PGA_ENABLED | ADS_GAIN_2;      							// PGA enabled, PGA gain = 2X
#define TC2_DATARATE  	ADS_CONVMODE_CONT | ADS_FILTERTYPE_LL | ADS_DR_20;   		// Continous conversion, low latency filter, 20 SPS rate
#define TC2_REF_SEL   	ADS_REFN_BYP_ENABLE | ADS_REFSEL_P1 | ADS_REFINT_ON_ALWAYS; // RefN Enabled, RefP1 and RefN1 selected, Int Ref always on
#define TC2_IDACMAG   	ADS_IDACMAG_OFF;                  						 	// IDAC Mag off
#define TC2_IDACMUX   	ADS_IDAC2_OFF | ADS_IDAC1_OFF;                      		// IDAC1 off, IDAC2 off
#define TC2_INT_VREF	INT_VREF


/********************************************************************************//**
 *
 * @name Thermocouple Connections (Figure 20 of ADS124S08 EVM User's Guide)
 *
 * AINCOM <- AINN Cold Junction
 * AIN10  <- AINP Cold Junction
 * AIN8   <- AINP Thermocouple
 * AIN9   <- AINN Thermocouple
 *
 */
#define TC3_INPUT_MUX	ADS_P_AIN8 | ADS_N_AIN9;           						 	// MuxP = Ain8, MuxN = Ain9 for Thermocouple
#define CJ3_INPUT_MUX	ADS_P_AINCOM | ADS_N_AIN10;           						// MuxP = AinCom, MuxN = Ain10 for Cold Junction
#define TC3_PGA     	ADS_PGA_ENABLED | ADS_GAIN_2;      							// PGA enabled, PGA gain = 2X
#define TC3_DATARATE  	ADS_CONVMODE_CONT | ADS_FILTERTYPE_LL | ADS_DR_20;   		// Continous conversion, low latency filter, 20 SPS rate
#define TC3_REF_SEL   	ADS_REFN_BYP_ENABLE | ADS_REFSEL_INT | ADS_REFINT_ON_ALWAYS; // RefN Enabled, Internal Reference, Int Ref always on
#define TC3_IDACMAG   	ADS_IDACMAG_OFF;                  						 	// IDAC Mag off
#define TC3_IDACMUX   	ADS_IDAC2_OFF | ADS_IDAC1_OFF;                      		// IDAC1 off, IDAC2 off
#define TC3_INT_VREF	INT_VREF


/********************************************************************************//**
 *
 * @name Thermistor Connections (Figure 19 of ADS124S08 EVM User's Guide) on J8
 *
 * REFP0 <- VREF
 * REFN0 <- GND
 * AIN8 <- AINP
 * AIN9 <- AINN
 *
 */
#define THERMISTOR_VBIAS			VBIAS_DEFAULT
#define THERMISTOR_INPUT_MUX    	ADS_P_AIN8 | ADS_N_AIN9;           						 		// MuxP = AIN8, MuxN = AIN9
#define THERMISTOR_WIRE_PGA     	ADS_PGA_ENABLED | ADS_GAIN_1;      						 		// PGA enabled, PGA gain = 1X
#define THERMISTOR_WIRE_DATARATE 	ADS_CONVMODE_CONT | ADS_FILTERTYPE_LL | ADS_DR_20;   			// Continous conversion, low latency filter, 20 SPS rate
#define THERMISTOR_WIRE_REF_SEL 	ADS_REFN_BYP_ENABLE | ADS_REFSEL_INT | ADS_REFINT_ON_ALWAYS; 	// Internal Ref, Int Ref always on
#define THERMISTOR_WIRE_IDACMAG 	ADS_IDACMAG_OFF;                  						 		// IDAC Mag off
#define THERMISTOR_WIRE_IDACMUX 	ADS_IDAC2_OFF | ADS_IDAC1_OFF;                      		 	// IDAC1 off, IDAC2 off
#define THERMISTOR_VREF				INT_VREF
#define THERMISTOR_RBIAS			2959.0

#endif /* APPLICATION_USER_APPLICATION_H_ */



/********************************************************************************//**
  *
  * @brief DRDY GPIO Pin
  */
#define DRDY_GPIO_PIN                  GPIO_PIN_8
#define DRDY_GPIO_PORT                 GPIOC
#define DRDY_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()
#define DRDY_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOC_CLK_DISABLE()
#define DRDY_EXTI_IRQn                 EXTI15_10_IRQn
#define DRDY_EXTI_LINE                 EXTI_LINE_13


