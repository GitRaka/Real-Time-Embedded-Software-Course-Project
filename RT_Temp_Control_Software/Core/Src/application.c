/*
 * application.c
 *
 *  Created on: 21-Jul-2020
 *      Author: Rakesh
 */
#include "application.h"
#include "main.h"
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include "rtd.h"
#include "rtd_tables.h"
#include "ads114s08.h"
//#include "ads124s08.h"
#define EMULATE

//extern TIM_HandleTypeDef htim7;

typedef enum RTDExampleDef{
    RTD_2_Wire_Fig15,     // 2-Wire RTD example using ADS124S08 EVM, User's Guide Figure 15
    RTD_3_Wire_Fig14,     // 3-Wire RTD example using ADS124S08 EVM, User's Guide Figure 14
    RTD_4_Wire_Fig16,     // 4-Wire RTD example using ADS124S08 EVM, User's Guide Figure 16
} RTD_Example;

ADCchar_Set adcChars, adcChars2;
float       rtdRes, rtdTemp;
RTD_Set     *rtdSet = NULL;
uint8_t     status;
uint32_t traceCount;
volatile uint8_t tempValueReady, speedValueReady;

uint8_t tempreadexcess = 0;
float tempreadexcesstempvalue = 0;


void emulateInit()
{
	// Initialize ADC Characteristics
	adcChars.resolution     = ADS124S08_BITRES;
	adcChars.VBIASReg       = VBIAS_DEFAULT;
	adcChars.offsetCalCoeff = 0;
	adcChars.gainCalCoeff   = 1;
	adcChars.sampleRate     = 20;      // 20 samples per second
	adcChars.pgaGain        = pow(2, (adcChars.pgaReg & ADS_GAIN_MASK) );
}

/************************************************************************************//**
 *
 * @brief readRTDtemp()
 *             Reads RTD temperature
 *
 * @return     none
 */
void readRTDtemp()
{
    RTD_Type    rtdType = Pt;
    RTD_Example rtdExample = RTD_4_Wire_Fig16;
    // ADCchar_Set adcChars, adcChars2;

    switch ( rtdType )
    {
        case Pt:
            rtdSet = &PT100_RTD;
            break;

        case Ni:
            break;

        case Cu:
            break;

        case NiFe:
            break;
    }

    switch ( rtdExample ) {
        case RTD_2_Wire_Fig15:
            adcChars.inputMuxConfReg = RTD_TWO_WIRE_INPUT_MUX;
            adcChars.pgaReg          = RTD_TWO_WIRE_PGA;
            adcChars.dataRateReg     = RTD_TWO_WIRE_DATARATE;
            adcChars.refSelReg       = RTD_TWO_WIRE_REF_SEL;
            adcChars.IDACmagReg      = RTD_TWO_WIRE_IDACMAG;
            adcChars.IDACmuxReg      = RTD_TWO_WIRE_IDACMUX;
            adcChars.Vref            = RTD_TWO_WIRE_EXT_VREF;
            rtdSet->Rref             = RTD_TWO_WIRE_REF_RES;
            rtdSet->wiring           = Two_Wire_High_Side_Ref;
            break;
        case RTD_3_Wire_Fig14:
            adcChars.inputMuxConfReg = RTD_THREE_WIRE_INPUT_MUX;
            adcChars.pgaReg          = RTD_THREE_WIRE_PGA;
            adcChars.dataRateReg     = RTD_THREE_WIRE_DATARATE;
            adcChars.refSelReg       = RTD_THREE_WIRE_REF_SEL;
            adcChars.IDACmagReg      = RTD_THREE_WIRE_IDACMAG;
            adcChars.IDACmuxReg      = RTD_THREE_WIRE_IDACMUX;
            adcChars.Vref            = RTD_THREE_WIRE_EXT_VREF;
            rtdSet->Rref             = RTD_THREE_WIRE_REF_RES;
            rtdSet->wiring           = Three_Wire_High_Side_Ref_Two_IDAC;
            break;
        case RTD_4_Wire_Fig16:
            adcChars.inputMuxConfReg = RTD_FOUR_WIRE_INPUT_MUX;
            adcChars.pgaReg          = RTD_FOUR_WIRE_PGA;
            adcChars.dataRateReg     = RTD_FOUR_WIRE_DATARATE;
            adcChars.refSelReg       = RTD_FOUR_WIRE_REF_SEL;
            adcChars.IDACmagReg      = RTD_FOUR_WIRE_IDACMAG;
            adcChars.IDACmuxReg      = RTD_FOUR_WIRE_IDACMUX;
            adcChars.Vref            = RTD_FOUR_WIRE_INT_VREF;
            rtdSet->Rref             = RTD_FOUR_WIRE_REF_RES;
            rtdSet->wiring           = Four_Wire_High_Side_Ref;
            break;
    }
    adcChars.VBIASReg = RTD_VBIAS;

    //add in device initialization

#ifdef EMULATE
	emulateInit();
#else
	if ( !InitADCPeripherals( &adcChars ) ) {
//        Display_printf( displayHdl, 0, 0, "Error initializing master SPI\n" );
        while (1);
    }
#endif
}

// The function that does the processing for the received count values from the ADC
// Separately provided for proper scheduling
void processInput()
{
	timeSpec timStart = {0, 0};
	timeSpec timStop = {0, 0};
	uint32_t timDelay = 0;

    BSP_LED_Off(LED2);
//        if ( waitForDRDYHtoL( TIMEOUT_COUNTER ) ) {
//            adcChars.adcValue1 = readConvertedData( //spiHdl,
//            		&status, COMMAND );

	// HAL_TIM_Base_Start(&htim7);
	// timStart = TIM7->CNT;
	getCurrentTime(&timStart);


	adcChars.adcValue1 = readConvertedData(&status, COMMAND);

	// Convert ADC values RTD resistance
	rtdRes = Convert_Code2RTD_Resistance( &adcChars, rtdSet);
	// Convert RTD resistance to temperature and linearize
	rtdTemp = RTD_Linearization( rtdSet, rtdRes );

	// Display_printf( displayHdl, 0, 0, "ADC conversion result 1: %i\n", adcChars.adcValue1 );
	if ( rtdSet->wiring == Three_Wire_High_Side_Ref_One_IDAC || rtdSet->wiring == Three_Wire_Low_Side_Ref_One_IDAC ) {
		//Display_printf( displayHdl, 0, 0, "ADC conversion result 2: %i\n", adcChars.adcValue2 );
	}

	if ( isnan(rtdTemp) ) {
		while (1);
	} else {
		//Display_printf( displayHdl, 0, 0, "RTD temperature: %.3f (C)\n\n", rtdTemp );
		tempValueReady = 1;
	}

// else {
//			//Display_printf( displayHdl, 0, 0, "Timeout on conversion\n" );
//			while (1);
//		}
// timStop = TIM7->CNT;

	//measure time elapsed by the task
	getCurrentTime(&timStop);
	timDelay = getTimeDiff(&timStop, &timStart);

	if (timDelay > 2000)
	{
		tempreadexcess = timDelay;
		tempreadexcesstempvalue = rtdTemp;
		__asm__("BKPT");
		// Process input task exceeded its deadline

	}

	//HAL_TIM_Base_Stop(&htim7);

}
