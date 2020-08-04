/**
 * 
 * @file adchal.h
 * 
 * @brief  Hardware Abstraction Layer for Analog Data Converters
 *
 *
 */
#ifndef ADCHAL_H_
#define ADCHAL_H_

/**
 * @addtogroup group_adc
 *
 * @{
 */

//****************************************************************************
//
// Standard libraries
//
//****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/**
 * @brief Describes the ADC Characteristics structure
 */
typedef struct ADCcharDef {
	uint8_t		 resolution;		// ADC bit resolution (8, 10, 12, 14, 16, 18, 24, 32)
	uint32_t	 inputMuxConfReg;	// ADC input mux register configuration
	uint32_t	 pgaGain;			// ADC PGA linear gain setting (1x, 2x, 4x, 8x, 16x, etc.)
	uint32_t	 pgaReg;			// ADC PGA register configuration
	uint32_t 	 offsetCalCoeff;	// ADC Offset Calibration Coefficient (ideal ADC has offsetCalCoeff = 0)
	uint32_t 	 gainCalCoeff;		// ADC Gain Calibration Coefficient (ideal ADC has gainCalCoeff = 1)
	uint32_t 	 sampleRate;		// ADC sample rate in samples/sec
	uint32_t 	 dataRateReg;		// ADC data rate register configuration
	float	 	 Vref;				// ADC Voltage reference
	uint32_t	 refSelReg;			// ADC Reference selection register configuration
	uint32_t	 startupDel;		// ADC Startup Delay: Delay between powering up ADC channel and starting a conversion
	uint32_t	 burnoutCur;		// ADC Burnout Current Source
	uint32_t	 IDACmuxReg;		// ADC IDAC mux selection register configuration
	uint32_t	 IDACmagReg;		// ADC IDAC magnitude register configuration
	uint32_t     VBIASReg;			// ADC VBIAS register configuration
	uint32_t	 staleness;			// ADC staleness of last reading
	int32_t	 	 adcValue1;			// ADC conversion result provided as sign extended 2's complement (first one)
	int32_t	 	 adcValue2;			// ADC conversion result provided as sign extended 2's complement (second one)
} ADCchar_Set;

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************

bool    InitADCPeripherals( ADCchar_Set *adcChars);
bool    ReconfigureADC( ADCchar_Set *adcChars );
void 	InitGPIO( void );
void 	InitSPI( void );
void    setSTART (GPIO_PinState state );
void    setPWDN( bool state );
#ifdef nCS_PIN_ENABLED
void 	setCS( bool state );
#endif
void    toggleSTART( void );
void    toggleRESET( void );
void 	sendSTART( );
void 	sendSTOP( );
void 	sendRESET( );
void 	sendWakeup( );
void 	spiSendReceiveArrays( uint8_t DataTx[], uint8_t DataRx[], uint8_t byteLength );
uint8_t spiSendReceiveByte( uint8_t dataTx );
bool    waitForDRDYHtoL( uint32_t timeout_ms );

// Functions used for testing only
bool    getCS(void);
bool    getPWDN(void);
GPIO_PinState    getRESET(void);
GPIO_PinState    getSTART(void);
void    setRESET(GPIO_PinState state);


//*****************************************************************************
//
// Macros
//
//*****************************************************************************
/** Alias used for setting GPIOs pins to the logic "high" state */
#define HIGH                ((bool) true)

/** Alias used for setting GPIOs pins to the logic "low" state */
#define LOW                 ((bool) false)

// Flag to indicate if an interrupt occured
extern volatile bool flag_nDRDY_INTERRUPT;
extern SPI_HandleTypeDef hspi1;
extern __IO uint32_t wTransferState;
extern volatile uint8_t tempValueReady, speedValueReady;
extern float rtdTemp;
extern uint8_t initDone;

extern __IO uint32_t uwTick;
// extern unsigned long long usTimerPeriodCounter;

/** @} // group group_adc
 *
 */

#endif /* ADCHAL_H_ */
