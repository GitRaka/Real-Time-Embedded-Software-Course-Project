/**
 *
 * @brief Hardware Abstraction Layer for Data Converters using TI Drivers
 */ 
#include <stdbool.h>
#include <unistd.h>
#include <string.h>

/* Driver Header files */

/* Driver configuration */

#include "adchal.h"
#include "ads114s08.h"
// #include "ads124s08.h"
#include "main.h"

/**
 * @addtogroup group_adc
 *
 * @{
 */

//****************************************************************************
//
// Internal variables and macros
//
//****************************************************************************

// Flag to indicate if an interrupt occured
volatile bool flag_nDRDY_INTERRUPT;

/************************************************************************************//**
 *
 * @brief gpioDRDYfxn()
 *          MCU callback routine for ADC_DRDY falling edge interrupt
 *
 * @return None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* Set nDRDY flag to true */
    flag_nDRDY_INTERRUPT = true;
}

/************************************************************************************//**
 *
 * @brief InitADCPeripherals()
 *          Initialize MCU peripherals and pins to interface with ADC
 *
 * @param[in]   *adcChars  ADC characteristics
 * @param[in]   *spiHdl    SPI_Handle pointer for TI Drivers
 * 
 * @return      true for successful initialization
 *              false for unsuccessful initialization
 *
 * @code     
 *     if ( !InitADCPeripherals( &adcChars, &spiHdl ) ) {
 *        // Error initializing MCU SPI Interface
 *        Display_printf( displayHdl, 0, 0, "Error initializing master SPI\n" );
 *        return( false );
 *     }
 *     // MCU initialized ADC successfully
 * @endcode
 */

bool InitADCPeripherals( ADCchar_Set *adcChars )
{
    //SPI_Params      spiParams;
    bool            status;

    //SPI_Params_init( &spiParams );
    //spiParams.dataSize    = SPI_WORD_SIZE;
    //spiParams.frameFormat = SPI_POL0_PHA1;
    //spiParams.bitRate     = SPI_SPEED;

/*
    *spiHdl = SPI_open( ADC_SPI_0, &spiParams );
    if (*spiHdl == NULL) {
        Display_printf( displayHdl, 0, 0, "Error initializing master SPI\n" );
        while (1);
    }
    else {
        Display_printf( displayHdl, 0, 0, "Master SPI initialized\n" );
    }
*/
    // Start up the ADC
    status = adcStartupRoutine( adcChars );

    // DRDY interrupt configuration
    HAL_NVIC_DisableIRQ(DRDY_EXTI_IRQn);
    HAL_NVIC_EnableIRQ(DRDY_EXTI_IRQn);                           // enable Interrupt

    startConversions();                           // Start Conversions

    return( status );
}

/************************************************************************************//**
 *
 * @brief ReconfigureADC()
 *          Reconfigure ADC Parameters
 *
 * @param[in]   *adcChars  ADC characteristics
 * @param[in]   spiHdl    SPI_Handle pointer for TI Drivers
 * 
 * @return      true for successful initialization
 *              false for unsuccessful initialization
 *
 * @code     
 *    if ( !ReconfigureADC( &RTDadcChars, spiHdl ) ) {
 *        // Error reconfiguring ADC
 *           Display_printf( displayHdl, 0, 0, "Error reconfiguring ADC\n" );
 *           return( false );
 *      }
 *      // ADC reconfigured successfully
 *      // Read next conversion result
 *      if ( waitForDRDYHtoL( TIMEOUT_COUNTER ) ) {
 *          RTDadcChars.adcValue1 = readConvertedData( spiHdl, &status, COMMAND );
 *      } else {
 *          // Error reading conversion result
 *          Display_printf( displayHdl, 0, 0, "Timeout on conversion\n" );
 *          return( false );
 *      }
 * @endcode
 */

bool ReconfigureADC( ADCchar_Set *adcChars )
{
    bool    status;
    uint8_t dataTx = OPCODE_WAKEUP;

    // Stop ADC conversions
    stopConversions( );
    
    // Wakeup device
    sendCommand( dataTx );

    status = changeADCParameters( adcChars );

    // DRDY interrupt configuration
    // GPIO_clearInt( ADC_DRDY );
    // Set nDRDY flag to false
    flag_nDRDY_INTERRUPT = false;

    startConversions();                           // Start Conversions

    return( status );
}

/************************************************************************************//**
 *
 * @brief getRESET()
 *          Returns the state of the MCU's ADC_RESET GPIO pin
 *        
 * @return boolean level of /RESET pin (false = low, true = high)
 */

GPIO_PinState getRESET( void )
{
    //return (bool) GPIO_read( ADC_RESET );
	return HAL_GPIO_ReadPin(ADC_RESET_GPIO_PORT, ADC_RESET_GPIO_PIN);
}

/************************************************************************************//**
 *
 * @brief setRESET()
 *            Sets the state of the MCU ADC_RESET GPIO pin
 *
 * @param[in]   state   level of /RESET pin (false = low, true = high)
 *
 * @return      None
 */

void setRESET( GPIO_PinState state )
{
    //GPIO_write( ADC_RESET, (unsigned int) state );
    HAL_GPIO_WritePin(ADC_RESET_GPIO_PORT, ADC_RESET_GPIO_PIN, state);
}
/************************************************************************************//**
 *
 * @brief toggleRESET()
 *            Pulses the /RESET GPIO pin low
 *
 * @return      None
 */
void toggleRESET( void )
{
    HAL_GPIO_WritePin(ADC_RESET_GPIO_PORT, ADC_RESET_GPIO_PIN, GPIO_PIN_RESET);

    // Minimum nRESET width: 4 tCLKs = 4 * 1/4.096MHz = 
    delay_us( DELAY_4TCLK );

    //GPIO_write( ADC_RESET, (unsigned int) HIGH );
    HAL_GPIO_WritePin(ADC_RESET_GPIO_PORT, ADC_RESET_GPIO_PIN, GPIO_PIN_SET);
}
/************************************************************************************//**
 *
 * @brief getSTART()
 *          Returns the state of the MCU's ADC_START GPIO pin
 *        
 * @return boolean level of START pin (false = low, true = high)
 */

GPIO_PinState getSTART( void )
{
    //return (bool) GPIO_read( ADC_START );
    return HAL_GPIO_ReadPin(ADC_START_GPIO_PORT, ADC_START_GPIO_PIN);
}
/************************************************************************************//**
 *
 * @brief setSTART()
 *            Sets the state of the MCU START GPIO pin
 *
 * @param[in]   state   level of START pin (false = low, true = high)
 *
 * @return      None
 */
void setSTART( GPIO_PinState state )
{
    //GPIO_write( ADC_START, (unsigned int) state );
	HAL_GPIO_WritePin(ADC_START_GPIO_PORT, ADC_START_GPIO_PIN, state);

    // Minimum START width: 4 tCLKs
	delay_us ( DELAY_4TCLK );
}
/************************************************************************************//**
 *
 * @brief toggleSTART()
 *            Pulses the START GPIO pin low
 *
 * @return      None
 */
void toggleSTART( void )
{
    //GPIO_write( ADC_START, (unsigned int) LOW );
	 HAL_GPIO_WritePin(ADC_START_GPIO_PORT, ADC_START_GPIO_PIN, GPIO_PIN_RESET);

    // Minimum START width: 4 tCLKs
	 delay_us ( DELAY_4TCLK );
    
    HAL_GPIO_WritePin(ADC_START_GPIO_PORT, ADC_START_GPIO_PIN, GPIO_PIN_SET);
    
    // Minimum delay after reset: 4096 tCLKs
    delay_us ( DELAY_4096TCLK );
}
/************************************************************************************//**
 *
 * @brief sendSTART()
 *            Sends START Command through SPI
 *
 * @param[in]   spiHdl    SPI_Handle pointer for TI Drivers
 * 
 * @return      None
 */
void sendSTART()
{
    uint8_t dataTx = OPCODE_START;

    // Send START Command
    sendCommand( dataTx );
}
/************************************************************************************//**
 *
 * @brief sendSTOP()
 *            Sends STOP Command through SPI
 *
 * @param[in]   spiHdl    SPI_Handle pointer for TI Drivers
 * 
 * @return      None
 */
void sendSTOP( )
{
    uint8_t dataTx = OPCODE_STOP;

    // Send STOP Command
    sendCommand( dataTx );
}
/************************************************************************************//**
 *
 * @brief sendRESET()
 *            Sends RESET Command through SPI, then waits 4096 tCLKs
 *
 * @param[in]   spiHdl    SPI_Handle pointer for TI Drivers
 *
 * @return      None
 */

void sendRESET()
{
    uint8_t dataTx = OPCODE_RESET;

    // Send RESET command
    sendCommand( dataTx );
    
    // Minimum delay after reset: 4096 tCLKs
    delay_us( DELAY_4096TCLK );
}

/************************************************************************************//**
 *
 * @brief sendWakeup()
 *            Sends Wakeup Command through SPI
 *
 * @param[in]   spiHdl    SPI_Handle pointer for TI Drivers
 *
 * @return      None
 */

void sendWakeup()
{
    uint8_t dataTx = OPCODE_WAKEUP;
    
    // Wakeup device
    sendCommand( dataTx );
}

#ifdef nCS_PIN_ENABLED
/************************************************************************************//**
 *
 * @brief setCS()
 *            Sets the state of the "/CS" GPIO pin
 *
 * @param[in]   level   Sets the state of the "/CS" pin
 * 
 * @return      None
 */
void setCS( bool state )
{
    GPIO_write( ADC_CS, (unsigned int) state );
}
#endif
/************************************************************************************//**
 *
 * @brief waitForDRDYHtoL()
 *            Waits for a nDRDY GPIO to go from High to Low or until a timeout condition occurs
 *            The DRDY output line is used as a status signal to indicate
 *            when a conversion has been completed. DRDY goes low
 *            when new data is available. 
 *
 * @param[in]   timeout_ms number of milliseconds to allow until a timeout
 * 
 * @return      Returns true if nDRDY interrupt occurred before the timeout
 *
 * @code     
 *      // Read next conversion result
 *      if ( waitForDRDYHtoL( TIMEOUT_COUNTER ) ) {
 *          RTDadcChars.adcValue1 = readConvertedData( spiHdl, &status, COMMAND );
 *      } else {
 *          // Error reading conversion result
 *          Display_printf( displayHdl, 0, 0, "Timeout on conversion\n" );
 *          return( false );
 *      }
 * @endcode
 */
bool waitForDRDYHtoL( uint32_t timeout_ms )
{
    uint32_t timeoutCounter = timeout_ms * 6000;   // convert to # of loop iterations;

    do {
    } while ( !(flag_nDRDY_INTERRUPT) && (--timeoutCounter) );
    
    if ( !timeoutCounter ) {
        return false;
    } else {
        flag_nDRDY_INTERRUPT = false; // Reset flag
        return true;
    }
}

/************************************************************************************//**
 *
 * @brief spiSendReceiveArrays()
 *             Sends SPI commands to ADC and returns a response in array format
 *             
 * @param[in]   spiHdl      SPI_Handle from TI Drivers
 * @param[in]   *DataTx     array of SPI data to send on MOSI pin
 * @param[in]   *DataRx     array of SPI data that will be received from MISO pin
 * @param[in]   byteLength  number of bytes to send/receive on the SPI
 *
 * @return     None
 */

void spiSendReceiveArrays( uint8_t DataTx[], uint8_t DataRx[], uint8_t byteLength )
{
    //SPI_Transaction spiTransaction;

    //
    // This function sends and receives multiple bytes over the SPI.
    //
    // A typical SPI send/receive sequence may look like the following:
    // 1) Make sure SPI receive buffer is empty
    // 2) Set the /CS pin low (if controlled by GPIO)
    // 3) Send command bytes to SPI transmit buffer
    // 4) Wait for SPI receive interrupt
    // 5) Retrieve data from SPI receive buffer
    // 6) Set the /CS pin high (if controlled by GPIO)
    //
    //

    //spiTransaction.txBuf = DataTx;
    //spiTransaction.rxBuf = DataRx;

#ifdef nCS_PIN_ENABLED
        setCS( LOW );
#endif
    // Send or Receive Data
    if ( byteLength > 0 ) {
        //spiTransaction.count = byteLength;

        /*##-1- Start the Full Duplex Communication process ########################*/
        /* While the SPI in TransmitReceive process, user can transmit data through
           "DataTx" buffer & receive data through "DataRx" */
        if (HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)DataTx, (uint8_t *)DataRx, byteLength) != 0) //HAL_OK
        {
          /* Transfer error in transmission process */
        	while (1);
        }
        /*##-2- Wait for the end of the transfer ###################################*/
        //Can implement blocking here instead of wait
        while (wTransferState == TRANSFER_WAIT)
        {
        }
    }
#ifdef nCS_PIN_ENABLED
   setCS( HIGH );
#endif    
}

/************************************************************************************//**
 *
 * @brief spiSendReceiveByte()
 *             Sends a single byte to ADC and returns a response
 *             
 * @param[in]   spiHdl      SPI_Handle from TI Drivers
 * @param[in]   dataTx      byte to send on DIN pin
 *
 * @return     SPI response byte
 */

uint8_t spiSendReceiveByte( uint8_t dataTx )
{
    uint8_t         dataRx = 0;

#ifdef nCS_PIN_ENABLED
    setCS( LOW );
#endif
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, &dataTx, &dataRx, ONE) != 0) //HAL_OK
    {
      /* Transfer error in transmission process */
    	while (1);
    }

    /*##-2- Wait for the end of the transfer ###################################*/
    //Can implement blocking here instead of wait
    while (wTransferState == TRANSFER_WAIT)
    {
    }

#ifdef nCS_PIN_ENABLED
    setCS( HIGH );
#endif
    return( dataRx );
}
/** @} // group group_adc
 *
 */
