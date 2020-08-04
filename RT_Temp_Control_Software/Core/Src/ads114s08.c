/**
 * 
 * @file ads114s08.c
 *
 * @brief  ADS114S08 Low level routines
 * 
 */ 


#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#include <assert.h>
#include "ads114s08.h"
// #include "ads124s08.h"
#include "adchal.h"
#define EMULATE
//****************************************************************************
//
// Internal variables
//
//****************************************************************************

/* Internal register map array (to recall current configuration) */
static uint8_t registerMap[NUM_REGISTERS];
static int32_t i = 0;

// Temperature profile 1
#define temProfileLowVal1  0x5000 // ~-150 degC
#define temProfileHighVal1  0x12F57 // ~140 degC

// Temperature profile 2
#define temProfileLowVal2  0xBBF3 // ~-10 degC
#define temProfileHighVal2 0x12F57 // ~140 degC

int32_t adcCount = temProfileLowVal2;
//****************************************************************************
//
// Functions
//
//****************************************************************************

/************************************************************************************//**
 *
 * @brief getRegisterValue()
 *          Getter function to access the registerMap array outside of this module
 *
 * @param[in]	address	The 8-bit register address
 *
 * @return 		The 8-bit register value
 */
uint8_t getRegisterValue( uint8_t address )
{
    assert( address < NUM_REGISTERS );
    return registerMap[address];
}

#ifdef EMULATE
// ADC Emulation function
int32_t adcCountEmulate()
{

	i++;
	//increment temperature for every 10 calls
	if((i % 10 == 0) && (adcCount < temProfileHighVal2))
	{
		adcCount = adcCount + 100;
	}

	return adcCount;
}
#endif
/************************************************************************************//**
 *
 * @brief adcStartupRoutine()
 *          Startup function to be called before communicating with the ADC
 *
 * @param[in]   *adcChars  ADC characteristics
 * @param[in]   *    SPI_Handle pointer for TI Drivers
 *
 * @return      true for successful initialization
 *              false for unsuccessful initialization
 */
 bool adcStartupRoutine( ADCchar_Set *adcChars )
{
    uint8_t initRegisterMap[NUM_REGISTERS] = { 0 };
    uint8_t status, i;
    // Provide additional delay time for power supply settling
    delay_us( DELAY_2p2MS );

    // Toggle nRESET pin to assure default register settings.
    toggleRESET();
    // Must wait 4096 tCLK after reset
    delay_us( DELAY_4096TCLK );

    status = readSingleRegister( REG_ADDR_STATUS );
    if ( (status & ADS_nRDY_MASK) ) {
        return( false );                      // Device not ready
    }

    // Ensure internal register array is initialized
    restoreRegisterDefaults();

    // Configure initial device register settings here 
    initRegisterMap[REG_ADDR_STATUS]  = 0x00;                      // Reset POR event
    initRegisterMap[REG_ADDR_INPMUX]  = adcChars->inputMuxConfReg;  
    initRegisterMap[REG_ADDR_PGA]     = adcChars->pgaReg; 
    initRegisterMap[REG_ADDR_DATARATE]= adcChars->dataRateReg;
    initRegisterMap[REG_ADDR_REF]     = adcChars->refSelReg;
    initRegisterMap[REG_ADDR_IDACMAG] = adcChars->IDACmagReg;
    initRegisterMap[REG_ADDR_IDACMUX] = adcChars->IDACmuxReg;
    initRegisterMap[REG_ADDR_VBIAS]   = adcChars->VBIASReg;
    initRegisterMap[REG_ADDR_SYS]     = SYS_DEFAULT;

    // Initialize ADC Characteristics
    adcChars->resolution     = ADS114S08_BITRES;
    adcChars->VBIASReg       = VBIAS_DEFAULT;
    adcChars->offsetCalCoeff = 0;
    adcChars->gainCalCoeff   = 1;
    adcChars->sampleRate     = 20;      // 20 samples per second
    adcChars->pgaGain        = pow(2, (adcChars->pgaReg & ADS_GAIN_MASK) );

    // Write to all modified registers
    writeMultipleRegisters(REG_ADDR_STATUS, REG_ADDR_SYS - REG_ADDR_STATUS + 1, initRegisterMap );

    // Read back all registers
    readMultipleRegisters(REG_ADDR_ID, NUM_REGISTERS );
    for ( i = REG_ADDR_STATUS; i < REG_ADDR_SYS - REG_ADDR_STATUS + 1; i++ ) {
        if ( i == REG_ADDR_STATUS )
            continue;
        if ( initRegisterMap[i] != registerMap[i] )
            return( false );
    }
    return( true );
}



/************************************************************************************//**
 *
 * @brief changeADCParameters()
 *          Change ADC parameters 
 *
 * @param[in]   *adcChars  ADC characteristics
 * @param[in]   *    SPI_Handle pointer for TI Drivers
 *
 * @return      true for successful initialization
 *              false for unsuccessful initialization
 */
 bool changeADCParameters( ADCchar_Set *adcChars )
{
    uint8_t initRegisterMap[NUM_REGISTERS] = { 0 };
    uint8_t i;
 
    // Reconfigure initial device register settings here 
    initRegisterMap[REG_ADDR_INPMUX]  = adcChars->inputMuxConfReg;  
    initRegisterMap[REG_ADDR_PGA]     = adcChars->pgaReg; 
    initRegisterMap[REG_ADDR_DATARATE]= adcChars->dataRateReg;
    initRegisterMap[REG_ADDR_REF]     = adcChars->refSelReg;
    initRegisterMap[REG_ADDR_IDACMAG] = adcChars->IDACmagReg;
    initRegisterMap[REG_ADDR_IDACMUX] = adcChars->IDACmuxReg;
    initRegisterMap[REG_ADDR_VBIAS]   = adcChars->VBIASReg;

    // Initialize ADC Characteristics based
    adcChars->resolution     = ADS114S08_BITRES;
    adcChars->VBIASReg       = VBIAS_DEFAULT;
    adcChars->offsetCalCoeff = 0;
    adcChars->gainCalCoeff   = 1;
    adcChars->sampleRate     = 20;      // 20 samples per second
    adcChars->pgaGain        = pow(2, (adcChars->pgaReg & ADS_GAIN_MASK) );

    // Write to all modified registers
    writeMultipleRegisters( REG_ADDR_INPMUX, REG_ADDR_VBIAS - REG_ADDR_INPMUX + 1, initRegisterMap );

    // Read back all registers
    readMultipleRegisters( REG_ADDR_ID, NUM_REGISTERS );
    for ( i = REG_ADDR_INPMUX; i < REG_ADDR_VBIAS - REG_ADDR_INPMUX + 1; i++ ) {
        if ( i == REG_ADDR_STATUS )
            continue;
        if ( initRegisterMap[i] != registerMap[i] )
            return( false );
    }
    return( true );
}



/************************************************************************************//**
 *
 * @brief readSingleRegister()
 *          Reads contents of a single register at the specified address
 *
 * @param[in]     SPI_Handle from TI Drivers
 * @param[in]	address Address of the register to be read
 *
 * @return 		8-bit register contents
 */
uint8_t readSingleRegister(  uint8_t address )
{
    /* Initialize arrays */
    uint8_t DataTx[COMMAND_LENGTH + 1] = { OPCODE_RREG | (address & OPCODE_RWREG_MASK), 0, 0 };
    uint8_t DataRx[COMMAND_LENGTH + 1] = { 0, 0, 0 };


    /* Check that the register address is in range */
    assert( address < NUM_REGISTERS );

    /* Build TX array and send it */
    spiSendReceiveArrays( DataTx, DataRx, COMMAND_LENGTH + 1);

    /* Update register array and return read result*/
    registerMap[address] = DataRx[COMMAND_LENGTH];
    return DataRx[COMMAND_LENGTH];
}

/************************************************************************************//**
 *
 * @brief readMultipleRegisters()
 *          Reads a group of registers starting at the specified address
 *          NOTE: Use getRegisterValue() to retrieve the read values
 *
 * @param[in]             SPI_Handle from TI Drivers
 * @param[in]	startAddress	Register address to start reading
 * @param[in]	count 			Number of registers to read
 *
 * @return 		None
 */
void readMultipleRegisters(  uint8_t startAddress, uint8_t count )
{
    uint8_t DataTx[COMMAND_LENGTH + NUM_REGISTERS] = { 0 };
    uint8_t DataRx[COMMAND_LENGTH + NUM_REGISTERS] = { 0 };
    uint8_t i;

    /* Check that the register address and count are in range */
    assert( startAddress + count <= NUM_REGISTERS );

    DataTx[0] = OPCODE_RREG | (startAddress & OPCODE_RWREG_MASK);
    DataTx[1] = count - 1;
    spiSendReceiveArrays( DataTx, DataRx, COMMAND_LENGTH + count);

    for ( i = 0; i < count; i++ ) {
        // Read register data bytes
        registerMap[i+startAddress] = DataRx[COMMAND_LENGTH + i];
    }
}

/************************************************************************************//**
 *
 * @brief writeSingleRegister()
 *          Write data to a single register at the specified address
 *
 * @param[in]         SPI_Handle from TI Drivers
 * @param[in]	address 	Register address to write
 * @param[in]	data 		8-bit data to write
 *
 * @return 		None
 */
void writeSingleRegister(  uint8_t address, uint8_t data )
{
    /* Initialize arrays */
    uint8_t DataTx[COMMAND_LENGTH + 1] = { OPCODE_WREG | (address & OPCODE_RWREG_MASK), 0, data};
    uint8_t DataRx[COMMAND_LENGTH + 1] = { 0 };

    /* Check that the register address is in range */
    assert( address < NUM_REGISTERS );

    /* Build TX array and send it */
    spiSendReceiveArrays( DataTx, DataRx, COMMAND_LENGTH + 1 );

    /* Update register array */
    registerMap[address] = DataTx[COMMAND_LENGTH];
}



/************************************************************************************//**
 *
 * @brief writeMultipleRegisters()
 *          Write data to a group of registers
 *          NOTES: Use getRegisterValue() to retrieve the written values.
 *          Registers should be re-read after a write operation to ensure proper configuration.
 *
 * @param[in]             SPI_Handle from TI Drivers
 * @param[in]	startAddress 	Register address to start writing
 * @param[in]	count 			Number of registers to write
 * @param[in]	regData			Array that holds the data to write, where element zero 
 *      						is the data to write to the starting address.
 *
 * @return 		None
 */
void writeMultipleRegisters(  uint8_t startAddress, uint8_t count, uint8_t regData[] )
{
    uint8_t DataTx[COMMAND_LENGTH + NUM_REGISTERS] = { 0 };
    uint8_t DataRx[COMMAND_LENGTH + NUM_REGISTERS] = { 0 };
    uint8_t i, j = 0;

    /* Check that the register address and count are in range */
    assert( startAddress + count <= NUM_REGISTERS );

    /* Check that regData is not a NULL pointer */
    assert( regData );

    DataTx[0] = OPCODE_WREG | (startAddress & OPCODE_RWREG_MASK);
    DataTx[1] = count - 1;
    for ( i = startAddress; i < startAddress + count; i++ ) {
        DataTx[2 + j++] = regData[i];
        registerMap[i] = regData[i];
    }

    // SPI communication
    spiSendReceiveArrays( DataTx, DataRx, COMMAND_LENGTH + count );
}



/************************************************************************************//**
 *
 * @brief sendCommand()
 *          Sends the specified SPI command to the ADC
 *
 * @param[in]         SPI_Handle from TI Drivers
 * @param[in]	op_code 	SPI command byte
 *
 * @return 		None
 */
void sendCommand( uint8_t op_code)
{
    /* Assert if this function is used to send any of the following commands */
    assert( OPCODE_RREG         != op_code );    /* Use "readSingleRegister()"  or "readMultipleRegisters()"  */
    assert( OPCODE_WREG         != op_code );    /* Use "writeSingleRegister()" or "writeMultipleRegisters()" */

    /* SPI communication */
    spiSendReceiveByte( op_code );

    // Check for RESET command
    if (OPCODE_RESET == op_code)
    {
        /* Update register array to keep software in sync with device */
        restoreRegisterDefaults();
    }
}


/************************************************************************************//**
 *
 * @brief startConversions()
 *        	Wakes the device from power-down and starts continuous conversions
 *            by setting START pin high or sending START Command
 *
 * @param[in]         SPI_Handle from TI Drivers
 *
 * @return 		None
 */
void startConversions(  )
{
	// Wakeup device
    sendWakeup(  );

#ifdef ADC_START_GPIO_PIN
     /* Begin continuous conversions */
    setSTART( HIGH );
#else
    sendSTART( );
#endif    
}

/************************************************************************************//**
 *
 * @brief stopConversions()
 *          Stops continuous conversions by setting START pin low or sending STOP Command
 *
 * @param[in]         SPI_Handle from TI Drivers
 *
 * @return      None
 */
void stopConversions()
{
     /* Stop continuous conversions */
#ifdef ADC_START_GPIO_PIN
    setSTART( LOW );
#else
    sendSTOP();
#endif    
}



/************************************************************************************//**
 *
 * @brief resetADC()
 *          Resets ADC by setting RESET pin low or sending RESET Command
 *
 * @param[in]         SPI_Handle from TI Drivers
 *
 * @return      None
 */
void resetADC(  )
{
     /* Reset ADC */
#ifdef RESET_PIN
    toggleRESET();
#else
    sendRESET();
#endif    
}

/************************************************************************************//**
 *
 * @brief readData()
 *          Sends the read command and retrieves STATUS (if enabled) and data
 *          NOTE: Call this function after /DRDY goes low and specify the 
 *          the number of bytes to read and the starting position of data
 *          
 * @param[in]         SPI_Handle from TI Drivers
 * @param[in]	status[] 	Pointer to location where STATUS byte will be stored
 * @param[in]	mode 		Direct or Command read mode
 * 
 * @return 		32-bit sign-extended conversion result (data only)
 */
int32_t readConvertedData( uint8_t status[], readMode mode )
{
#ifdef EMULATE
	return adcCountEmulate();
#else
    uint8_t DataTx[STATUS_LENGTH + DATA_LENGTH + CRC_LENGTH + 1] = { 0 };    // Initialize all array elements to 0
    uint8_t DataRx[STATUS_LENGTH + DATA_LENGTH + CRC_LENGTH + 1] = { 0 };    
    uint8_t byteLength;
    uint8_t dataPosition;
    uint8_t byte_options;
    bool    status_byte_enabled = 0;
	int32_t signByte, upperByte, middleByte, lowerByte;

    // Status Byte is sent if SENDSTAT bit of SYS register is set
    byte_options = IS_SENDSTAT_SET << 1 | IS_CRC_SET;
    switch ( byte_options ) {
    	case 0:							// No STATUS and no CRC
			byteLength   = DATA_LENGTH;
			dataPosition = 0;
    		break;
    	case 1:							// No STATUS and CRC
    		byteLength 	 = DATA_LENGTH + CRC_LENGTH;
			dataPosition = 0;
   			break;
    	case 2:							// STATUS and no CRC
    		byteLength   = STATUS_LENGTH + DATA_LENGTH;
			dataPosition = 1;
			status_byte_enabled = 1;
    		break;
    	case 3:							// STATUS and CRC
    		byteLength   = STATUS_LENGTH + DATA_LENGTH + CRC_LENGTH;
			dataPosition = 1;
			status_byte_enabled = 1;
   			break;
    }
	
	if ( mode == COMMAND ) {
        DataTx[0]     = OPCODE_RDATA;
        byteLength   += 1;
        dataPosition += 1;
	}
	// Todo: must uncomment
    spiSendReceiveArrays( DataTx, DataRx, byteLength );

    // Parse returned SPI data
    /* Check if STATUS byte is enabled and if we have a valid "status" memory pointer */
    if ( status_byte_enabled && status ) {
        status[0] = DataRx[dataPosition - 1];
    }

    /* Return the 32-bit sign-extended conversion result */
    if ( DataRx[dataPosition] & 0x80u ) {
    	signByte = 0xFF000000; 
    } else { 
    	signByte = 0x00000000; 
    }

	upperByte 	= ((int32_t) DataRx[dataPosition] & 0xFF) << 16;
	middleByte  = ((int32_t) DataRx[dataPosition + 1] & 0xFF) << 8;
	lowerByte	= ((int32_t) DataRx[dataPosition + 2] & 0xFF);

	return ( signByte + upperByte + middleByte + lowerByte );		// Basically the code value 7FFFF to 80000 hexadecimal
#endif
}



/************************************************************************************//**
 *
 * @brief restoreRegisterDefaults()
 *          Updates the registerMap[] array to its default values
 *          NOTES: If the MCU keeps a copy of the ADC register settings in memory,
 *          then it is important to ensure that these values remain in sync with the
 *          actual hardware settings. In order to help facilitate this, this function
 *          should be called after powering up or resetting the device (either by
 *          hardware pin control or SPI software command).
 *          Reading back all of the registers after resetting the device will
 *          accomplish the same result.
 *
 * @return 		None
 */
 void restoreRegisterDefaults( void )
{
	/* Default register settings */
	registerMap[REG_ADDR_ID]       = ID_DEFAULT;
	registerMap[REG_ADDR_STATUS]   = STATUS_DEFAULT;
	registerMap[REG_ADDR_INPMUX]   = INPMUX_DEFAULT;
	registerMap[REG_ADDR_PGA]      = PGA_DEFAULT;
	registerMap[REG_ADDR_DATARATE] = DATARATE_DEFAULT;
	registerMap[REG_ADDR_REF]      = REF_DEFAULT;
	registerMap[REG_ADDR_IDACMAG]  = IDACMAG_DEFAULT;
	registerMap[REG_ADDR_IDACMUX]  = IDACMUX_DEFAULT;
	registerMap[REG_ADDR_VBIAS]    = VBIAS_DEFAULT;
	registerMap[REG_ADDR_SYS]      = SYS_DEFAULT;
	registerMap[REG_ADDR_OFCAL0]   = OFCAL0_DEFAULT;
	registerMap[REG_ADDR_OFCAL1]   = OFCAL1_DEFAULT;
	registerMap[REG_ADDR_OFCAL2]   = OFCAL2_DEFAULT;
	registerMap[REG_ADDR_FSCAL0]   = FSCAL0_DEFAULT;
	registerMap[REG_ADDR_FSCAL1]   = FSCAL1_DEFAULT;
	registerMap[REG_ADDR_FSCAL2]   = FSCAL2_DEFAULT;
	registerMap[REG_ADDR_GPIODAT]  = GPIODAT_DEFAULT;
	registerMap[REG_ADDR_GPIOCON]  = GPIOCON_DEFAULT;
}
