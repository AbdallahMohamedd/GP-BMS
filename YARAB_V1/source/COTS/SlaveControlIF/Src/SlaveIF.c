/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Abdullah Mohamed
 *	Component:  Slave_Control_IF driver
 *	File: 		SlaveIF.c
 */



#include <COTS/SlaveControlIF/Inc/SlaveIF.h>
#include <COTS/SlaveControlIF/Inc/SlaveIF_Cfg.h>


/* ============================= SPI Communication APIs ============================= */
static uint8_t calculate_crc8(uint8_t *data, uint16_t data_len) {
	uint8_t crc = 0x42;  // Seed
	for (uint16_t i = 0; i < data_len; i++) {
		uint8_t tbl_idx = (crc ^ data[i]) & 0xff;
		crc = (crc_table[tbl_idx] ^ (crc << 8)) & 0xff;
	}
	return crc;
}




static uint8_t checkcrc8(uint8_t *data, uint16_t data_len)  {
	uint8_t tbl_idx;
	uint8_t crc;

	crc = 0x42;				// seed

	while (data_len--) {
		tbl_idx = ((crc >> 0) ^ *data) & 0xff;
		crc = (crc_table[tbl_idx] ^ (crc << 8)) & 0xff;

		data++;
	}
	return crc & 0xff;
}




volatile bool spiTransferCompleted = false;
spi_master_handle_t spiHandle;

/* Callback Function */
// Callback Function (Called when the non-blocking SPI transfer is complete)
static void SPI_Callback(SPI_Type *base, spi_master_handle_t *handle, status_t status, void *userData)
{
	if (status == kStatus_Success)
	{
		// SPI transfer completed successfully
		PRINTF("SPI Transfer Callback - Success! Status: %d\n\r\r", status);
		spiTransferCompleted = true;
	}
	else
	{
		// SPI transfer failed
		//PRINTF("SPI Transfer Failed! Status: %d\n\r\r", status);
		spiTransferCompleted = true; // Ensure the flag is set even on failure to prevent infinite loop
	}
}

spi_master_callback_t SPICallback = SPI_Callback;




/* SPI Initialization */
void SlaveIF_TransferInit(void)
{

SPI_Type *base = SPI_used;
#ifdef DebugInfoManager
	DebugInfo;
#endif
	spi_master_config_t masterConfig;

	// Get default configuration
	SPI_MasterGetDefaultConfig(&masterConfig);

	// Enable SPI clock based on the selected SPI module
	if (base == SPI0)
	{
		CLOCK_EnableClock(kCLOCK_Spi0);
		NVIC_EnableIRQ(SPI0_IRQn);
		NVIC_SetPriority(SPI0_IRQn, 2);
	}
	else if (base == SPI1)
	{
		CLOCK_EnableClock(kCLOCK_Spi1);
		NVIC_EnableIRQ(SPI1_IRQn);
		NVIC_SetPriority(SPI1_IRQn, 2);
	}

	// Initialize SPI with configuration
	SPI_MasterInit(base, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	// Create SPI handle and set callback (optional for non-blocking transmit only)
	SPI_MasterTransferCreateHandle(base, &spiHandle, SPI_Callback, NULL);

	// Enable specific SPI interrupts as needed
	//SPI_EnableInterrupts(base, kSPI_TxEmptyInterruptEnable); // Enable only TxEmpty interrupt
}





static void SlaveIF_SlaveWriteRegister(uint8_t regAddress, uint16_t data)
{
	SPI_Type *base = SPI_used;
	uint8_t txFrame[BUFFERSIZE] = {0};
	uint8_t rxFrame[BUFFERSIZE] = {0}; // Rxframe is not used, but it needs to be passed to the transfer function
#ifdef DebugInfoManager
	DebugInfo;
#endif
	// Construct the SPI frame
	txFrame[0] = 0x00;                      		// CRC (Placeholder)
	txFrame[1] = 0x02;                      		// Write command + Cluster ID
	txFrame[2] = (regAddress & 0x7F) | 0x80;        // Memory Address + Master/Slave Bit
	txFrame[3] = (data >> 8) & 0xFF;          		// Data High Byte
	txFrame[4] = data & 0xFF;                   	// Data Low Byte


	// Prepare SPI transfer structure
	spi_transfer_t spiXfer;
	spiXfer.txData = txFrame;
	spiXfer.rxData = rxFrame; 						// Rxframe is not used, but it needs to be passed to the transfer function
	spiXfer.dataSize = sizeof(txFrame);

	// Initiate non-blocking SPI transfer
	spiTransferCompleted = false; 					// Reset the transfer completion flag

	status_t status = SPI_MasterTransferNonBlocking(base, &spiHandle, &spiXfer);
	//delay_us(100000);
	if (status != kStatus_Success)
	{
		PRINTF("SPI Transfer Failed to Start! Status: %d\n\r\r", status);
		return;
	}

	// Wait for transfer completion with timeout -->
	// feh mo4kla hna eno bee5rog mn el llop in line 117 due to timeout!!!
	uint32_t timeout = 1000000; 					// Adjust timeout value as needed
	while (!spiTransferCompleted && timeout--)
	{
		// Wait for the transfer to complete or timeout
	}

	if (!spiTransferCompleted)
	{
		PRINTF("SPI Write Timeout!\n\r\r");
	}
}





static uint16_t SlaveIF_SlaveReadRegister(uint8_t regAddress)
{
	SPI_Type *base = SPI_used;
	uint8_t txFrame[BUFFERSIZE] = {0};  			// Transmit frame (request)
	uint8_t rxFrame[BUFFERSIZE] = {0};  			// Receive buffer (response)
	uint16_t receivedData = 0;

	// Construct the 40-bit SPI frame for reading
	txFrame[0] = 0x00;                              // CRC (Placeholder)
	txFrame[1] = 0x01;                              // Read command (Bits[8:11]) + Cluster ID (Bits[12:15])
	txFrame[2] = (regAddress & 0x7F) | 0x80;        // Memory Address (Bits[16:22]) + Master/Slave Bit[23] (Set MSB for read)
	txFrame[3] = 0x00;                              // Placeholder for receiving data (Bits[24:31])
	txFrame[4] = 0x00;                              // Placeholder for receiving data (Bits[32:39])

	// Prepare SPI transfer structure
	spi_transfer_t spiXfer;
	spiXfer.txData = txFrame;
	spiXfer.rxData = rxFrame;
	spiXfer.dataSize = sizeof(txFrame);

	// Initiate non-blocking SPI transfer
	spiTransferCompleted = false;  // Reset flag

	status_t status = SPI_MasterTransferNonBlocking(base, &spiHandle, &spiXfer);

	if (status != kStatus_Success)
	{
		PRINTF("SPI Read Failed to Start! Status: %d\n\r\r", status);
		return 0; // Or some other error value
	}
	uint32_t timeout = 1000000;  // Adjust timeout value as needed
	while (!spiTransferCompleted && timeout--)
	{
		// Wait for the transfer to complete or timeout
	}

	if (!spiTransferCompleted)
	{
		PRINTF("SPI Read Timeout!\n\r\r");
		return 0;
	}
	// Extract the received 16-bit data from the response frame
	receivedData = (rxFrame[3] << 8) | rxFrame[4];

	// Print the received frame after reading
	//PRINTF("SPI Frame received (READ): ");
	for (int i = 0; i < 5; i++)
	{
		//PRINTF("%02X ", rxFrame[i]);  // Print each byte in HEX format
	}
	//PRINTF("\n");

	return receivedData;
}














/* ============================= System Configuration APIs ============================= */
// Function to set the initialization register
void SlaveIF_SystemSetup(bool channel_switch)
{
	uint8_t Reg_Address = INIT_REGISTER;
	uint16_t data = 0x0000; // Start with a zero value

	// Set the CID value from the macro
	data |= (DEFAULT_CID & 0x000F); // Ensure we only use the lower 4 bits

	// Set bit 4 (BUS_SW)
	(channel_switch) ? (SET_BIT(data, 4)) : (RESET_BIT(data, 4));

	// Send SPI write command to INIT register
	SlaveIF_SlaveWriteRegister(Reg_Address, data);
}


void SlaveIF_Go2SleepMode(bool Sleep_Mode)
{
	uint8_t Reg_Address = SYS_CFG_GLOBAL;
	uint8_t data = 0x0000; 			// Start with a zero value

	(Sleep_Mode) ? (SET_BIT(data, 0)) : (RESET_BIT(data, 0));

	// Send SPI write command
	SlaveIF_SlaveWriteRegister(Reg_Address, data);
}



void SlaveIF_SystemConfig1(void)
{
#ifdef DebugInfoManager
	DebugInfo;
#endif
	uint8_t Reg_Address = SYS_CFG1; // System Configuration Register 1
	uint16_t data = 0x9381;         // Configuration value
	//uint16_t data = 0x1111;		// Test data
	// Send SPI write command
	SlaveIF_SlaveWriteRegister(Reg_Address, data);
}



void SlaveIF_SystemConfig2(void)
{
	uint8_t Reg_Address = SYS_CFG2;  // System Configuration Register 1
	uint16_t data = 0x6231;

	// Send SPI write command
	SlaveIF_SlaveWriteRegister(Reg_Address, data);
}





/* ============================= ADC Configuration APIs ============================= */
// lesa
void SlaveIF_CfgADC(void)
{

}

// lesa
void SlaveIF_CfgADCOffset(void)
{


}



/* ============================= Voltage Fault Monitoring APIs ============================= */
void SlaveIF_EnableOVUV(void)
{
	uint8_t Reg_Address = OV_UV_EN_ADDR;  // OV/UV enable register address
	uint16_t data = 0xFFFF;        		  // Enable all cells if 1, disable if 0

	SlaveIF_SlaveWriteRegister(Reg_Address, data);
}



// return //0b00xxxxxxxx --> 1 = ov fault in order of this cell
uint16_t SlaveIF_ReadCellOverVoltageStatus(void)
{
	uint16_t cell_Over_Voltage_Flags = 0;
	uint16_t rawData =0;
	uint8_t Reg_Address = CELL_OV_FLT;
	// Read the CELL_OV_FLT register via SPI
	rawData = SlaveIF_SlaveReadRegister(Reg_Address);

	// Extract the relevant bits (bits 0 to 13)
	cell_Over_Voltage_Flags = rawData & 0x3FFF; //0b0011111111111111

	return cell_Over_Voltage_Flags;
}



uint16_t SlaveIF_ReadCellUnderVoltageStatus(void)
{
	uint8_t Reg_Address = CELL_UV_FLT;
	uint16_t cell_Under_Voltage_Flags = 0;

	// Read the CELL_OV_FLT register via SPI
	uint16_t rawData = SlaveIF_SlaveReadRegister(Reg_Address);

	// Extract the relevant bits (bits 0 to 13)
	cell_Under_Voltage_Flags = rawData & 0x3FFF; //0b0011111111111111

	return cell_Under_Voltage_Flags;
}



/* ============================= Cell Balancing APIs ============================= */
void SlaveIF_EnableCellBalancing(uint8_t cellNumber, bool enable, float Timer_Value_In_Minutes)
{
	uint8_t Reg_Address;
	uint16_t data = 0x0000; // Start with zero

	// Select the correct register address
	switch (cellNumber)
	{
	case 1: Reg_Address  = CB1_CFG_ADDR ; break;
	case 2: Reg_Address  = CB2_CFG_ADDR ; break;
	case 3: Reg_Address  = CB3_CFG_ADDR ; break;
	case 4: Reg_Address  = CB4_CFG_ADDR ; break;
	case 5: Reg_Address  = CB5_CFG_ADDR ; break;
	case 6: Reg_Address  = CB6_CFG_ADDR ; break;
	case 7: Reg_Address  = CB7_CFG_ADDR ; break;
	case 8: Reg_Address  = CB8_CFG_ADDR ; break;
	case 9: Reg_Address  = CB9_CFG_ADDR ; break;
	case 10: Reg_Address = CB10_CFG_ADDR; break;
	case 11: Reg_Address = CB11_CFG_ADDR; break;
	case 12: Reg_Address = CB12_CFG_ADDR; break;
	case 13: Reg_Address = CB13_CFG_ADDR; break;
	case 14: Reg_Address = CB14_CFG_ADDR; break;
	default:	// Invalid cell number
		return; // Do nothing
	}

	// Configure the data
	(enable) ? (SET_BIT(data,9)) : (RESET_BIT(data,9)); // Set CB_EN bit

	// Convert the float timer value to an integer number of "half-minute" units
	uint16_t Timer_Value_In_Half_Minutes = (uint16_t)(Timer_Value_In_Minutes / 0.5);

	// Check if the timer value exceeds the valid range
	if (Timer_Value_In_Half_Minutes > CB_CFG_DURATION_MASK)
		Timer_Value_In_Half_Minutes = CB_CFG_DURATION_MASK; // Limit timer value to the maximum

	// Set the timer value (ensure it's within the valid range)
	data |= Timer_Value_In_Half_Minutes & CB_CFG_DURATION_MASK; // Apply the mask and set the timer value

	// Write the data to the register
	SlaveIF_SlaveWriteRegister(Reg_Address, data);
}



void SlaveIF_ReadCellBalancingDriverStatus(void)
{
	uint8_t Reg_Address = CB_DRV_STS_ADDR;
	uint16_t Driver_Status = SlaveIF_SlaveReadRegister(Reg_Address);

	PRINTF("Cell Balancing Driver Status:\n");

	// Iterate through each cell (1 to 14)
	for (int i = 1; i <= 14; i++)
	{
		// Calculate the bit position (adjusting for 0-based indexing in the code)
		int bit_Position = i - 1;  // CB1_STS is at bit 0, CB2_STS is at bit 1, and so on.
		uint8_t is_Enabled = READ_BIT(Driver_Status, bit_Position);

		PRINTF("Cell %d: ", i);
		if (is_Enabled)
			PRINTF("ON\n");
		else
			PRINTF("OFF\n");
	}
}



uint16_t SlaveIF_ReadCellBalancingShortedStatus(void)
{
	uint8_t Reg_Address = CB_SHORT_FLT_ADDR;
	uint16_t Shorted_Status = SlaveIF_SlaveReadRegister(Reg_Address);
	return Shorted_Status;
}



uint16_t SlaveIF_ReadCellBalancingOpenLoadStatus(void)
{
	uint8_t Reg_Address = CB_OPEN_FLT_ADDR;
	uint16_t Open_Load_Status = SlaveIF_SlaveReadRegister(Reg_Address);
	return Open_Load_Status;
}



/* ============================= GPIO Configuration APIs ============================= */
void SlaveIF_CfgAllGpiosForTempSensors(void)
{
	uint8_t Reg_Address = GPIO_CFG1_ADDR;
	uint16_t gpioCfg1Value = 0x0000;  // Initialize to 0 (POR value)

	// Loop through all GPIO pins (0 to 6)
	for (int gpioNumber = 0; gpioNumber <= 7; gpioNumber++)
	{
		// Set the GPIO as analog input for temperature (ratiometric)
		gpioCfg1Value |= (0b00 << (gpioNumber * 2));
	}

	// Write the data to the register
	SlaveIF_SlaveWriteRegister(Reg_Address, gpioCfg1Value);
}


/* ============================= GPIO flag APIs ============================= */
GPIO_AN_Flags SlaveIF_ReadGpioAnStatus(void)
{
	uint8_t Reg_Address = GPIO_SH_AN_OL_STS_ADDR;

	GPIO_AN_Flags flags;
	uint16_t faultStatus = SlaveIF_SlaveReadRegister(Reg_Address);

	// Extract the GPIO short bits (bits 8 to 15)
	flags.GPIO_SH_Flag = (uint8_t)((faultStatus & 0x7F00) >> 8);

	// Extract the AN open load bits (bits 0 to 7)
	flags.AN_OL_Flags = (uint8_t)(faultStatus & 0x7F);

	return flags;
}


/* ============================= Temp APIs ============================= */
Temperature_Flags SlaveIF_ReadOtUttatus(void)
{
	uint8_t Reg_Address = AN_OT_UT_FLT_STS_ADDR;
	Temperature_Flags flags;
	uint16_t faultStatus = SlaveIF_SlaveReadRegister(Reg_Address);

	// Extract the over temperature bits (bits 8 to 15)
	flags.Over_Temp_Flags = (uint8_t)((faultStatus & 0x7F00) >> 8);

	// Extract the under temperature bits (bits 0 to 7)
	flags.Under_Temp_Flags = (uint8_t)(faultStatus & 0x7F);

	return flags;
}


/* ============================= Coulomb Counter & Threshold Configuration ============================= */
uint16_t SlaveIF_ReadNumberCoulombSamples(void)
{
	uint8_t Reg_Address = CC_NB_SAMPLES_ADDR;
	uint16_t no_Samples = SlaveIF_SlaveReadRegister(Reg_Address);
	return no_Samples;
}



bool SlaveIF_IsCoulombSamplesSufficient(void)
{

	uint16_t no_Samples = SlaveIF_ReadNumberCoulombSamples();
	if (no_Samples < 1)
	{
		return false;
	}
	return true;
}



/* ============================= Voltage, Current & Temperature Measurement APIs ============================= */
float SlaveIF_ReadCellVoltage(uint8_t cellNumber)
{
	uint8_t Reg_Address;

	switch (cellNumber)
	{
	case 1:  Reg_Address = MEAS_CELL1_ADDR; break;
	case 2:  Reg_Address = MEAS_CELL2_ADDR ; break;
	case 3:  Reg_Address = MEAS_CELL3_ADDR; break;
	case 4:  Reg_Address = MEAS_CELL4_ADDR ; break;
	case 5:  Reg_Address = MEAS_CELL5_ADDR; break;
	case 6:  Reg_Address = MEAS_CELL6_ADDR; break;
	case 7:  Reg_Address = MEAS_CELL7_ADDR; break;
	case 8:  Reg_Address = MEAS_CELL8_ADDR; break;
	case 9:  Reg_Address = MEAS_CELL9_ADDR; break;
	case 10: Reg_Address = MEAS_CELL10_ADDR; break;
	case 11: Reg_Address = MEAS_CELL11_ADDR; break;
	case 12: Reg_Address = MEAS_CELL12_ADDR; break;
	case 13: Reg_Address = MEAS_CELL13_ADDR; break;
	case 14: Reg_Address = MEAS_CELL14_ADDR; break;
	default:
		PRINTF("Invalid cell number.\n");
		return 0;
	}
	uint16_t Cell_Voltage = SlaveIF_SlaveReadRegister(Reg_Address);

	// Check if data is valid
	if (!(Cell_Voltage & MEAS_DATA_READY))
	{
		PRINTF("Cell Voltage Data Not Ready!\n");
		return 0; // Data not ready
	}
	float Voltage_in_uV = (float)Cell_Voltage * VCT_ANX_RES_V;  // To convert to microVolts
	return (Voltage_in_uV / 1000000); // to converts it to voltage
}



float SlaveIF_ReadStackVoltage(void)
{
	uint8_t Reg_Address = MEAS_STACK_ADDR;
	uint16_t Meas_Stack = SlaveIF_SlaveReadRegister(Reg_Address);

	// Check if data is ready
	if (!(Meas_Stack & MEAS_DATA_READY))
	{
		PRINTF("Stack Voltage Data Not Ready!\n");
		return -1.0; // Data not ready
	}

	// Extract the 15-bit voltage value
	uint16_t Raw_Voltage = Meas_Stack & 0x7FFF;

	// Convert to Volts
	float Voltage_In_Micro_Volts = (float)Raw_Voltage * VVPWR_RES;  //calculate voltage in microvolts
	float stack_Voltage = (Voltage_In_Micro_Volts / 1000000.0f);   // Convert from microvolts to Volts
	return stack_Voltage;
}
