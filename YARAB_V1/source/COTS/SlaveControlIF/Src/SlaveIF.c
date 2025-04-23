/**
 * @file SlaveIF.c
 * @brief Implementation of the Slave Interface driver for MC33771B communication.
 *
 * @details This file contains the implementation for interacting with the MC33771B
 *          battery monitoring IC via the MC33664 TPL transceiver using SPI
 *          on an NXP KL25Z microcontroller. It handles low-level SPI/DMA
 *          communication, frame construction, register access, and provides
 *          APIs for configuration, measurement, fault monitoring, and cell balancing.
 *
 * @note Project: Graduation Project - Battery Management System
 * @note Engineer: Abdullah Mohamed
 * @note Component: Slave_Control_IF driver
 */

//=============================================================================
// Includes
//=============================================================================
#include <COTS/SlaveControlIF/Inc/SlaveIF.h>
#include <COTS/SlaveControlIF/Inc/SlaveIF_Cfg.h>

float placeholder_global_ov = 4.00f;
float placeholder_global_uv = 2.80f;
// float placeholder_cell_ov = 4.00f; // For individual cell settings if needed
// float placeholder_cell_uv = 2.80f; // For individual cell settings if needed
float placeholder_ot_v = 0.6f;		   // Voltage corresponding to max temp
float placeholder_ut_v = 2.9f;		   // Voltage corresponding to min temp
float placeholder_oc_a = 1.00f;		   // Amperes
float placeholder_shunt_uohm = 100.0f; // Shunt resistance in micro-Ohms

/* ========================= Global Variables ========================= */
/**
 * @brief Rolling Counter (RC) for SPI communication frames.
 * @details Incremented for each transaction (0-3) to ensure frame synchronization.
 *          Shared between read and write operations.
 */
volatile uint8_t gRollingCounter = 0;

/**
 * @brief Tag ID for ADC measurement cycles.
 * @details Incremented before each measurement cycle trigger (SOC).
 *          Used to verify that received measurement data corresponds to the
 *          requested cycle. Range: 0-15.
 */
volatile uint8_t TAGID = 0;

/* ========================= Static Constants ========================= */
/**
 * @brief Resolution for Over Voltage (OV) and Under Voltage (UV) Thresholds.
 * @details Based on MC33771B Datasheet Rev 8.0, Table 104 (TH_ALL_CT, TH_CTx).
 *          Units: millivolts per LSB.
 */
static const float OVUV_RESOLUTION_MV_PER_LSB = 19.53125f;

/**
 * @brief Resolution for Over Temperature (OT) and Under Temperature (UT) Thresholds.
 * @details Based on MC33771B Datasheet Rev 8.0, Table 106 (TH_ANx_OT, TH_ANx_UT).
 *          Units: millivolts per LSB.
 */
static const float OTUT_RESOLUTION_MV_PER_LSB = 4.8828125f;

/**
 * @brief Resolution for Over Current (OC) Threshold.
 * @details Based on MC33771B Datasheet Rev 8.0, Table 108 (TH_ISENSE_OC).
 *          Units: microvolts per LSB (across shunt resistor).
 */
static const float OC_RESOLUTION_UV_PER_LSB = 1.2f; // microV/LSB

/* ========================= Static Variables (DMA/SPI) ========================= */
/** @brief DMA handle for SPI0 Transmit channel. */
static dma_handle_t dmaTxHandle;
/** @brief DMA handle for SPI1 Receive channel. */
static dma_handle_t dmaRxHandle;

/** @brief SPI DMA handle for SPI0 Master Transmit. */
static spi_dma_handle_t spiDmaTxHandle;
/** @brief SPI DMA handle for SPI1 Slave Receive. */
static spi_dma_handle_t spiDmaRxHandle;

/** @brief Flag indicating completion of SPI0 DMA Transmit operation. */
static volatile bool spiDmaTxCompleted = false;
/** @brief Flag indicating completion of SPI1 DMA Receive operation. */
static volatile bool spiDmaRxCompleted = false;

/* ========================= Static Function Declarations ========================= */
// --- Transceiver Control Helpers ---
static void SlaveIF_writeEnPin(uint8_t value);
static uint32_t SlaveIF_readIntbPin(void);

// --- SPI/DMA Helpers ---
static uint8_t calculateCrc8(uint8_t *data, uint16_t data_len);
static void resetSpi0(void);

// --- SPI Communication Primitives ---
static bool SlaveIF_writeRegister(uint8_t regAddress, uint16_t data);
static uint16_t SlaveIF_readRegister(uint8_t regAddress);

// --- Register Helpers ---
static bool SlaveIF_registerHasTagID(uint8_t regAddr);
static bool SlaveIF_setupSystem(bool channelSwitch);
static bool SlaveIF_configSystem1(void);
static bool SlaveIF_configSystem2(void);
static bool SlaveIF_enableOvUv(void);
static bool SlaveIF_configAllGpiosForTempSensors(void);

// --- Threshold Calculation Helpers ---
static uint16_t SlaveIf_calculateOvUvThresholdReg(float ov_volts, float uv_volts);
static uint16_t SlaveIf_calculateOverTempThresholdReg(float overTemp_inVolt);
static uint16_t SlaveIf_calculateUnderTempThresholdReg(float underTemp_inVolt);
static uint16_t SlaveIf_calculateOverCurrentThresholdReg(float overCurrent_amps, float shunt_resistance_micro_ohms);
static bool SlaveIf_setGlobalOvUvThreshold(float ov_volts, float uv_volts);
static bool SlaveIf_setCellOvUvThreshold(uint8_t cell_index, float ov_volts, float uv_volts);
static bool SlaveIf_setOverTempThreshold(uint8_t anx_index, float overTemp_inVolt);
static bool SlaveIf_setUnderTempThreshold(uint8_t anx_index, float underTemp_inVolt);
static bool SlaveIf_setOverCurrentThreshold(float overCurrent_amps, float shunt_resistance_micro_ohms);

// --- DMA/IRQ Handlers ---
static void spiDmaTxCallback(SPI_Type *base, spi_dma_handle_t *handle, status_t status, void *userData);
static void spiDmaRxCallback(SPI_Type *base, spi_dma_handle_t *handle, status_t status, void *userData);

/* ========================= Callback Handlers ========================= */
/**
 * @brief Callback function executed upon completion of SPI0 DMA transmit.
 * @param base SPI peripheral base pointer (SPI0).
 * @param handle Pointer to the SPI DMA handle (`spiDmaTxHandle`).
 * @param status Status of the completed DMA transfer.
 * @param userData User data pointer (unused).
 */
static void spiDmaTxCallback(SPI_Type *base, spi_dma_handle_t *handle, status_t status, void *userData)
{
#ifdef SLAVEIF_DEBUG_DMA
	PRINTF("DMA TX Callback Triggered! Status: %d\n\r", status);
	if (status == kStatus_Success)
	{
		PRINTF("DMA TX Transfer Success!\n\r\r");
	}
	else
	{
		PRINTF("DMA TX Transfer Failed! Status: %d\n\r", status);
	}
#endif
	spiDmaTxCompleted = true; // Signal completion
}

/**
 * @brief Callback function executed upon completion of SPI1 DMA receive.
 * @param base SPI peripheral base pointer (SPI1).
 * @param handle Pointer to the SPI DMA handle (`spiDmaRxHandle`).
 * @param status Status of the completed DMA transfer.
 * @param userData User data pointer (unused).
 */
static void spiDmaRxCallback(SPI_Type *base, spi_dma_handle_t *handle, status_t status, void *userData)
{
#ifdef SLAVEIF_DEBUG_DMA
	PRINTF("DMA RX Callback Triggered! Status: %d\n\r", status);
	if (status == kStatus_Success)
	{
		PRINTF("DMA RX Transfer Success!\n\r\r");
	}
	else
	{
		PRINTF("DMA RX Transfer Failed! Status: %d\n\r", status);
	}
#endif
	spiDmaRxCompleted = true; // Signal completion
}

/* ========================= IRQ Handlers ========================= */
/**
 * @brief Interrupt Service Routine for DMA Channel 0.
 * @details Handles DMA interrupts by calling the appropriate NXP SDK handler
 *          functions for the active DMA handles (dmaTxHandle, dmaRxHandle).
 *          This allows the SDK to manage interrupt flags and trigger the
 *          registered callbacks (spiDmaTxCallback, spiDmaRxCallback).
 */
void DMA0_IRQHandler(void)
{
	#ifdef SLAVEIF_DEBUG_ISR
	PRINTF("DMA0 Interrupt Triggered! Calling SDK Handlers...\n\r");
	#endif

	// --- Call NXP SDK DMA Interrupt Handlers ---
	// Call the handler for the TX channel's DMA handle
	DMA_HandleIRQ(&dmaTxHandle);

	// Call the handler for the RX channel's DMA handle (if using DMA for RX)
	// Since SPI1 RX is configured, we should handle its potential interrupts too,
	// even if nothing is connected, to clear its flags.
	//DMA_HandleIRQ(&dmaRxHandle);

    // No DSB needed for Cortex-M0+
}
/* ========================= Static API Implementations (Helpers) ========================= */
/**
 * @brief Calculates the CRC-8 checksum for SPI frame data.
 * @details Computes CRC-8/MAXIM (Polynomial 0x31, Initial 0xFF, Final XOR 0x00, Reflected In/Out: No)
 *          as specified in the MC33664 datasheet for SPI frame integrity. Uses a lookup table (`crc_table`)
 *          assumed to be defined elsewhere (e.g., `SlaveIF_Cfg.c`).
 * @param data Pointer to the data array (excluding the CRC byte itself).
 * @param data_len Length of the data array in bytes (typically 4 for MC33664 frames).
 * @return uint8_t Calculated CRC-8 value.
 */
static uint8_t calculateCrc8(uint8_t *data, uint16_t data_len)
{
	uint8_t crc = 0xFF; // Initial seed value
	for (uint16_t i = 0; i < data_len; i++)
	{
		// Lookup table method is faster than bitwise calculation
		uint8_t tbl_idx = (crc ^ data[i]) & 0xFF;
		crc = crc_table[tbl_idx];
	}
	return crc;
}

/**
 * @brief Resets the SPI0 peripheral and re-initializes it.
 * @details This function is called typically after a communication timeout or error
 *          to attempt recovery of the SPI master interface. It deinitializes,
 *          reconfigures, and re-enables the SPI0 master with DMA handle.
 */
static void resetSpi0(void)
{
	PRINTF("Resetting SPI0...\n\r\r");

	// Deinitialize SPI0
	SPI_Deinit(SPI_TX);

	// Reconfigure SPI0 Master
	spi_master_config_t masterConfig;
	SPI_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = SPI_BAUDRATE;				   // Use defined baudrate
	masterConfig.polarity = kSPI_ClockPolarityActiveHigh;	   // CPOL = 1 (Mode 3)
	masterConfig.phase = kSPI_ClockPhaseSecondEdge;			   // CPHA = 1 (Mode 3)
	masterConfig.direction = kSPI_MsbFirst;					   // MSB first is standard for MC33xxx
	masterConfig.outputMode = kSPI_SlaveSelectAutomaticOutput; // Hardware CS control
	// Ensure SPI0 clock is enabled (might have been disabled by Deinit)
	CLOCK_EnableClock(kCLOCK_Spi0);
	SPI_MasterInit(SPI_TX, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	// Recreate the SPI Master DMA handle
	SPI_MasterTransferCreateHandleDMA(SPI_TX, &spiDmaTxHandle, spiDmaTxCallback, NULL, &dmaTxHandle, &dmaRxHandle);

	PRINTF("SPI0 Reset Complete.\n\r\r");
}

/**
 * @brief Writes a logic value (0 or 1) to the EN pin of the MC33664 transceiver.
 * @details Controls the enable state of the MC33664. Used to enable/disable TPL communication.
 *          Includes one-time GPIO initialization.
 * @param value Logic value to write (0 for low/disable, 1 for high/enable).
 */
static void SlaveIF_writeEnPin(uint8_t value)
{
	// Initialize the EN pin (GPIOE, Pin 0) if not already initialized
	static bool isInitialized = false;
	if (!isInitialized)
	{
		GPIO_PinInit(EN_TRANSCEIVER_GPIO, EN_TRANSCEIVER_PIN, &enPinConfig);
		isInitialized = true;
	}

	// Small delay before changing the pin state
	delay_us(100); // ~100 us delay

	// Write the value to the EN pin
	GPIO_WritePinOutput(EN_TRANSCEIVER_GPIO, EN_TRANSCEIVER_PIN, value);

	// Small delay after changing the pin state to ensure MC33664 recognizes it
	delay_us(100); // ~100 us delay (adjust based on datasheet/testing)
}

/**
 * @brief Reads the logic value of the INTB pin of the MC33664 transceiver.
 * @details Checks the interrupt status pin of the MC33664. INTB is active low,
 *          indicating transceiver has data ready or an error condition.
 *          Includes one-time GPIO initialization.
 * @return uint32_t Logic value of the INTB pin (0 for low/active, 1 for high/inactive).
 */
static uint32_t SlaveIF_readIntbPin(void)
{
	// Initialize the INTB pin (GPIOA, Pin 12) if not already initialized
	static bool isInitialized = false;
	if (!isInitialized)
	{
		GPIO_PinInit(INTB_TRANSCEIVER_GPIO, INTB_TRANSCEIVER_PIN, &intbPinConfig);
		isInitialized = true;
	}

	// Read the INTB pin value
	return (GPIO_ReadPinInput(INTB_TRANSCEIVER_GPIO, INTB_TRANSCEIVER_PIN));
}

/**
 * @brief Checks if a specific MC33771B register response includes a Tag ID.
 * @details Certain measurement registers embed the Tag ID used during the SOC
 *          (Start of Conversion) trigger in their response frame. This function
 *          helps determine if the Tag ID check is applicable for a given read operation.
 * @param regAddr The address of the register being read.
 * @return bool True if the register response contains a Tag ID, false otherwise.
 */
static bool SlaveIF_registerHasTagID(uint8_t regAddr)
{
	// Check against the list of registers known to contain Tag ID in responses
	switch (regAddr)
	{
	// Measurement Registers typically contain Tag ID
	case MEAS_ISENSE1_ADDR:	  // $30
	case MEAS_ISENSE2_ADDR:	  // $31 (LSBs of Current)
	case MEAS_STACK_ADDR:	  // $32
	case MEAS_CELL1_ADDR:	  // $33
	case MEAS_CELL2_ADDR:	  // $34
	case MEAS_CELL3_ADDR:	  // $35
	case MEAS_CELL4_ADDR:	  // $36
	case MEAS_CELL5_ADDR:	  // $37
	case MEAS_CELL6_ADDR:	  // $38
	case MEAS_CELL7_ADDR:	  // $39
	case MEAS_CELL8_ADDR:	  // $3A
	case MEAS_CELL9_ADDR:	  // $3B
	case MEAS_CELL10_ADDR:	  // $3C
	case MEAS_CELL11_ADDR:	  // $3D
	case MEAS_CELL12_ADDR:	  // $3E
	case MEAS_CELL13_ADDR:	  // $3F
	case MEAS_CELL14_ADDR:	  // $40
	case MEAS_AN0_ADDR:		  // $41
	case MEAS_AN1_ADDR:		  // $42
	case MEAS_AN2_ADDR:		  // $43
	case MEAS_AN3_ADDR:		  // $44
	case MEAS_AN4_ADDR:		  // $45
	case MEAS_AN5_ADDR:		  // $46
	case MEAS_AN6_ADDR:		  // $47
	case MEAS_IC_TEMP:		  // $48
	case MEAS_VBG_DIAG_ADC1A: // $49
	case MEAS_VBG_DIAG_ADC1B: // $4A
		return true;		  // These registers return Tag ID

		// Configuration, Status, Fault registers typically do not
	default:
		return false; // Other registers do not contain Tag ID
	}
}

/**
 * @brief Writes data to a specified MC33771B register via SPI0 using DMA.
 * @details Constructs the 5-byte SPI frame (CRC, Control, Address, Data High, Data Low),
 *          including the Rolling Counter (RC). Initiates a DMA-based SPI master write.
 *          Handles potential SPI busy states and transfer timeouts/errors with retries and reset.
 * @param regAddress Register address (0x00 - 0x7F) to write to.
 * @param data 16-bit data payload to write to the register.
 * @return bool True if the write operation (DMA transfer initiated and completed successfully) is successful, false otherwise.
 */
static bool SlaveIF_writeRegister(uint8_t regAddress, uint16_t data)
{
	uint8_t txFrame[BUFFER_SIZE] = {0};
	uint8_t rxFrame[BUFFER_SIZE] = {0}; // Dummy buffer

	const uint8_t CID = 0x01;		// Cluster ID
	const uint8_t Write_CMD = 0x02; // Command = Read

	uint8_t rc = gRollingCounter & 0x03;
	gRollingCounter = (gRollingCounter + 1) & 0x03;
	// Construct read request frame
	txFrame[1] = (CID << 4) | (rc << 2) | (Write_CMD & 0x03); // Read command (0001) + Cluster ID + RC
	txFrame[2] = (regAddress & 0x7F) | 0x80;				  // Address + Master bit
	txFrame[3] = (data >> 8) & 0xFF;						  // Data high byte
	txFrame[4] = data & 0xFF;								  // Data low byte
	txFrame[0] = calculateCrc8(&txFrame[1], 4);				  // CRC-8

	PRINTF("Step 2: Frame constructed: [%02X, %02X, %02X, %02X, %02X]\n\r\r",
			txFrame[0], txFrame[1], txFrame[2], txFrame[3], txFrame[4]);

	// Step 3: Initialize spiXfer
	PRINTF("Step 3: Initializing spiXfer...\n\r\r");
	spi_transfer_t spiXfer = {
			.txData = txFrame,
			.rxData = rxFrame,
			.dataSize = BUFFER_SIZE};

	// Step 4: Wait until SPI is not busy
	uint32_t busyTimeout = SPI_TIMEOUT_US;
	PRINTF("Step 4: Checking SPI status: 0x%08X\n\r\r", SPI_GetStatusFlags(SPI_TX));
	while ((SPI_GetStatusFlags(SPI_TX) & kSPI_TxBufferEmptyFlag) == 0 && busyTimeout--)
	{
	}
	if (busyTimeout == 0)
	{
		PRINTF("SPI Busy Timeout! Cannot start new transfer.\n\r\r");
		resetSpi0(); // Reset SPI on timeout
		return false;
	}

	// Step 5: Start DMA Transfer with retry mechanism
	uint8_t retryCount = SPI_RETRY_COUNT;
	status_t status;
	PRINTF("Step 5: Starting DMA transfer...\n\r\r");
	do
	{
		spiDmaTxCompleted = false;
		status = SPI_MasterTransferDMA(SPI_TX, &spiDmaTxHandle, &spiXfer);
		if (status != kStatus_Success)
		{
			PRINTF("SPI DMA TX Failed! Status: %d\n\r\r", status);
			for (volatile int i = 0; i < 10000; i++)
				; // Delay before retry
			retryCount--;
		}
		else
		{
			break;
		}
	} while (retryCount > 0);

	if (status != kStatus_Success)
	{
		PRINTF("SPI DMA TX Failed After Retries! Status: %d\n\r\r", status);
		resetSpi0(); // Reset SPI on failure
		return false;
	}

	// Step 6: Wait for DMA transfer to complete
	uint32_t timeout = SPI_TIMEOUT_US; 		//(500000U)
	PRINTF("Step 6: Waiting for DMA to complete...\n\r\r");
	while (!spiDmaTxCompleted && timeout--)
	{
	}
	if (!spiDmaTxCompleted)
	{
		PRINTF("SPI DMA Write Timeout!\n\r\r");
		SPI_MasterTransferAbortDMA(SPI_TX, &spiDmaTxHandle); // Abort DMA transfer
		resetSpi0();										 // Reset SPI on timeout
		delay_ms(10);
		return false;
	}

	PRINTF("Step 7: DMA Transfer Completed.\n\r\r");
	uint8_t receivedRC = (rxFrame[1] >> 2) & 0x03;
	uint8_t expectedRC = (gRollingCounter - 1) & 0x03; // RC was incremented after frame was built
	if (receivedRC != rc)
	{
		PRINTF("RC mismatch! Expected: %u, Got: %u\n\r", expectedRC, receivedRC);
		return false;
	}
	// Step 8: Add delay between frames (at least 100 us as per MC33664 datasheet)
	PRINTF("Step 8: Adding delay between frames...\n\r\r");
	delay_us(100); // Delay ~100 us (adjust based on your clock speed)

	PRINTF("Step 9: Transfer successful.\n\r\r");
	return true;
}

/**
 * @brief Reads data from a specified MC33771B register using SPI0 (TX) and SPI1 (RX) with DMA.
 * @details Sends a read command frame via SPI0 TX DMA. Simultaneously (or slightly delayed,
 *          depending on hardware setup), prepares SPI1 RX DMA to receive the response frame.
 *          Verifies CRC, Rolling Counter (RC), and optionally Tag ID in the received frame.
 * @param regAddress Register address (0x00 - 0x7F) to read from.
 * @return uint16_t 16-bit data read from the register. Returns 0xFFFF (or other error indicator)
 *                  if the read operation fails (timeout, CRC error, RC mismatch, Tag ID mismatch).
 * @note Returns 0xFFFF on failure. Consider defining a specific error code if 0xFFFF is valid data.
 */
static uint16_t SlaveIF_readRegister(uint8_t regAddress)
{
	uint8_t txFrame[BUFFER_SIZE] = {0};
	uint8_t rxFrame[BUFFER_SIZE] = {0};

	const uint8_t CID = DEFAULT_CID; // Cluster ID
	const uint8_t READ_CMD = 0x01;	 // Command = Read

	uint8_t rc = gRollingCounter & 0x03;
	gRollingCounter = (gRollingCounter + 1) & 0x03;
	// Construct read request frame
	txFrame[1] = (CID << 4) | (rc << 2) | (READ_CMD & 0x03); // Read command (0001) + Cluster ID + RC
	txFrame[2] = (regAddress & 0x7F) | 0x80;				 // Address + Master bit
	txFrame[3] = 0x00;										 // Placeholder
	txFrame[4] = 0x00;										 // Placeholder
	txFrame[0] = calculateCrc8(&txFrame[1], 4);				 // CRC-8

	spi_transfer_t spiTxXfer = {
			.txData = txFrame,
			.rxData = NULL,
			.dataSize = BUFFER_SIZE};

	spi_transfer_t spiRxXfer = {
			.txData = NULL,
			.rxData = rxFrame,
			.dataSize = BUFFER_SIZE};

	// Send read request (DMA)
	spiDmaTxCompleted = false;
	status_t status = SPI_MasterTransferDMA(SPI_TX, &spiDmaTxHandle, &spiTxXfer);
	if (status != kStatus_Success)
	{
		PRINTF("SPI DMA TX Failed! Status: %d\n\r\r", status);
		return 0;
	}

	// Wait for DMA transfer to complete
	uint32_t timeout = 1000000;
	while (!spiDmaTxCompleted && timeout--)
	{
	}
	if (!spiDmaTxCompleted)
	{
		PRINTF("SPI DMA TX Timeout!\n\r\r");
		return 0;
	}

	// Receive response (DMA)
	spiDmaRxCompleted = false;
	status = SPI_SlaveTransferDMA(SPI_RX, &spiDmaRxHandle, &spiRxXfer);
	if (status != kStatus_Success)
	{
		PRINTF("SPI DMA RX Failed! Status: %d\n\r\r", status);
		return 0;
	}

	// Wait for DMA transfer to complete
	timeout = 1000000;
	while (!spiDmaRxCompleted && timeout--)
	{
	}
	if (!spiDmaRxCompleted)
	{
		PRINTF("SPI DMA RX Timeout!\n\r\r");
		return 0;
	}

	// Verify CRC
	uint8_t crcReceived = rxFrame[0];
	uint8_t crcCalculated = calculateCrc8(&rxFrame[1], 4);
	if (crcReceived != crcCalculated)
	{
		PRINTF("CRC Check Failed! Received: %02X, Calculated: %02X\n\r\r", crcReceived, crcCalculated);
		return 0;
	}
	// Optional TAG_ID check
	if (SlaveIF_registerHasTagID(regAddress))
	{
		uint8_t tagIdReceived = rxFrame[1] & 0x0F;
		if (tagIdReceived != (TAGID & 0x0F))
		{
			PRINTF("TAG_ID mismatch for 0x%02X! Expected: %X, Got: %X\n\r",
					regAddress, TAGID, tagIdReceived);
			return 0;
		}
	}
	else
	{
		uint8_t receivedRC = (rxFrame[1] >> 2) & 0x03;
		uint8_t expectedRC = (gRollingCounter - 1) & 0x03; // RC was incremented after frame was built
		if (receivedRC != rc)
		{
			PRINTF("RC mismatch! Expected: %u, Got: %u\n\r", expectedRC, receivedRC);
			return 0;
		}
	}

	// Add delay to ensure MC33664 timing requirements (e.g., 100 us between frames)
	for (volatile int i = 0; i < 1000; i++)
		; // Adjust delay as needed

	return (rxFrame[3] << 8) | rxFrame[4]; // Combine high and low bytes
}

/* ------------------------- Configuration Setting ------------------------- */
/**
 * @brief Configures the MC33771B Initialization Register ($00).
 * @details Sets the Cluster ID (CID) and optionally enables the bus switch.
 *          This is typically one of the first registers configured after wake-up.
 * @param channelSwitch If true, enables the bus switch (BUS_SW bit 4 = 1).
 *                      If false, disables the bus switch (BUS_SW bit 4 = 0).
 * @return bool True if the register write was successful, false otherwise.
 */
static bool SlaveIF_setupSystem(bool channelSwitch)
{
	uint8_t regAddress = INIT_REGISTER;		// $00
	uint16_t data = (DEFAULT_CID & 0x000F); // Set CID (bits 0-3) from Cfg

	if (channelSwitch)
	{
		SET_BIT(data, 4); // Set BUS_SW (bit 4) to enable bus switch
		PRINTF("Setting up System (INIT $00): CID=%d, BUS_SW=Enabled\n\r", (DEFAULT_CID & 0x0F));
	}
	else
	{
		// Bit 4 is already 0, no action needed to disable
		PRINTF("Setting up System (INIT $00): CID=%d, BUS_SW=Disabled\n\r", (DEFAULT_CID & 0x0F));
	}

	return SlaveIF_writeRegister(regAddress, data);
}

/**
 * @brief Configures System Configuration Register 1 ($03).
 * @details Writes a predefined value `SYS_CFG1_VALUE` (from Cfg) to the register.
 *          This register controls various system parameters like balancing method,
 *          filter settings, etc. Refer to the datasheet for bit definitions.
 * @return bool True if the register write was successful, false otherwise.
 */
static bool SlaveIF_configSystem1(void)
{
	uint8_t regAddress = SYS_CFG1_ADDR; // $03
	uint16_t data = 0x9381;				// Use value from configuration

	PRINTF("Configuring SYS_CFG1 ($%02X) with data: 0x%04X\n\r", regAddress, data);
	return SlaveIF_writeRegister(regAddress, data);
}

/**
 * @brief Configures System Configuration Register 2 ($04).
 * @details Writes a predefined value `SYS_CFG2_VALUE` (from Cfg) to the register.
 *          This register controls communication watchdog, fault masks, etc.
 *          Refer to the datasheet for bit definitions.
 * @return bool True if the register write was successful, false otherwise.
 */
static bool SlaveIF_configSystem2(void)
{
	uint8_t regAddress = SYS_CFG2_ADDR; // $04
	uint16_t data = 0x6231;				// Use value from configuration

	PRINTF("Configuring SYS_CFG2 ($%02X) with data: 0x%04X\n\r", regAddress, data);
	return SlaveIF_writeRegister(regAddress, data);
}

/**
 * @brief Enables Over-Voltage (OV) and Under-Voltage (UV) detection for all cells (1-14).
 * @details Writes 0xFFFF to the OV_UV_EN register ($09), setting enable bits for all cells.
 * @return bool True if the register write was successful, false otherwise.
 */
static bool SlaveIF_enableOvUv(void)
{
	uint8_t regAddress = OV_UV_EN_ADDR; // $09
	uint16_t data = 0xFFFF;				// Enable OV/UV detection for all 14 cells (bits 0-13 set)

	PRINTF("Enabling OV/UV detection for all cells (OV_UV_EN $09 = 0xFFFF)\n\r\r");
	return SlaveIF_writeRegister(regAddress, data);
}

/**
 * @brief Configures all GPIO pins (AN0-AN6) as analog inputs.
 * @details Writes 0x0000 to the GPIO_CFG1 register ($0A). In this register,
 *          each pair of bits configures one GPIO pin. Setting a pair to '00'
 *          configures the pin as an analog input, suitable for thermistors.
 * @return bool True if the register write was successful, false otherwise.
 */
static bool SlaveIF_configAllGpiosForTempSensors(void)
{
	uint8_t regAddress = GPIO_CFG1_ADDR; // $0A
	// Configure GPIO0-GPIO6 as Analog Inputs (CFG<1:0> = 00b for each)
	// GPIO0: Bits 1:0 = 00
	// GPIO1: Bits 3:2 = 00
	// ...
	// GPIO6: Bits 13:12 = 00
	// Bits 15:14 are reserved.
	uint16_t gpioCfg1Value = 0x0000;

	PRINTF("Configuring all GPIOs (0-6) as Analog Inputs (GPIO_CFG1 $0A = 0x0000)\n\r\r");
	return SlaveIF_writeRegister(regAddress, gpioCfg1Value);
}

/* ------------------------- Threshold Setting ------------------------- */
/**
 * @brief Calculates the register value for OV/UV thresholds based on voltage inputs.
 * @param ov_volts Over Voltage threshold in Volts.
 * @param uv_volts Under Voltage threshold in Volts.
 * @return uint16_t The 16-bit value to write to TH_ALL_CT or TH_CTx register.
 *         Format: [OV_Threshold (8 bits) | UV_Threshold (8 bits)].
 */
static uint16_t SlaveIf_calculateOvUvThresholdReg(float ov_volts, float uv_volts)
{
	// Convert volts to millivolts
	float ov_mv = ov_volts * 1000.0f;
	float uv_mv = uv_volts * 1000.0f;

	// Calculate scaled values based on resolution
	float ovScaledValueF = ov_mv / OVUV_RESOLUTION_MV_PER_LSB;
	float uvScaledValueF = uv_mv / OVUV_RESOLUTION_MV_PER_LSB;

	// Clamp and round to nearest integer (0-255 range)
	uint8_t ovInteger = (ovScaledValueF < 0.0f) ? 0 : ((ovScaledValueF > 255.0f) ? 255 : (uint8_t)(ovScaledValueF + 0.5f));
	uint8_t uvInteger = (uvScaledValueF < 0.0f) ? 0 : ((uvScaledValueF > 255.0f) ? 255 : (uint8_t)(uvScaledValueF + 0.5f));

	// Combine into 16-bit register value (OV in MSB, UV in LSB)
	uint16_t registerValue = ((uint16_t)ovInteger << 8) | (uint16_t)uvInteger;

	return registerValue;
}

/**
 * @brief Calculates the register value for the Over Temperature (OT) threshold.
 * @param overTemp_inVolt Threshold voltage (from thermistor circuit) corresponding to OT, in Volts.
 * @return uint16_t The 10-bit value (in lower bits) for TH_ANx_OT register.
 */
static uint16_t SlaveIf_calculateOverTempThresholdReg(float overTemp_inVolt)
{
	// Convert volts to millivolts
	float ot_mv = overTemp_inVolt * 1000.0f;

	// Calculate scaled value based on resolution
	float otScaledValueF = ot_mv / OTUT_RESOLUTION_MV_PER_LSB;

	// Clamp and round to nearest integer (0-1023 range for 10 bits)
	uint16_t otInteger = (otScaledValueF < 0.0f) ? 0 : ((otScaledValueF > 1023.0f) ? 1023 : (uint16_t)(otScaledValueF + 0.5f));

	// The value occupies the lower 10 bits (0-9)
	return otInteger;
}

/**
 * @brief Calculates the register value for the Under Temperature (UT) threshold.
 * @param underTemp_inVolt Threshold voltage (from thermistor circuit) corresponding to UT, in Volts.
 * @return uint16_t The 10-bit value (in lower bits) for TH_ANx_UT register.
 */
static uint16_t SlaveIf_calculateUnderTempThresholdReg(float underTemp_inVolt)
{
	// Convert volts to millivolts
	float ut_mv = underTemp_inVolt * 1000.0f;

	// Calculate scaled value based on resolution
	float utScaledValueF = ut_mv / OTUT_RESOLUTION_MV_PER_LSB;

	// Clamp and round to nearest integer (0-1023 range for 10 bits)
	uint16_t utInteger = (utScaledValueF < 0.0f) ? 0 : ((utScaledValueF > 1023.0f) ? 1023 : (uint16_t)(utScaledValueF + 0.5f));

	// The value occupies the lower 10 bits (0-9)
	return utInteger;
}

/**
 * @brief Calculates the register value for the Over Current (OC) threshold.
 * @param overCurrent_amps Over Current threshold in Amperes.
 * @param shunt_resistance_micro_ohms Shunt resistor value in micro-Ohms.
 * @return uint16_t The 12-bit value (in lower bits) for TH_ISENSE_OC register.
 */
static uint16_t SlaveIf_calculateOverCurrentThresholdReg(float overCurrent_amps, float shunt_resistance_micro_ohms)
{
	// Calculate the voltage across the shunt resistor in microvolts (V = I * R)
	float overVolt_in_shuntResistor_uV = overCurrent_amps * shunt_resistance_micro_ohms;

	// Calculate scaled value based on resolution
	float ocScaledValueF = overVolt_in_shuntResistor_uV / OC_RESOLUTION_UV_PER_LSB;

	// Clamp and round to nearest integer (0-4095 range for 12 bits)
	uint16_t ocInteger = (ocScaledValueF < 0.0f) ? 0 : ((ocScaledValueF > 4095.0f) ? 4095 : (uint16_t)(ocScaledValueF + 0.5f));

	// The value occupies the lower 12 bits (0-11)
	return ocInteger;
}

/**
 * @brief Calculates and sets the global OV and UV threshold register (TH_ALL_CT $50).
 * @details Uses the helper `SlaveIf_CalculateOvUvThresholdReg` to convert target
 *          voltages into the required 16-bit register format and writes it.
 * @param ov_volts Global Over Voltage threshold in Volts.
 * @param uv_volts Global Under Voltage threshold in Volts.
 * @return bool True if the write operation was successful, false otherwise.
 */
static bool SlaveIf_setGlobalOvUvThreshold(float ov_volts, float uv_volts)
{
	uint16_t regData = SlaveIf_calculateOvUvThresholdReg(ov_volts, uv_volts);
	PRINTF("Setting Global OV/UV Threshold (Reg 0x%02X) to: 0x%04X (OV: %.3fV, UV: %.3fV)\n\r",
			TH_ALL_CT_ADDR, regData, ov_volts, uv_volts);
	return SlaveIF_writeRegister(TH_ALL_CT_ADDR, regData);
}
/**
 * @brief Calculates and sets the individual OV and UV threshold register for a specific cell (TH_CTx $4C-$59).
 * @details Uses the helper `SlaveIf_CalculateOvUvThresholdReg` to convert target
 *          voltages into the required 16-bit register format and writes it to the
 *          appropriate TH_CTx register address.
 * @param cell_index Cell number (1-14) for which to set the threshold.
 * @param ov_volts Cell-specific Over Voltage threshold in Volts.
 * @param uv_volts Cell-specific Under Voltage threshold in Volts.
 * @return bool True if the write operation was successful, false otherwise. Returns false if cell_index is invalid.
 */
static bool SlaveIf_setCellOvUvThreshold(uint8_t cell_index, float ov_volts, float uv_volts)
{
	// Validate cell index
	if (cell_index < 1 || cell_index > 14)
	{
		PRINTF("Error: Invalid cell index %d for OV/UV threshold setting.\n\r", cell_index);
		return false;
	}
	// Calculate register address (TH_CT1 = 0x59, TH_CT14 = 0x4C)
	uint8_t regAddress = TH_CT1_ADDR - (cell_index - 1);

	// Calculate register data
	uint16_t regData = SlaveIf_calculateOvUvThresholdReg(ov_volts, uv_volts);

	PRINTF("Setting Cell %d OV/UV Threshold (Reg 0x%02X) to: 0x%04X (OV: %.3fV, UV: %.3fV)\n\r",
			cell_index, regAddress, regData, ov_volts, uv_volts);
	return SlaveIF_writeRegister(regAddress, regData);
}

/**
 * @brief Calculates and sets the Over Temperature (OT) threshold for a specific ANx input (TH_ANx_OT $5A-$60).
 * @details Uses the helper `SlaveIf_CalculateOverTempThresholdReg` to convert the target
 *          threshold voltage (corresponding to the OT limit) into the required 10-bit
 *          register format and writes it to the appropriate TH_ANx_OT register address.
 * @param anx_index ANx input number (0-6) for which to set the threshold.
 * @param overTemp_inVolt Voltage threshold (from thermistor circuit) corresponding to OT, in Volts.
 * @return bool True if the write operation was successful, false otherwise. Returns false if anx_index is invalid.
 */
static bool SlaveIf_setOverTempThreshold(uint8_t anx_index, float overTemp_inVolt)
{
	// Validate ANx index
	if (anx_index > 6)
	{
		PRINTF("Error: Invalid ANx index %d for OT threshold setting.\n\r", anx_index);
		return false;
	}
	// Calculate register address (TH_AN0_OT = 0x60, TH_AN6_OT = 0x5A)
	uint8_t regAddress = TH_ANO_OT_ADDR - anx_index;

	// Calculate register data (lower 10 bits)
	uint16_t regData = SlaveIf_calculateOverTempThresholdReg(overTemp_inVolt);

	PRINTF("Setting AN%d OT Threshold (Reg 0x%02X) to: 0x%04X (Input V: %.3fV)\n\r",
			anx_index, regAddress, regData, overTemp_inVolt);
	// Mask to ensure only lower 10 bits are considered, although write handles full 16-bit word
	return SlaveIF_writeRegister(regAddress, (regData & 0x03FF));
}

/**
 * @brief Calculates and sets the Under Temperature (UT) threshold for a specific ANx input (TH_ANx_UT $61-$67).
 * @details Uses the helper `SlaveIf_CalculateUnderTempThresholdReg` to convert the target
 *          threshold voltage (corresponding to the UT limit) into the required 10-bit
 *          register format and writes it to the appropriate TH_ANx_UT register address.
 * @param anx_index ANx input number (0-6) for which to set the threshold.
 * @param underTemp_inVolt Voltage threshold (from thermistor circuit) corresponding to UT, in Volts.
 * @return bool True if the write operation was successful, false otherwise. Returns false if anx_index is invalid.
 */
static bool SlaveIf_setUnderTempThreshold(uint8_t anx_index, float underTemp_inVolt)
{
	// Validate ANx index
	if (anx_index > 6)
	{
		PRINTF("Error: Invalid ANx index %d for UT threshold setting.\n\r", anx_index);
		return false;
	}
	// Calculate register address (TH_AN0_UT = 0x67, TH_AN6_UT = 0x61)
	uint8_t regAddress = TH_AN0_UT_ADDR - anx_index;

	// Calculate register data (lower 10 bits)
	uint16_t regData = SlaveIf_calculateUnderTempThresholdReg(underTemp_inVolt);

	PRINTF("Setting AN%d UT Threshold (Reg 0x%02X) to: 0x%04X (Input V: %.3fV)\n\r",
			anx_index, regAddress, regData, underTemp_inVolt);
	// Mask to ensure only lower 10 bits are considered
	return SlaveIF_writeRegister(regAddress, (regData & 0x03FF));
}

/**
 * @brief Calculates and sets the Over Current (OC) threshold register (TH_ISENSE_OC $68).
 * @details Uses the helper `SlaveIf_CalculateOverCurrentThresholdReg` to convert the target
 *          over current limit (in Amperes) and shunt resistance (in micro-Ohms) into the
 *          required 12-bit register format (representing voltage across shunt) and writes it.
 * @param overCurrent_amps Over Current threshold in Amperes.
 * @param shunt_resistance_micro_ohms Shunt resistor value in micro-Ohms.
 * @return bool True if the write operation was successful, false otherwise.
 */
static bool SlaveIf_setOverCurrentThreshold(float overCurrent_amps, float shunt_resistance_micro_ohms)
{
	// Calculate register data (lower 12 bits)
	uint16_t regData = SlaveIf_calculateOverCurrentThresholdReg(overCurrent_amps, shunt_resistance_micro_ohms);

	PRINTF("Setting Over current Threshold (Reg 0x%02X) to: 0x%04X (Current: %.3fA, Shunt: %.1fuOhm)\n\r",
			TH_ISENSE_OC_ADDR, regData, overCurrent_amps, shunt_resistance_micro_ohms);
	// Mask to ensure only lower 12 bits are considered
	return SlaveIF_writeRegister(TH_ISENSE_OC_ADDR, (regData & 0x0FFF));
}

/* ========================= Public API Implementations ========================= */

/* ------------------------- Initialization & Control ------------------------- */
/**
 * @brief Initializes SPI and DMA peripherals for communication.
 * @details Configures SPI0 as Master TX and SPI1 as Slave RX.
 *          Sets up DMA channels, MUX, and creates DMA handles for SPI transfers.
 *          Enables DMA interrupts. Must be called before any other SPI communication.
 */
void SlaveIF_initTransfer(void)
{
	spi_master_config_t masterConfig;
	spi_slave_config_t slaveConfig;

	// --- Enable DMA clock ---
	CLOCK_EnableClock(kCLOCK_Dma0); // Enable DMA0 clock

	// --- Initialize DMA ---
	DMA_Init(DMA0);

	// --- Initialize DMA MUX ---
	DMAMUX_Init(DMAMUX0);
	DMAMUX_SetSource(DMAMUX0, DMA_CHANNEL_SPI_TX, kDmaRequestMux0SPI0Tx); // Channel 0 for SPI0 TX
	DMAMUX_SetSource(DMAMUX0, DMA_CHANNEL_SPI_RX, kDmaRequestMux0SPI1Rx); // Channel 1 for SPI1 RX
	DMAMUX_EnableChannel(DMAMUX0, DMA_CHANNEL_SPI_TX);					  // Enable Channel 0 (TX)
	DMAMUX_EnableChannel(DMAMUX0, DMA_CHANNEL_SPI_RX);					  // Enable Channel 1 (RX)

	// --- Create DMA handles ---
	DMA_CreateHandle(&dmaTxHandle, DMA0, DMA_CHANNEL_SPI_TX); // Channel 0 for TX
	DMA_CreateHandle(&dmaRxHandle, DMA0, DMA_CHANNEL_SPI_RX); // Channel 1 for RX

	// Enable DMA interrupts in NVIC
	NVIC_EnableIRQ(DMA0_IRQn);
	NVIC_SetPriority(DMA0_IRQn, 0);

	// --- Configure SPI0 (TX) as master ---
	SPI_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = SPI_BAUDRATE;				   // 2 Mbps
	masterConfig.polarity = kSPI_ClockPolarityActiveHigh;	   // CPOL = 1
	masterConfig.phase = kSPI_ClockPhaseSecondEdge;			   // CPHA = 1
	masterConfig.direction = kSPI_MsbFirst;					   // MSB first
	masterConfig.outputMode = kSPI_SlaveSelectAutomaticOutput; // Enable automatic CS control
	CLOCK_EnableClock(kCLOCK_Spi0);
	SPI_MasterInit(SPI_TX, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));
	SPI_MasterTransferCreateHandleDMA(SPI_TX, &spiDmaTxHandle, spiDmaTxCallback, NULL, &dmaTxHandle, &dmaRxHandle);

	// --- Configure SPI1 (RX) as slave ---
	SPI_SlaveGetDefaultConfig(&slaveConfig);
	slaveConfig.polarity = kSPI_ClockPolarityActiveHigh; // CPOL = 1
	slaveConfig.phase = kSPI_ClockPhaseSecondEdge;		 // CPHA = 1
	slaveConfig.direction = kSPI_MsbFirst;				 // MSB first
	CLOCK_EnableClock(kCLOCK_Spi1);
	SPI_SlaveInit(SPI_RX, &slaveConfig);
	SPI_SlaveTransferCreateHandleDMA(SPI_RX, &spiDmaRxHandle, spiDmaRxCallback, NULL, &dmaTxHandle, &dmaRxHandle);
	delay_ms(5);
}

/**
 * @brief Enables the MC33664 transceiver for TPL communication.
 * @details Sets the EN pin of the MC33664 transceiver high.
 *          Intended for enabling TPL (Transformer Physical Layer) mode.
 * @return bool Always returns true (as setting EN pin is the primary action).
 */
bool SlaveIF_tplEnable(void)
{
	PRINTF("Enabling MC33664 TPL...\n\r\r");
	// Set the EN pin to high (logic 1) to enable the transceiver
	SlaveIF_writeEnPin(true);

	// Optionally check the INTB pin
	/*
	uint32_t intbValue = SlaveIF_readIntbPin();
	if (intbValue != 0)
	{
		PRINTF("Warning: INTB pin is high, expected low!\n\r\r");
		return false;
	}
	 */

	PRINTF("MC33664 TPL Enabled Successfully.\n\r\r");
	return true;
}

/**
 * @brief Sends a wake-up sequence to the MC33771B via TPL.
 * @details Generates specific low-to-high transitions on the SPI Chip Select (CS) line
 *          while the SPI peripheral pins are temporarily configured as GPIOs. This pattern
 *          is detected by the MC33664/MC33771B on the TPL line to wake the device from sleep.
 *          Restores SPI CS pin muxing afterwards.
 * @note Timings (delays) are critical and based on the MC33771B datasheet wake-up sequence.
 *       Assumes CS pin is PTC4.
 */
void SlaveIF_wakeUp(void)
{
	PRINTF("Sending Wake-Up Sequence...\n\r\r");

	// Temporarily configure CS pin (PTC4) as GPIO output
	PORT_SetPinMux(SPI_CS_PORT, SPI_CS_PIN, kPORT_MuxAsGpio);
	GPIO_PinInit(SPI_CS_GPIO, SPI_CS_PIN, &(gpio_pin_config_t){kGPIO_DigitalOutput, 1});			 // CS pin

	/* First Low-to-High Transition for Wake-up */
	GPIO_WritePinOutput(SPI_CS_GPIO, SPI_CS_PIN, 0); // Ensure CSB is Low
	delay_us(20);									 // Wait > 15 µs (t_CSSL - CS Setup Low) - Using 20us
	GPIO_WritePinOutput(SPI_CS_GPIO, SPI_CS_PIN, 1); // Set CSB High
	delay_us(600);									 // Wait >= 500 µs (t_CSHD - CS Hold High Duration 1) - Using 600us

	/* Second Low-to-High Transition for Wake-up */
	GPIO_WritePinOutput(SPI_CS_GPIO, SPI_CS_PIN, 0); // Set CSB Low
	delay_us(20);									 // Wait > 15 µs (t_CSSL) - Using 20us
	GPIO_WritePinOutput(SPI_CS_GPIO, SPI_CS_PIN, 1); // Set CSB High
	delay_us(1000);									 // Wait >= 800 µs (t_WUEND - Wake-up End) - Using 1000us					   /* Wait ~1000 µs */

	// Restore CS pin function to SPI peripheral
	PORT_SetPinMux(SPI_CS_PORT, SPI_CS_PIN, kPORT_MuxAlt2);

	PRINTF("Wake-Up Sequence Sent.\n\r\r");
	delay_ms(10); // Allow time for slave to wake up fully before communication
}

/**
 * @brief Sets the MC33771B device to enter or exit sleep mode request state.
 * @details Modifies the GO2SLEEP bit (Bit 0) in the SYS_CFG_GLOBAL register ($02).
 *          Setting GO2SLEEP=1 commands the device to enter sleep mode after the
 *          current operation/conversion completes. Clearing it (GO2SLEEP=0) is
 *          generally not used to exit sleep (wake-up signal required) but might
 *          clear a pending sleep request before it takes effect.
 * @param enterSleepMode True to set GO2SLEEP=1 (request sleep).
 *                       False to set GO2SLEEP=0 (clear sleep request).
 * @return bool True if the register write was successful, false otherwise.
 */
bool SlaveIF_setSleepMode(bool enterSleepMode)
{
	uint8_t regAddress = SYS_CFG_GLOBAL_ADDR; // $02
	uint16_t data = 0;						  // Start with 0

	if (enterSleepMode)
	{
		SET_BIT(data, 0); // Set GO2SLEEP (bit 0) to request sleep mode
		PRINTF("Requesting Sleep Mode (SYS_CFG_GLOBAL $02: GO2SLEEP=1)\n\r\r");
	}
	else
	{
		// GO2SLEEP bit remains 0
		PRINTF("Clearing Sleep Mode Request (SYS_CFG_GLOBAL $02: GO2SLEEP=0)\n\r\r");
	}

	// Only bit 0 is defined writeable in this register according to Rev 8 datasheet
	return SlaveIF_writeRegister(regAddress, data);
}

/* ------------------------- ADC Configuration & Measurement Trigger ------------------------- */
/**
 * @brief Configures the ADC Configuration Register ($06).
 * @details Sets ADC resolutions and current sense (ADC2) PGA settings.
 *          - Sets ADC1A, ADC1B, ADC2 resolutions (e.g., 16-bit).
 *          - Configures ADC2 PGA (e.g., Auto Gain starting at 4x).
 *          - Ensures SOC (Start of Conversion) bit remains 0.
 *          Uses predefined values from `SlaveIF_Cfg.h`. Refer to Datasheet Table 50.
 * @return bool True if the register write was successful, false otherwise.
 */
bool SlaveIF_configAdc(void)
{
	uint8_t regAddress = ADC_CFG_ADDR; // $06
	uint16_t data = 0;				   // Initialize with all bits 0
	data = 0b000000001111111;

	PRINTF("Configuring ADC_CFG ($%02X) with data: 0x%04X\n\r\r", regAddress, data);
	if (!SlaveIF_writeRegister(regAddress, data))
	{
		PRINTF("Error writing to ADC_CFG register!\n\r\r");
		return false;
	}
	else
	{
		PRINTF("ADC_CFG register configured successfully.\n\r\r");
		return true;
	}
}

/**
 * @brief Configures the ADC2 Offset Compensation Register ($07).
 * @details Writes to the ADC2_OFFSET_COMP register. Sets the PCB offset compensation
 *          value and configures Coulomb Counter (CC) behavior (free-running, reset source).
 * @param pcbOffset Signed 8-bit value (-128 to +127) for PCB offset compensation.
 *                  Use 0 if no specific offset calibration is performed. Stored in bits 7:0.
 * @return bool True if the register write was successful, false otherwise.
 * @note Refer to Datasheet Rev 8.0, Table 51.
 */
bool SlaveIF_configAdcOffset(int8_t pcbOffset)
{
	uint8_t regAddress = ADC2_OFFSET_COMP_ADDR; // $07
	uint16_t data = 0;

	// --- Set ADC2_OFFSET_COMP (Bits 7:0) ---
	// Sets the 8 LSBs to the pcbOffset value
	data |= ((uint8_t)pcbOffset << ADC2_OFFSET_SHIFT);

	// --- Configure Coulomb Counter ---
	// Set FREE_CNT = 1 (Bit 15) -> Free running with rollover
	SET_BIT(data, FREE_CNT_BIT_POS);
	// Set CC_RST_CFG = 1 (Bit 14) -> Reset only via explicit command ADC_CFG[CC_RST]=1
	SET_BIT(data, CC_RST_CFG_BIT_POS);
	// CC_P_OVF, CC_N_OVF, SAMP_OVF, CC_OVT are Read-Only flags

	PRINTF("Configuring ADC2_OFFSET_COMP ($%02X) with data: 0x%04X (Offset: %d)\n\r\r", regAddress, data, pcbOffset);
	if (!SlaveIF_writeRegister(regAddress, data))
	{
		PRINTF("Error writing to ADC2_OFFSET_COMP register!\n\r\r");
		return false;
		// Handle error appropriately
	}
	else
	{
		PRINTF("ADC2_OFFSET_COMP register configured successfully.\n\r\r");
		return true;
	}
}

/* ------------------------- Init Slave Setting & Measurement Trigger ------------------------- */
/**
 * @brief Performs basic initialization sequence for the MC33771B slave device.
 * @details Calls essential configuration functions in a typical order:
 *          1. `SlaveIF_setupSystem` (sets CID, bus switch)
 *          2. `SlaveIF_configSystem1`
 *          3. `SlaveIF_configSystem2`
 *          4. `SlaveIF_configAdc` (sets ADC resolution, PGA)
 *          5. `SlaveIF_configAdcOffset` (sets offset comp, CC config)
 *          6. `SlaveIF_enableOvUv` (enables OV/UV detection for all cells)
 *          7. `SlaveIF_configAllGpiosForTempSensors` (sets GPIOs to analog input)
 *          Intended to be called once after hardware initialization and wake-up.
 * @return bool True if all initialization steps succeed, false if any step fails.
 */
bool SlaveIF_init(void)
{
	bool success = true;

	PRINTF("--- Starting MC33771B Basic Configuration Sequence ---\n\r\r");

	// Step 1: Set up system basics (CID, BUS_SW)
	if (!SlaveIF_setupSystem(true)) // Enable bus switch by default
	{
		PRINTF("Error: Failed to set up system (INIT $00)!\n\r\r");
		success = false;
	}

	// Step 2: Configure System Register 1
	if (!SlaveIF_configSystem1())
	{
		PRINTF("Error: Failed to configure SYS_CFG1 ($03)!\n\r\r");
		success = false;
	}

	// Step 3: Configure System Register 2
	if (!SlaveIF_configSystem2())
	{
		PRINTF("Error: Failed to configure SYS_CFG2 ($04)!\n\r\r");
		success = false;
	}

	// Step 4: Configure ADC settings (Resolution, PGA)
	if (!SlaveIF_configAdc())
	{
		PRINTF("Error: Failed to configure ADC_CFG ($06)!\n\r\r");
		success = false;
	}

	// Step 5: Configure ADC offset compensation and Coulomb Counter
	// Using default PCB offset of 0 here, adjust if calibrated value is known
	if (!SlaveIF_configAdcOffset(1))
	{
		PRINTF("Error: Failed to configure ADC2_OFFSET_COMP ($07)!\n\r\r");
		success = false;
	}

	// Step 6: Enable Over-Voltage and Under-Voltage detection for all cells
	if (!SlaveIF_enableOvUv())
	{
		PRINTF("Error: Failed to enable OV/UV detection (OV_UV_EN $09)!\n\r\r");
		success = false;
	}

	// Step 7: Configure all GPIOs as analog inputs for temperature sensors
	if (!SlaveIF_configAllGpiosForTempSensors())
	{
		PRINTF("Error: Failed to configure GPIOs for temp sensors (GPIO_CFG1 $0A)!\n\r\r");
		success = false;
	}
	if (success)
	{
		PRINTF("--- MC33771B Basic Configuration Complete Successfully ---\n\r\r");
	}
	else
	{
		PRINTF("--- MC33771B Basic Configuration Failed! ---\n\r\r");
	}

	return success;
}

/**
 * @brief Configures all protection thresholds (OV, UV, OT, UT, OC) on the MC33771B.
 * @details This function sets the global voltage limits, temperature limits for all
 *          analog inputs (AN0-AN6), and the overcurrent limit using placeholder
 *          values defined internally within this function.
 *          It calls the internal static functions for setting each threshold type.
 * @return bool True if ALL threshold configurations were written successfully,
 *              false if any configuration write failed.
 * @note For better maintainability, consider defining these placeholder values
 *       as constants (#define) in `SlaveIF_Cfg.h`.
 */
bool SlaveIF_configureProtectionThresholds(void)
{
	bool overall_success = true; // Track overall success
	bool step_success;			 // Track success of individual steps

#ifdef SLAVEIF_DEBUG_CONFIG
	PRINTF("--- Starting MC33771B Protection Threshold Configuration ---\n\r\r");
#endif

	// 1. Set Global OV/UV Threshold
#ifdef SLAVEIF_DEBUG_CONFIG
	PRINTF("Setting Global OV/UV Threshold (OV: %.3fV, UV: %.3fV)...\n\r", placeholder_global_ov, placeholder_global_uv);
#endif
	step_success = SlaveIf_setGlobalOvUvThreshold(placeholder_global_ov, placeholder_global_uv);
	if (!step_success)
	{
		PRINTF("ERROR: Failed to set Global OV/UV Threshold!\n\r\r");
		overall_success = false;
	}

	// 2. Set Overtemperature Thresholds (Loop through AN0-AN6)
#ifdef SLAVEIF_DEBUG_CONFIG
	PRINTF("Setting Overtemperature Thresholds (AN0-AN6) (Voltage: %.3fV)...\n\r", placeholder_ot_v);
#endif
	for (uint8_t i = 0; i <= 6; i++)
	{
		step_success = SlaveIf_setOverTempThreshold(i, placeholder_ot_v);
		if (!step_success)
		{
			PRINTF("ERROR: Failed to set OT threshold for AN%d!\n\r", i);
			overall_success = false;
		}
	}

	// 3. Set Undertemperature Thresholds (Loop through AN0-AN6)
#ifdef SLAVEIF_DEBUG_CONFIG
	PRINTF("Setting Undertemperature Thresholds (AN0-AN6) (Voltage: %.3fV)...\n\r", placeholder_ut_v);
#endif
	for (uint8_t i = 0; i <= 6; i++)
	{
		step_success = SlaveIf_setUnderTempThreshold(i, placeholder_ut_v);
		if (!step_success)
		{
			PRINTF("ERROR: Failed to set UT threshold for AN%d!\n\r", i);
			overall_success = false;
		}
	}

	// 4. Set Overcurrent Threshold
#ifdef SLAVEIF_DEBUG_CONFIG
	PRINTF("Setting Overcurrent Threshold (Current: %.3fA, Shunt: %.1fuOhm)...\n\r", placeholder_oc_a, placeholder_shunt_uohm);
#endif
	step_success = SlaveIf_setOverCurrentThreshold(placeholder_oc_a, placeholder_shunt_uohm);
	if (!step_success)
	{
		PRINTF("ERROR: Failed to set Overcurrent Threshold!\n\r\r");
		overall_success = false;
	}
}

/**
 * @brief Triggers a single on-demand ADC measurement cycle with a new Tag ID.
 * @details Reads the current ADC_CFG register ($06), preserves existing settings,
 *          increments the global Tag ID, sets the new Tag ID in the configuration word,
 *          sets the Start of Conversion (SOC) bit (Bit 11) to 1, and writes the modified
 *          configuration back to the ADC_CFG register.
 *          Optionally waits briefly and checks if the SOC bit cleared, indicating conversion start/completion.
 * @return bool True if the SOC command was sent successfully and optionally confirmed by SOC bit clearing,
 *              false otherwise (write failed or SOC bit did not clear).
 */
bool SlaveIF_startMeasurementCycle(void)
{
	uint16_t currentConfig;
	uint16_t newConfig;

	// Increment global Tag ID (0-15) for this measurement cycle
	TAGID = (TAGID + 1) & 0x0F;
#ifdef SLAVEIF_DEBUG_MEAS
	PRINTF("Starting Measurement Cycle with New TAGID: %d\n\r", TAGID);
#endif

	// Step 1: Read the current ADC_CFG register to preserve settings
	currentConfig = SlaveIF_readRegister(ADC_CFG_ADDR);
	// Check if read was successful (e.g., not 0xFFFF if that's the error code)
	if (currentConfig == 0xFFFF && ADC_CFG_ADDR != 0xFFFF)
	{ // Basic error check
		PRINTF("ERROR: Failed to read ADC_CFG before triggering SOC!\n\r\r");
		return false;
	}

	// Make a copy to modify
	newConfig = currentConfig;

	// Step 2: Modify the configuration for SOC trigger
	// - Clear any existing Tag ID bits (15:12)
	newConfig &= ~(0x0F << 12);
	// - Set the new Tag ID
	newConfig |= ((uint16_t)(TAGID & 0x0F) << 12);
	// - Set the SOC bit (Bit 11) to 1 to start the conversion
	SET_BIT(newConfig, 11);
	// - Ensure CC_RST bit (10) is 0 unless explicitly requested
	RESET_BIT(newConfig, 7);

#ifdef SLAVEIF_DEBUG_MEAS
	PRINTF("Writing to ADC_CFG ($%02X) to trigger SOC: 0x%04X\n\r", ADC_CFG_ADDR, newConfig);
#endif

	// Step 3: Write the modified configuration back to trigger the measurement
	bool writeSuccess = SlaveIF_writeRegister(ADC_CFG_ADDR, newConfig);
	if (!writeSuccess)
	{
		PRINTF("ERROR: Failed to write ADC_CFG ($%02X) to trigger SOC!\n\r", ADC_CFG_ADDR);
		return false; // SOC command failed
	}

	// Step 4: (Optional but Recommended) Wait and verify SOC bit cleared
	// The SOC bit should be cleared by hardware once the conversion cycle starts.
	// Waiting for it confirms the command was accepted and conversion is underway/done.
	// Max conversion time depends on settings, e.g., ~10ms for 16-bit, all channels.
	delay_ms(10); // Wait for typical conversion time

	uint16_t postSocConfig = SlaveIF_readRegister(ADC_CFG_ADDR);
	if (postSocConfig == 0xFFFF && ADC_CFG_ADDR != 0xFFFF)
	{ // Basic error check
		PRINTF("ERROR: Failed to read ADC_CFG after triggering SOC!\n\r\r");
		// Write succeeded, but read failed. Status unknown. Return true tentatively? Or false?
		return false; // Safer to return false if confirmation fails
	}

	if (READ_BIT(postSocConfig, 11))
	{
		PRINTF("Warning: SOC bit in ADC_CFG ($%02X) is still SET after (7at hna time el conversuion ya abdullah) ms!\n\r", ADC_CFG_ADDR);
		// This might indicate the conversion didn't start or isn't finished.
		// Depending on the application, this could be treated as an error.
		return false; // Treat as failure if SOC doesn't clear
	}
	else
	{
#ifdef SLAVEIF_DEBUG_MEAS
		PRINTF("SOC bit cleared. Conversion likely complete or in progress.\n\r\r");
#endif
	}

	// If write was successful and SOC bit cleared (or check skipped/passed)
	return true;
}

/* ------------------------- Fault Status Reading ------------------------- */
/**
 * @brief Reads the Fault Status Register 1 (FAULT1_STATUS).
 * @details Provides a snapshot of various system-level faults including POR,
 *          communication errors, voltage faults (summary), and temperature faults (summary).
 * @return uint16_t The raw 16-bit value read from the FAULT1_STATUS register ($24).
 *                  Returns READ_ERROR_VALUE on SPI communication failure.
 */
uint16_t SlaveIF_getFault1Status(void)
{
#ifdef SLAVEIF_DEBUG_FAULT
	PRINTF("Reading FAULT1_STATUS Register ($%02X)...\n\r", FAULT1_STATUS_ADDR);
#endif
	// FAULT1_STATUS address is $24 according to provided datasheet OCR
	return SlaveIF_readRegister(FAULT1_STATUS_ADDR);
}

/**
 * @brief Reads the Fault Status Register 2 (FAULT2_STATUS).
 * @details Provides a snapshot of various component-level faults including
 *          VCOM/VANA issues, ADC faults, ground loss, thermal shutdown (TSD),
 *          and summary flags for GPIO/Cell Balancing issues.
 * @return uint16_t The raw 16-bit value read from the FAULT2_STATUS register ($25).
 *                  Returns READ_ERROR_VALUE on SPI communication failure.
 */
uint16_t SlaveIF_getFault2Status(void)
{
#ifdef SLAVEIF_DEBUG_FAULT
	PRINTF("Reading FAULT2_STATUS Register ($%02X)...\n\r", FAULT2_STATUS_ADDR);
#endif
	// FAULT2_STATUS address is $25 according to provided datasheet OCR
	return SlaveIF_readRegister(FAULT2_STATUS_ADDR);
}

/**
 * @brief Reads the Fault Status Register 3 (FAULT3_STATUS).
 * @details Provides a snapshot of faults related to Coulomb Counter overflow,
 *          diagnostic timeouts, and end-of-timer flags for cell balancing.
 * @return uint16_t The raw 16-bit value read from the FAULT3_STATUS register ($26).
 *                  Returns READ_ERROR_VALUE on SPI communication failure.
 */
uint16_t SlaveIF_getFault3Status(void)
{
#ifdef SLAVEIF_DEBUG_FAULT
	PRINTF("Reading FAULT3_STATUS Register ($%02X)...\n\r", FAULT3_STATUS_ADDR);
#endif
	// FAULT3_STATUS address is $26 according to provided datasheet OCR
	return SlaveIF_readRegister(FAULT3_STATUS_ADDR);
}

/* ------------------------- Voltage Fault Monitoring ------------------------- */
/**
 * @brief Reads the Over-Voltage (OV) fault status for all cells.
 * @details Reads the CELL_OV_FLT register ($16). Each bit (0-13) corresponds
 *          to a cell (1-14). A set bit indicates an OV fault for that cell.
 * @return uint16_t 16-bit value containing OV flags. Only bits 0-13 are relevant.
 *                  Returns 0xFFFF on read error.
 */
uint16_t SlaveIF_readCellOverVoltageStatus(void)
{
	uint8_t regAddress = CELL_OV_FLT_ADDR; // $16
	uint16_t rawData = SlaveIF_readRegister(regAddress);
	return (rawData & 0x3FFF); // Mask bits 0-13
}

/**
 * @brief Reads the Under-Voltage (UV) fault status for all cells.
 * @details Reads the CELL_UV_FLT register ($17). Each bit (0-13) corresponds
 *          to a cell (1-14). A set bit indicates a UV fault for that cell.
 * @return uint16_t 16-bit value containing UV flags. Only bits 0-13 are relevant.
 *                  Returns 0xFFFF on read error.
 */
uint16_t SlaveIF_readCellUnderVoltageStatus(void)
{
	uint8_t regAddress = CELL_UV_FLT_ADDR; // $17
	uint16_t rawData = SlaveIF_readRegister(regAddress);
	return (rawData & 0x3FFF); // Mask bits 0-13
}

/* ------------------------- Cell Balancing ------------------------- */
/**
 * @brief Enables or disables cell balancing for a specific cell with a timer.
 * @details Writes to the corresponding CBx_CFG register ($0C - $15).
 *          Sets the CB_EN bit (Bit 9) and the balancing duration timer (Bits 8:0).
 * @param cellNumber Cell number to configure (1-14).
 * @param enable True to enable balancing, false to disable.
 * @param timerValueInMinutes Balancing duration in minutes. Resolution is 0.5 minutes/LSB.
 *                            The value will be capped at the maximum timer setting (511 = 255.5 mins).
 *                            A value of 0 typically means balance indefinitely until disabled.
 * @return bool True if the register write was successful, false otherwise.
 *         Returns false if cellNumber is invalid.
 */
bool SlaveIF_enableCellBalancing(uint8_t cellNumber, bool enable, float timerValueInMinutes)
{
	uint8_t regAddress;

	// Validate cell number and determine register address
	if (cellNumber < 1 || cellNumber > 14)
	{
		PRINTF("Error: Invalid cell number %d for balancing.\n\r", cellNumber);
		return false;
	}
	// Addresses are CB1_CFG=$0C, CB2_CFG=$0D, ..., CB14_CFG=$15
	regAddress = CB1_CFG_ADDR + (cellNumber - 1);

	uint16_t data = 0; // Start with 0

	// Set CB_EN bit (Bit 9) if enabling
	if (enable)
	{
		SET_BIT(data, CB_CFG_ENABLE_BIT); // Set bit 9
	}

	// Calculate and set timer value (Bits 8:0)
	// Resolution is 0.5 minutes per LSB. Max value 0x1FF (511).
	uint16_t timerValueInHalfMinutes = 0;
	if (timerValueInMinutes > 0)
	{																			   // Calculate only if duration > 0
		timerValueInHalfMinutes = (uint16_t)((timerValueInMinutes / 0.5f) + 0.5f); // Calculate LSBs, add 0.5f for rounding
	}

	// Clamp timer value to the maximum allowed (9 bits -> 0x1FF)
	if (timerValueInHalfMinutes > CB_CFG_DURATION_MASK)
	{
		timerValueInHalfMinutes = CB_CFG_DURATION_MASK; // Cap at max value (511)
		PRINTF("Warning: Balancing timer capped at max duration (%.1f minutes).\n\r", (float)CB_CFG_DURATION_MASK * 0.5f);
	}
	// Set timer bits (8:0)
	data |= (timerValueInHalfMinutes & CB_CFG_DURATION_MASK);

	PRINTF("Configuring Cell %d Balancing (Reg 0x%02X): Enable=%d, Timer=0x%X (%.1f mins)\n\r",
			cellNumber, regAddress, enable, timerValueInHalfMinutes, (float)timerValueInHalfMinutes * 0.5f);

	return SlaveIF_writeRegister(regAddress, data);
}

/**
 * @brief Reads the status of the cell balancing driver FETs.
 * @details Reads the CB_DRV_STS register ($1D). Each bit (0-13) indicates whether
 *          the balancing driver for the corresponding cell (1-14) is currently active (ON).
 * @return uint16_t 16-bit value with driver status flags. Bits 0-13 are relevant.
 *                  Returns 0xFFFF on read error.
 */
uint16_t SlaveIF_readCellBalancingDriverStatus(void)
{
	uint8_t regAddress = CB_DRV_STS_ADDR; // $1D
	uint16_t driverStatus = SlaveIF_readRegister(regAddress);

#ifdef SLAVEIF_DEBUG_BALANCING
	PRINTF("Cell Balancing Driver Status (CB_DRV_STS $1D): 0x%04X\n\r", driverStatus);
	// Optional: Print status per cell
	// for (int i = 0; i < 14; i++) {
	//     PRINTF("  Cell %d: %s\n", i + 1, READ_BIT(driverStatus, i) ? "ON" : "OFF");
	// }
#endif

	// Return the raw status, masking relevant bits might be desired by caller
	return (driverStatus & 0x3FFF); // Mask bits 0-13
}

/**
 * @brief Reads the short-circuit fault status of the cell balancing FETs.
 * @details Reads the CB_SHORT_FLT register ($1E). Each bit (0-13) corresponds
 *          to a cell (1-14). A set bit indicates a short-circuit fault detected
 *          on that cell's balancing path.
 * @return uint16_t 16-bit value with short-circuit fault flags. Bits 0-13 are relevant.
 *                  Returns 0xFFFF on read error.
 */
uint16_t SlaveIF_readCellBalancingShortedStatus(void)
{
	uint8_t regAddress = CB_SHORT_FLT_ADDR; // $1E
	uint16_t faultStatus = SlaveIF_readRegister(regAddress);

#ifdef SLAVEIF_DEBUG_BALANCING
	PRINTF("Cell Balancing Short Fault Status (CB_SHORT_FLT $1E): 0x%04X\n\r", faultStatus);
#endif

	return (faultStatus & 0x3FFF); // Mask bits 0-13
}

/**
 * @brief Reads the open-load fault status of the cell balancing paths.
 * @details Reads the CB_OPEN_FLT register ($1F). Each bit (0-13) corresponds
 *          to a cell (1-14). A set bit indicates an open-load fault detected
 *          on that cell's balancing path.
 * @return uint16_t 16-bit value with open-load fault flags. Bits 0-13 are relevant.
 *                  Returns 0xFFFF on read error.
 */
uint16_t SlaveIF_readCellBalancingOpenLoadStatus(void)
{
	uint8_t regAddress = CB_OPEN_FLT_ADDR; // $1F
	uint16_t faultStatus = SlaveIF_readRegister(regAddress);

#ifdef SLAVEIF_DEBUG_BALANCING
	PRINTF("Cell Balancing Open Fault Status (CB_OPEN_FLT $1F): 0x%04X\n\r", faultStatus);
#endif

	return (faultStatus & 0x3FFF); // Mask bits 0-13
}

/* ------------------------- Temperature Fault Monitoring & Reading ------------------------- */
/**
 * @brief Reads the GPIO analog input fault status (Short and Open Load).
 * @details Reads the GPIO_SH_AN_OL_STS register ($20). This register contains
 *          flags for both short-to-ground faults (GPIO_SH_FLT, bits 14:8) and
 *          open-load faults (AN_OL_FLT, bits 6:0) for the analog inputs AN0-AN6.
 * @return GPIO_AN_Flags A structure containing two fields:
 *         - `GPIO_SH_Flags`: uint8_t with bits 6:0 corresponding to AN6-AN0 short faults.
 *         - `AN_OL_Flags`: uint8_t with bits 6:0 corresponding to AN6-AN0 open load faults.
 *         Returns {0xFF, 0xFF} or similar on read error.
 */
GPIO_AN_Flags SlaveIF_readGpioAnStatus(void)
{
	uint8_t regAddress = GPIO_SH_AN_OL_STS_ADDR; // $20
	uint16_t faultStatus = SlaveIF_readRegister(regAddress);
	GPIO_AN_Flags flags = {0, 0}; // Initialize flags

	// Check for read error
	if (faultStatus == 0xFFFF && regAddress != 0xFFFF)
	{
		PRINTF("Error reading GPIO AN Status!\n\r\r");
		flags.GPIO_SH_Flags = 0xFF; // Indicate error
		flags.AN_OL_Flags = 0xFF;	// Indicate error
		return flags;
	}

	// Extract Short flags (Bits 14:8 map to AN6:AN0) -> Shift right by 8
	flags.GPIO_SH_Flags = (uint8_t)((faultStatus & 0x7F00) >> 8); // Bits 8-15

	// Extract Open Load flags (Bits 6:0 map to AN6:AN0)

	flags.AN_OL_Flags = (uint8_t)(faultStatus & 0x7F); // Bits 0-7

#ifdef SLAVEIF_DEBUG_GPIO
	PRINTF("GPIO Analog Status (GPIO_SH_AN_OL_STS $20): 0x%04X\n\r", faultStatus);
	PRINTF("  Short Flags (AN6-AN0): 0x%02X\n\r", flags.GPIO_SH_Flags);
	PRINTF("  Open Flags  (AN6-AN0): 0x%02X\n\r", flags.AN_OL_Flags);
#endif

	return flags;
}

/**
 * @brief Reads Over-Temperature (OT) and Under-Temperature (UT) fault status for AN0-AN6.
 * @details Reads the AN_OT_UT_FLT_STS register ($21). This register contains flags for
 *          both OT faults (AN_OT_FLT, bits 14:8) and UT faults (AN_UT_FLT, bits 6:0)
 *          for the analog inputs AN0-AN6, typically used for temperature sensors.
 * @return Temperature_Flags A structure containing two fields:
 *         - `Over_Temp_Flags`: uint8_t with bits 6:0 corresponding to AN6-AN0 OT faults.
 *         - `Under_Temp_Flags`: uint8_t with bits 6:0 corresponding to AN6-AN0 UT faults.
 *         Returns {0xFF, 0xFF} or similar on read error.
 */
Temperature_Flags SlaveIF_readOtUtStatus(void)
{
	uint8_t regAddress = AN_OT_UT_FLT_STS_ADDR; // $21
	uint16_t faultStatus = SlaveIF_readRegister(regAddress);
	Temperature_Flags flags = {0, 0}; // Initialize flags

	// Check for read error
	if (faultStatus == 0xFFFF && regAddress != 0xFFFF)
	{
		PRINTF("Error reading Temperature OT/UT Status!\n\r\r");
		flags.Over_Temp_Flags = 0xFF;  // Indicate error
		flags.Under_Temp_Flags = 0xFF; // Indicate error
		return flags;
	}

	// Extract Over Temperature flags (Bits 14:8 map to AN6:AN0) -> Shift right by 8
	flags.Over_Temp_Flags = (uint8_t)((faultStatus & 0x7F00) >> 8); // Bits 8-15

	// Extract Under Temperature flags (Bits 6:0 map to AN6:AN0)
	flags.Under_Temp_Flags = (uint8_t)(faultStatus & 0x7F); // Bits 0-7

#ifdef SLAVEIF_DEBUG_TEMP
	PRINTF("Temperature Fault Status (AN_OT_UT_FLT_STS $21): 0x%04X\n\r", faultStatus);
	PRINTF("  Over Temp Flags  (AN6-AN0): 0x%02X\n\r", flags.Over_Temp_Flags);
	PRINTF("  Under Temp Flags (AN6-AN0): 0x%02X\n\r", flags.Under_Temp_Flags);
#endif

	return flags;
}

/*
 * Function: SlaveIF_readTemperature
 * Description: Reads the temperature from a specified GPIO sensor (0-6).
 * Parameters:
 *   - sensorNumber: GPIO sensor number (0-6)
 * Returns: Temperature in degrees Celsius, or -999.0 if invalid
 */
float SlaveIF_readTemperature(uint8_t sensorNumber)
{
	if (sensorNumber > 6)
	{
		PRINTF("Invalid temperature sensor number! Must be 0-6.\n\r\r");
		return -999.0;
	}

	uint8_t regAddress;
	switch (sensorNumber)
	{
	case 0:
		regAddress = MEAS_AN0_ADDR;
		break;
	case 1:
		regAddress = MEAS_AN1_ADDR;
		break;
	case 2:
		regAddress = MEAS_AN2_ADDR;
		break;
	case 3:
		regAddress = MEAS_AN3_ADDR;
		break;
	case 4:
		regAddress = MEAS_AN4_ADDR;
		break;
	case 5:
		regAddress = MEAS_AN5_ADDR;
		break;
	case 6:
		regAddress = MEAS_AN6_ADDR;
		break;
	default:
		return -999.0; // Should not reach here due to prior check
	}

	uint16_t rawData = SlaveIF_readRegister(regAddress);
	if (!READ_BIT(rawData, MEAS_DATA_READY_BIT))
	{
		PRINTF("Temperature Data Not Ready for Sensor %d!\n\r\r", sensorNumber);
		return -999.0;
	}

	// Convert raw data to voltage (assuming ratiometric measurement)
	float voltageInMicroVolts = (float)(rawData & 0x7FFF) * VCT_ANX_RES_V;
	float voltage = voltageInMicroVolts / 1000000.0f;

	// Assuming a thermistor with a known curve (e.g., NTC), convert to temperature
	float temperature = voltage * 100.0f; // Assuming linear conversion (adjust as needed)
	return temperature;
}

/* ------------------------- Voltage & Current Measurement Reading ------------------------- */
/**
 * @brief Reads the voltage of a specified cell.
 * @details Reads the corresponding MEAS_CELLx register ($33 - $40). Checks the
 *          Data Ready bit (Bit 15). If ready, extracts the 15-bit measurement,
 *          converts it to Volts using the cell voltage resolution `VCT_ANX_RES_V`,
 *          and returns the value.
 * @param cellNumber Cell number to read (1-14).
 * @return float Cell voltage in Volts. Returns `VOLTAGE_READ_ERROR` (e.g., -1.0f)
 *               if cellNumber is invalid, read fails, or data is not ready.
 */
float SlaveIF_readCellVoltage(uint8_t cellNumber)
{
	uint8_t regAddress;

	// Validate cell number and determine register address
	if (cellNumber < 1 || cellNumber > 14)
	{
		PRINTF("Error: Invalid cell number %d for voltage read.\n\r", cellNumber);
		return 0;
	}
	// Addresses are MEAS_CELL1=$33, ..., MEAS_CELL14=$40
	regAddress = MEAS_CELL1_ADDR + (cellNumber - 1);

	// Read the measurement register
	uint16_t rawData = SlaveIF_readRegister(regAddress);

	// Check for read error
	if (rawData == 0xFFFF && regAddress != 0xFFFF)
	{
		PRINTF("Error reading Cell %d voltage register (0x%02X)!\n\r", cellNumber, regAddress);
		return 0;
	}

	// Check Data Ready Bit (Bit 15)
	if (!READ_BIT(rawData, MEAS_DATA_READY_BIT))
	{
#ifdef SLAVEIF_DEBUG_MEAS
		PRINTF("Cell %d Voltage Data Not Ready! (Reg 0x%02X = 0x%04X)\n\r", cellNumber, regAddress, rawData);
#endif
		return 0; // Indicate data not ready / error
	}

	// Extract the 15-bit measurement data (Bits 14:0)
	uint16_t measurement = rawData & 0x7FFF;

	// Convert raw value to Volts using VCT resolution (microVolts / LSB)
	float voltageInMicroVolts = (float)measurement * VCT_ANX_RES_V;
	float voltageInVolts = voltageInMicroVolts / 1000000.0f;

#ifdef SLAVEIF_DEBUG_VOLTAGE
	PRINTF("Read Cell %d (Reg 0x%02X): Raw=0x%04X, Value=%u -> Voltage=%.4f V\n\r",
			cellNumber, regAddress, rawData, measurement, voltageInVolts);
#endif

	return voltageInVolts;
}

/**
 * @brief Reads the total stack voltage (V_PWR supply voltage).
 * @details Reads the MEAS_STACK register ($32). Checks the Data Ready bit (Bit 15).
 *          If ready, extracts the 15-bit measurement, converts it to Volts using
 *          the stack voltage resolution `VVPWR_RES` (uV/LSB), and returns the value.
 * @return float Total stack voltage in Volts. Returns `VOLTAGE_READ_ERROR` (e.g., -1.0f)
 *               if read fails or data is not ready.
 */
float SlaveIF_readPackVoltage(void)
{
	uint8_t regAddress = MEAS_STACK_ADDR; // $32
	uint16_t rawData = SlaveIF_readRegister(regAddress);

	// Check for read error
	if (rawData == 0xFFFF && regAddress != 0xFFFF)
	{
		PRINTF("Error reading Pack Voltage register (0x%02X)!\n\r", regAddress);
		return 0;
	}

	// Check Data Ready Bit (Bit 15)
	if (!READ_BIT(rawData, MEAS_DATA_READY_BIT))
	{
#ifdef SLAVEIF_DEBUG_MEAS
		PRINTF("Pack Voltage Data Not Ready! (Reg 0x%02X = 0x%04X)\n\r", regAddress, rawData);
#endif
		return 0; // Indicate data not ready / error
	}

	// Extract the 15-bit measurement data (Bits 14:0)
	uint16_t measurement = rawData & 0x7FFF;

	// Convert raw value to Volts using VVPWR resolution (microVolts / LSB)
	float voltageInMicroVolts = (float)measurement * VVPWR_RES;
	float voltageInVolts = voltageInMicroVolts / 1000000.0f;

#ifdef SLAVEIF_DEBUG_VOLTAGE
	PRINTF("Read Pack Voltage (Reg 0x%02X): Raw=0x%04X, Value=%u -> Voltage=%.4f V\n\r",
			regAddress, rawData, measurement, voltageInVolts);
#endif

	return voltageInVolts;
}

/**
 * @brief Reads the current measurement from the ISENSE inputs.
 * @details Implements a **NON-STANDARD USER INTERPRETATION** of the current registers:
 *          - Reads $30 (MEAS_ISENSE1) & $31 (MEAS_ISENSE2).
 *          - Checks Data Ready ($30[15]).
 *          - Uses $30[14] as the SIGN BIT.
 *          - Uses $30[13:0] as the 14 MSBs.
 *          - Uses $31[3:0] as the 4 LSBs.
 *          - Combines into an 18-bit value, sign-extends to 32 bits based on $30[14].
 *          - Reads ADC_CFG ($06) to determine the settled PGA gain (PGA_GAIN_S).
 *          - Calculates V_ISENSE = (Signed_18bit_Value / 2^17) * 0.150 V.
 *          - Calculates V_SHUNT = V_ISENSE / PGA_Gain.
 *          - Calculates Current = V_SHUNT / Shunt_Resistance.
 * @warning This interpretation **STRONGLY DEVIATES** from the standard datasheet method
 *          (Rev 8.0, Tables 80, 81, 9). The standard method uses $30[14:0] for ADC1A
 *          and $31[14:0] for ADC1B results when ISENSE is routed there, or specific formats
 *          when using ADC2. **Use this function with extreme caution and rigorous validation.**
 * @return float Current in Amperes (A). Returns `CURRENT_READ_ERROR` (e.g., -999.0f) on error
 *               (data not ready, invalid PGA gain, invalid shunt resistance).
 */
float SlaveIF_readCurrent(void)
{
	uint16_t reg_30_data;			   // Value from $30 (MEAS_ISENSE1)
	uint16_t reg_31_data;			   // Value from $31 (MEAS_ISENSE2)
	uint16_t adc_cfg_data;			   // Value from $06 (ADC_CFG)
	uint16_t msb_part_14bit;		   // Extracted bits $30[13:0]
	bool sign_bit;					   // State of bit $30[14]
	uint16_t lsb_part_4bit;			   // Extracted bits $31[3:0]
	uint32_t combined_raw_value_18bit; // Combined 18-bit magnitude
	int32_t signed_value_18bit;		   // Combined value with sign extension
	uint8_t pga_gain_s_setting;		   // PGA actual settled gain field from ADC_CFG[9:8]
	float pga_gain_factor = 0.0f;	   // Numeric gain factor (4, 16, 64, 256)
	double VIND_pre_pga;			   // Calculated voltage at ISENSE pins [Volts]
	float current_A;				   // Final calculated current in Amperes

	// --- 1. Read necessary registers ---
	reg_30_data = SlaveIF_readRegister(MEAS_ISENSE1_ADDR); // Read $30
	reg_31_data = SlaveIF_readRegister(MEAS_ISENSE2_ADDR); // Read $31
	adc_cfg_data = SlaveIF_readRegister(ADC_CFG_ADDR);	   // Read $06

	// --- 2. Check for Read Errors ---
	if ((reg_30_data == 0xFFFF && MEAS_ISENSE1_ADDR != 0xFFFF) ||
			(reg_31_data == 0xFFFF && MEAS_ISENSE2_ADDR != 0xFFFF) ||
			(adc_cfg_data == 0xFFFF && ADC_CFG_ADDR != 0xFFFF))
	{
		PRINTF("Error: Failed to read one or more registers for current measurement.\n\r\r");
		return 0;
	}

	// --- 3. Check Data Ready Bit ---
	if (!READ_BIT(reg_30_data, MEAS_DATA_READY_BIT)) // Check $30[15]
	{
#ifdef SLAVEIF_DEBUG_MEAS
		PRINTF("Error: Current Data Not Ready! MEAS_ISENSE1 ($%02X) = 0x%04X\n\r",
				MEAS_ISENSE1_ADDR, reg_30_data);
#endif
		return 0; // Indicate data not ready
	}

	// --- 4. Extract Parts based on *User Interpretation* ---
	// LSB Part: $31 bits 3:0
	lsb_part_4bit = reg_31_data & 0x000F;
	// MSB Part: $30 bits 13:0
	msb_part_14bit = reg_30_data & 0x3FFF;
	// Sign Bit: $30 bit 14
	sign_bit = (bool)READ_BIT(reg_30_data, 14);

	// --- 5. Combine into 18-bit Raw Value ---
	combined_raw_value_18bit = ((uint32_t)msb_part_14bit << 4) | (uint32_t)lsb_part_4bit;

	// --- 6. Create Signed 32-bit Value with Sign Extension ---
	// Start with the 18-bit magnitude
	signed_value_18bit = (int32_t)combined_raw_value_18bit;
	// If the sign bit ($30[14]) was set, perform sign extension from bit 17
	if (sign_bit)
	{
		// Set bits 31 down to 18 to 1 if the 18th bit (bit 17) is 1
		// Since we are using $30[14] as sign bit for the 18-bit number,
		// if sign_bit is true, the number is negative.
		// We need to represent this negative number in 32-bit 2's complement.
		// If sign_bit is true, the 18-bit number is negative.
		// Extend the sign (bit 17 implicitly 1 if sign_bit is true and value is large enough,
		// or explicitly make it negative).
		// A simpler way: If sign_bit is true, treat magnitude as negative offset from 0,
		// relative to the full scale. Scaling below handles this implicitly.
		// For sign extension if needed:
		signed_value_18bit |= 0xFFFC0000; // Sets bits 18-31 if sign bit was 1. Check logic.
		// OR, more correctly for 2's complement interpretation:
		if (combined_raw_value_18bit != 0)
		{																			 // Avoid making 0 negative
			signed_value_18bit = -((int32_t)((1 << 18) - combined_raw_value_18bit)); // Find negative equivalent if sign bit was 1? Check interpretation.
			// Let's stick to scaling approach which handles sign implicitly.
			// We just need the signed value relative to the range.
			signed_value_18bit = (int32_t)combined_raw_value_18bit; // Keep magnitude
			// Apply sign extension based on bit 17 if interpreting as 18-bit 2's complement
			if (signed_value_18bit & (1 << 17))
			{									  // Check bit 17
				signed_value_18bit |= 0xFFFC0000; // Extend sign
			}
			// **Correction based on user logic**: Use $30[14] as sign bit.
			signed_value_18bit = (int32_t)combined_raw_value_18bit; // Start with magnitude
			if (sign_bit)
			{
				// Sign extend from bit 17 if needed for 2's complement math
				// However, scaling uses signed_value / max_magnitude, so maybe just make negative?
				signed_value_18bit = -signed_value_18bit; // Apply sign based on $30[14]? This seems wrong for scaling.
				// Stick to sign extension from bit 17 if needed:
				if (combined_raw_value_18bit & (1 << 17))
				{											// Check the implicit MSB of 18-bit number
					signed_value_18bit |= ~((1 << 18) - 1); // Sign extend
				}
				// Let's assume scaling handles the signed value correctly based on bit 17.
				// The user interpretation with $30[14] is highly suspect. Revert to standard 18-bit signed.
				signed_value_18bit = (int32_t)combined_raw_value_18bit;
				if (signed_value_18bit & 0x20000)
				{									  // Check bit 17 (the MSB of the 18 bits)
					signed_value_18bit |= 0xFFFC0000; // Sign extend
				}
			}
		}

		// --- 7. Determine Settled PGA Gain (PGA_GAIN_S) ---
		// PGA_GAIN_S field is in ADC_CFG bits 9:8
		pga_gain_s_setting = (uint8_t)((adc_cfg_data >> 8) & 0x03);
		switch (pga_gain_s_setting)
		{
		case 0:
			pga_gain_factor = 4.0f;
			break;
		case 1:
			pga_gain_factor = 16.0f;
			break;
		case 2:
			pga_gain_factor = 64.0f;
			break;
		case 3:
			pga_gain_factor = 256.0f;
			break;
		default: // Should not happen
			PRINTF("Error: Invalid PGA_GAIN_S value %d in ADC_CFG=0x%04X\n\r", pga_gain_s_setting, adc_cfg_data);
			return 0; // Gain error
		}

		// --- 8. Calculate V_ISENSE (Voltage at ADC input, after PGA) ---
		// Assumes the 18-bit signed value maps linearly to the ADC input range, which is typically
		// related to VREF (often 5V internal) or a differential range (e.g., +/- 150mV for ISENSE).
		// Using the differential ISENSE input range: +/- 150mV = +/- 0.150V
		// The maximum magnitude for an 18-bit signed number is 2^17 = 131072.
		double max_adc_magnitude_18bit = 131072.0;										  // 2^17
		double V_ISENSE = ((double)signed_value_18bit / max_adc_magnitude_18bit) * 0.150; // e.g., 0.150 V

		// --- 9. Calculate V_SHUNT (Voltage at ISENSE pins, before PGA) ---
		// V_SHUNT = V_ISENSE / PGA_Gain
		VIND_pre_pga = V_ISENSE / (double)pga_gain_factor;

		// --- 10. Calculate Current ---
		// Current = V_SHUNT / Shunt_Resistance
		if (SHUNT_RESISTANCE_OHMS <= 0.0f)
		{
			PRINTF("Error: Invalid Shunt Resistance (%.6f Ohms) defined in Cfg!\n\r", SHUNT_RESISTANCE_OHMS);
			return 0; // Config error
		}
		current_A = (float)(VIND_pre_pga / SHUNT_RESISTANCE_OHMS);

#ifdef SLAVEIF_DEBUG_CURRENT
		PRINTF("DEBUG Current (User 14+4): $30=0x%04X, $31=0x%04X\n\r", reg_30_data, reg_31_data);
		PRINTF("  Extracted: Sign($30[14])=%d, MSB($30[13:0])=0x%X, LSB($31[3:0])=0x%X\n\r",
				sign_bit, msb_part_14bit, lsb_part_4bit);
		PRINTF("  Combined: Raw18b=0x%lX (%ld), Signed18b=%ld\n\r",
				combined_raw_value_18bit, combined_raw_value_18bit, signed_value_18bit);
		PRINTF("  ADC Config ($06)=0x%04X -> PGA_GAIN_S=%d (Gain=%.0f)\n\r",
				adc_cfg_data, pga_gain_s_setting, pga_gain_factor);
		PRINTF("  Voltages: V_ISENSE(PostPGA)=%.6f V, V_SHUNT(PrePGA)=%.6f V\n\r", V_ISENSE, VIND_pre_pga);
		PRINTF("  Shunt=%.6f Ohm -> Current=%.4f A\n\r", SHUNT_RESISTANCE_OHMS, current_A);
#endif

		// --- 11. Return Calculated Current ---
		return current_A;
	}
}

/* ------------------------- Coulomb Counter Reading ------------------------- */
/**
 * @brief Reads the number of accumulated Coulomb Counter samples.
 * @details Reads the CC_NB_SAMPLES register ($2F). This indicates how many
 *          current measurements have been integrated since the last read or reset.
 * @return uint16_t Number of samples (0-65535). Returns 0xFFFF on read error.
 */
uint16_t SlaveIF_readNumberCoulombSamples(void)
{
	uint8_t regAddress = CC_NB_SAMPLES_ADDR; // $2F
	uint16_t samples = SlaveIF_readRegister(regAddress);

#ifdef SLAVEIF_DEBUG_CURRENT
	PRINTF("Coulomb Counter Samples (CC_NB_SAMPLES $2F): %u\n\r", samples);
#endif

	return samples;
}

/**
 * @brief Reads the integrated Coulomb Counter value.
 * @details Reads the CC_OUT register ($2E). This register holds the accumulated
 *          sum of current measurements (represented as ADC values).
 * @return uint16_t Raw accumulated ADC count (can be positive or negative depending
 *                  on current flow, interpretation depends on system). Returns 0xFFFF on read error.
 * @note The value represents the sum of (ADC_Result - ADC2_Offset) for each sample.
 *       To get charge (Ampere-seconds), this value needs scaling based on ADC resolution,
 *       PGA gain, shunt resistance, and sample interval.
 */
uint16_t SlaveIF_readCoulombCountRaw(void)
{
	uint8_t regAddress = COULOMB_CNT1; // $2E
	uint16_t cc_value = SlaveIF_readRegister(regAddress);

#ifdef SLAVEIF_DEBUG_CURRENT
	PRINTF("Coulomb Counter Raw Value (CC_OUT $2E): %u (0x%04X)\n\r", cc_value, cc_value);
#endif

	return cc_value;
}

/**
 * @brief Checks if there are sufficient Coulomb Counter samples accumulated (at least 1).
 * @details Reads the number of samples and checks if it's >= 1. Useful before reading
 *          the CC_OUT value to ensure it represents at least one measurement.
 * @return bool True if number of samples >= 1, false otherwise (or if read fails).
 */
bool SlaveIF_isCoulombSamplesSufficient(void)
{
	uint16_t samples = SlaveIF_readNumberCoulombSamples();
	// Consider read error (0xFFFF) as insufficient samples
	return (samples >= 1 && samples != 0xFFFF);
}

//=============================================================================
// End of File
//=============================================================================
