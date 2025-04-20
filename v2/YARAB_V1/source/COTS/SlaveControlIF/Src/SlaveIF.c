/*
 * Graduation Project: Battery Management System
 * Engineer: Abdullah Mohamed
 * Component: Slave_Control_IF driver
 * File: SlaveIF.c
 * Description: This file contains the implementation of the Slave Interface driver
 *              for communication with the MC33771A slave via the MC33664 transceiver
 *              using SPI on the KL25Z microcontroller.
 */

#include <COTS/SlaveControlIF/Inc/SlaveIF.h>
#include <COTS/SlaveControlIF/Inc/SlaveIF_Cfg.h>

// --- Constants based on Data sheet MC33771A Rev 8.0 ---
// Resolution for OV/UV Thresholds (TH_ALL_CT, TH_CTx)
static const float OVUV_RESOLUTION_MV_PER_LSB = 19.53125f;
// Resolution for OT/UT Thresholds (TH_ANx_OT, TH_ANx_UT)
static const float OTUT_RESOLUTION_MV_PER_LSB = 4.8828125f;
// Resolution for Over current Threshold (TH_ISENSE_OC)
static const float OC_RESOLUTION_UV_PER_LSB = 1.2f; // microV/LSB


static void SlaveIF_writeEnPin(uint8_t value);
static uint32_t SlaveIF_readIntbPin(void);
bool SlaveIF_tplEnable(void);
bool RegisterHasTagID(uint8_t regAddr);
volatile uint8_t TAGID = 0;





/* ========================= Transceiver APIs ========================= */

/*
 * Function: SlaveIF_writeEnPin
 * Description: Writes a logic value (0 or 1) to the EN pin of the MC33664 transceiver.
 *              This function is used to enable or disable the TPL communication.
 * Parameters:
 *   - value: Logic value to write (0 for low, 1 for high)
 * Returns: None
 */
static void SlaveIF_writeEnPin(uint8_t value)
{
	// Initialize the EN pin (GPIOE, Pin 0) if not already initialized
	static bool isInitialized = false;
	if (!isInitialized)
	{
		GPIO_PinInit(enTransceiverGPIO, enTransceiverPin, &enPinConfig);
		isInitialized = true;
	}

	delay_us(100);
	// Write the specified value to the EN pin
	GPIO_WritePinOutput(enTransceiverGPIO, enTransceiverPin, value);

	// Add a small delay to ensure MC33664 is ready (~100 us)
	delay_us(100); // Adjust based on CPU clock
}

/*
 * Function: SlaveIF_readIntbPin
 * Description: Reads the logic value of the INTB pin of the MC33664 transceiver.
 *              This function is used to check if the transceiver has data ready or an error.
 * Returns: uint32_t - Logic value of the INTB pin (0 for low, 1 for high)
 */
static uint32_t SlaveIF_readIntbPin(void)
{
	// Initialize the INTB pin (GPIOA, Pin 12) if not already initialized
	static bool isInitialized = false;
	if (!isInitialized)
	{
		GPIO_PinInit(intbTransceiverGPIO, intbTransceiverPin, &intbPinConfig);
		isInitialized = true;
	}

	// Read the INTB pin value
	return (GPIO_ReadPinInput(intbTransceiverGPIO, intbTransceiverPin));
}

/*
 * Function: SlaveIF_tplEnable
 * Description: Enables the MC33664 transceiver by setting the EN pin high and optionally checking the INTB pin.
 *              This function is intended for TPL mode only.
 * Returns: bool - Returns true if the EN pin is set successfully, false otherwise.
 */
bool SlaveIF_tplEnable(void)
{
	// Set the EN pin to high (enable the MC33664)
	SlaveIF_writeEnPin(1);

	// Optionally check the INTB pin (commented out as per requirement)
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











/* ========================= Private Helper Functions ========================= */
/*
 * Function: calculateCrc8
 * Description: Computes the CRC-8 checksum for SPI frame data as specified in the MC33664 datasheet.
 * Parameters:
 *   - data: Pointer to the data array
 *   - data_len: Length of the data array
 * Returns: Calculated CRC-8 value
 */
static uint8_t calculateCrc8(uint8_t *data, uint16_t data_len)
{
	uint8_t crc = 0xFF; // Initial seed value
	for (uint16_t i = 0; i < data_len; i++) {
		uint8_t tbl_idx = (crc ^ data[i]) & 0xFF;
		crc = crc_table[tbl_idx]; // Lookup table method
	}
	return crc;
}

/* ========================= SPI Communication Variables ========================= */
// DMA handles for SPI TX and RX
static dma_handle_t dmaTxHandle;
static dma_handle_t dmaRxHandle;
static spi_dma_handle_t spiDmaTxHandle;
static spi_dma_handle_t spiDmaRxHandle;

// Callback for DMA transfer completion
static volatile bool spiDmaTxCompleted = false;
static volatile bool spiDmaRxCompleted = false;

static void spiDmaTxCallback(SPI_Type *base, spi_dma_handle_t *handle, status_t status, void *userData) {
	PRINTF("DMA TX Callback Triggered! Status: %d\n\r\r", status);
	if (status == kStatus_Success) {
		PRINTF("DMA TX Transfer Success!\n\r\r");
	} else {
		PRINTF("DMA TX Transfer Failed! Status: %d\n\r\r", status);
	}
	spiDmaTxCompleted = true;
}

static void spiDmaRxCallback(SPI_Type *base, spi_dma_handle_t *handle, status_t status, void *userData) {
	PRINTF("DMA RX Callback Triggered! Status: %d\n\r\r", status);
	if (status == kStatus_Success) {
		PRINTF("DMA RX Transfer Success!\n\r\r");
	} else {
		PRINTF("DMA RX Transfer Failed! Status: %d\n\r\r", status);
	}
	spiDmaRxCompleted = true;
}

// Reset SPI0 function
static void resetSpi0(void) {
	PRINTF("Resetting SPI0...\n\r\r");
	SPI_Deinit(SPI_TX);
	spi_master_config_t masterConfig;
	SPI_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = 2000000;                  // 2 Mbps
	masterConfig.polarity = kSPI_ClockPolarityActiveHigh; // CPOL = 1
	masterConfig.phase = kSPI_ClockPhaseSecondEdge;       // CPHA = 1
	masterConfig.direction = kSPI_MsbFirst;               // MSB first
	masterConfig.outputMode = kSPI_SlaveSelectAutomaticOutput; // Enable automatic CS control
	CLOCK_EnableClock(kCLOCK_Spi0);
	SPI_MasterInit(SPI_TX, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));
	SPI_MasterTransferCreateHandleDMA(SPI_TX, &spiDmaTxHandle, spiDmaTxCallback, NULL, &dmaTxHandle, &dmaRxHandle);
}

void SlaveIF_initTransfer(void) {
	spi_master_config_t masterConfig;
	spi_slave_config_t slaveConfig;

	// --- Enable DMA clock ---
	CLOCK_EnableClock(kCLOCK_Dma0); // Enable DMA0 clock

	// --- Initialize DMA ---
	DMA_Init(DMA0);

	// --- Initialize DMA MUX ---
	DMAMUX_Init(DMAMUX0);
	DMAMUX_SetSource(DMAMUX0, 0, kDmaRequestMux0SPI0Tx); // Channel 0 for SPI0 TX
	DMAMUX_SetSource(DMAMUX0, 1, kDmaRequestMux0SPI1Rx); // Channel 1 for SPI1 RX
	DMAMUX_EnableChannel(DMAMUX0, 0); // Enable Channel 0 (TX)
	DMAMUX_EnableChannel(DMAMUX0, 1); // Enable Channel 1 (RX)

	// --- Create DMA handles ---
	DMA_CreateHandle(&dmaTxHandle, DMA0, 0); // Channel 0 for TX
	DMA_CreateHandle(&dmaRxHandle, DMA0, 1); // Channel 1 for RX

	// Enable DMA interrupts in NVIC
	NVIC_EnableIRQ(DMA0_IRQn);
	NVIC_SetPriority(DMA0_IRQn, 0);

	// --- Configure SPI0 (TX) as master ---
	SPI_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = 2000000;                  // 2 Mbps
	masterConfig.polarity = kSPI_ClockPolarityActiveHigh; // CPOL = 1
	masterConfig.phase = kSPI_ClockPhaseSecondEdge;       // CPHA = 1
	masterConfig.direction = kSPI_LsbFirst;               // MSB first
	masterConfig.outputMode = kSPI_SlaveSelectAutomaticOutput; // Enable automatic CS control
	CLOCK_EnableClock(kCLOCK_Spi0);
	SPI_MasterInit(SPI_TX, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));
	SPI_MasterTransferCreateHandleDMA(SPI_TX, &spiDmaTxHandle, spiDmaTxCallback, NULL, &dmaTxHandle, &dmaRxHandle);

	// --- Configure SPI1 (RX) as slave ---
	SPI_SlaveGetDefaultConfig(&slaveConfig);
	slaveConfig.polarity = kSPI_ClockPolarityActiveHigh; // CPOL = 1
	slaveConfig.phase = kSPI_ClockPhaseSecondEdge;       // CPHA = 1
	slaveConfig.direction = kSPI_LsbFirst;               // MSB first
	CLOCK_EnableClock(kCLOCK_Spi1);
	SPI_SlaveInit(SPI_RX, &slaveConfig);
	SPI_SlaveTransferCreateHandleDMA(SPI_RX, &spiDmaRxHandle, spiDmaRxCallback, NULL, &dmaTxHandle, &dmaRxHandle);
	delay_ms(5);
}

// DMA Interrupt Handler
void DMA0_IRQHandler(void) {
	PRINTF("DMA0 Interrupt Triggered!\n\r\r");
	// Call the FSL DMA driver IRQ handler
	DMA0_DriverIRQHandler();
}


uint8_t gRollingCounter ;
/*
 * Function: SlaveIF_writeRegister
 * Description: Writes data to a specified MC33771C register via SPI0 using DMA.
 *              Supports Rolling Counter (RC) for communication synchronization.
 * Parameters:
 *   - regAddress: Register address to write to
 *   - data: 16-bit data to write
 * Returns: bool - Returns true if the write operation is successful, false otherwise
 */
static bool SlaveIF_writeRegister(uint8_t regAddress, uint16_t data) {
	uint8_t txFrame[BUFFER_SIZE] = {0};
	uint8_t rxFrame[BUFFER_SIZE] = {0}; // Dummy buffer

	const uint8_t CID = 0x01;         // Cluster ID
	const uint8_t Write_CMD = 0x02;    // Command = Read

	uint8_t rc = gRollingCounter & 0x03;
	gRollingCounter = (gRollingCounter + 1) & 0x03;
	// Construct read request frame
	txFrame[1] = (CID<<4) | (rc << 2) | (Write_CMD & 0x03); // Read command (0001) + Cluster ID + RC
	txFrame[2] = (regAddress & 0x7F) | 0x80;       // Address + Master bit
	txFrame[3] = (data >> 8) & 0xFF;               // Data high byte
	txFrame[4] = data & 0xFF;                      // Data low byte
	txFrame[0] = calculateCrc8(&txFrame[1], 4);    // CRC-8

	PRINTF("Step 2: Frame constructed: [%02X, %02X, %02X, %02X, %02X]\n\r\r",
			txFrame[0], txFrame[1], txFrame[2], txFrame[3], txFrame[4]);

	// Step 3: Initialize spiXfer
	PRINTF("Step 3: Initializing spiXfer...\n\r\r");
	spi_transfer_t spiXfer = {
			.txData = txFrame,
			.rxData = rxFrame,
			.dataSize = BUFFER_SIZE
	};

	// Step 4: Wait until SPI is not busy
	uint32_t busyTimeout = 5000000;
	PRINTF("Step 4: Checking SPI status: 0x%08X\n\r\r", SPI_GetStatusFlags(SPI_TX));
	while ((SPI_GetStatusFlags(SPI_TX) & kSPI_TxBufferEmptyFlag) == 0 && busyTimeout--) {}
	if (busyTimeout == 0) {
		PRINTF("SPI Busy Timeout! Cannot start new transfer.\n\r\r");
		resetSpi0(); // Reset SPI on timeout
		return false;
	}

	// Step 5: Start DMA Transfer with retry mechanism
	uint8_t retryCount = 5;
	status_t status;
	PRINTF("Step 5: Starting DMA transfer...\n\r\r");
	do {
		spiDmaTxCompleted = false;
		status = SPI_MasterTransferDMA(SPI_TX, &spiDmaTxHandle, &spiXfer);
		if (status != kStatus_Success) {
			PRINTF("SPI DMA TX Failed! Status: %d\n\r\r", status);
			for (volatile int i = 0; i < 10000; i++); // Delay before retry
			retryCount--;
		} else {
			break;
		}
	} while (retryCount > 0);

	if (status != kStatus_Success) {
		PRINTF("SPI DMA TX Failed After Retries! Status: %d\n\r\r", status);
		resetSpi0(); // Reset SPI on failure
		return false;
	}

	// Step 6: Wait for DMA transfer to complete
	uint32_t timeout = 10;
	PRINTF("Step 6: Waiting for DMA to complete...\n\r\r");
	while (!spiDmaTxCompleted && timeout--) {}
	if (!spiDmaTxCompleted) {
		PRINTF("SPI DMA Write Timeout!\n\r\r");
		SPI_MasterTransferAbortDMA(SPI_TX, &spiDmaTxHandle); // Abort DMA transfer
		//resetSpi0(); // Reset SPI on timeout
		delay_ms(10);
		return false;
	}

	PRINTF("Step 7: DMA Transfer Completed.\n\r\r");
	uint8_t receivedRC = (rxFrame[1] >> 2) & 0x03;
	uint8_t expectedRC = (gRollingCounter - 1) & 0x03; // RC was incremented after frame was built
	if (receivedRC != rc) {
		PRINTF("RC mismatch! Expected: %u, Got: %u\n\r", expectedRC, receivedRC);
		return 0;
	}
	// Step 8: Add delay between frames (at least 100 us as per MC33664 datasheet)
	PRINTF("Step 8: Adding delay between frames...\n\r\r");
	for (volatile int i = 0; i < 5000; i++); // Delay ~100 us (adjust based on your clock speed)

	PRINTF("Step 9: Transfer successful.\n\r\r");
	return true;
}

/*
 * Function: SlaveIF_readRegister
 * Description: Reads data from a specified MC33771C register using SPI0 (TX) and SPI1 (RX) with DMA.
 *              Supports Rolling Counter (RC) for communication synchronization.
 * Parameters:
 *   - regAddress: Register address to read from
 * Returns: uint16_t - 16-bit data read from the register, or 0 if the operation fails
 */
static uint16_t SlaveIF_readRegister(uint8_t regAddress) {
    uint8_t txFrame[BUFFER_SIZE] = {0};
    uint8_t rxFrame[BUFFER_SIZE] = {0};

    const uint8_t CID = 0x01;         // Cluster ID
    const uint8_t READ_CMD = 0x01;    // Command = Read

    uint8_t rc = gRollingCounter & 0x03;
    gRollingCounter = (gRollingCounter + 1) & 0x03;
    // Construct read request frame
    txFrame[1] = (CID << 4) | (rc << 2) | (READ_CMD & 0x03); // Read command (0001) + Cluster ID + RC
    txFrame[2] = (regAddress & 0x7F) | 0x80;  // Address + Master bit
    txFrame[3] = 0x00;                        // Placeholder
    txFrame[4] = 0x00;                        // Placeholder
    txFrame[0] = calculateCrc8(&txFrame[1], 4); // CRC-8

    spi_transfer_t spiTxXfer = {
        .txData = txFrame,
        .rxData = NULL,
        .dataSize = BUFFER_SIZE
    };

    spi_transfer_t spiRxXfer = {
        .txData = NULL,
        .rxData = rxFrame,
        .dataSize = BUFFER_SIZE
    };

    // Send read request (DMA)
    spiDmaTxCompleted = false;
    status_t status = SPI_MasterTransferDMA(SPI_TX, &spiDmaTxHandle, &spiTxXfer);
    if (status != kStatus_Success) {
        PRINTF("SPI DMA TX Failed! Status: %d\n\r\r", status);
        return 0;
    }

    // Wait for DMA transfer to complete
    uint32_t timeout = 1000000;
    while (!spiDmaTxCompleted && timeout--) {}
    if (!spiDmaTxCompleted) {
        PRINTF("SPI DMA TX Timeout!\n\r\r");
        return 0;
    }


    // Receive response (DMA)
    spiDmaRxCompleted = false;
    status = SPI_SlaveTransferDMA(SPI_RX, &spiDmaRxHandle, &spiRxXfer);
    if (status != kStatus_Success) {
        PRINTF("SPI DMA RX Failed! Status: %d\n\r\r", status);
        return 0;
    }

    // Wait for DMA transfer to complete
    timeout = 1000000;
    while (!spiDmaRxCompleted && timeout--) {}
    if (!spiDmaRxCompleted) {
        PRINTF("SPI DMA RX Timeout!\n\r\r");
        return 0;
    }


    // Verify CRC
    uint8_t crcReceived = rxFrame[0];
    uint8_t crcCalculated = calculateCrc8(&rxFrame[1], 4);
    if (crcReceived != crcCalculated) {
        PRINTF("CRC Check Failed! Received: %02X, Calculated: %02X\n\r\r", crcReceived, crcCalculated);
        return 0;
    }
    // Optional TAG_ID check
    if (RegisterHasTagID(regAddress)) {
        uint8_t tagIdReceived = rxFrame[1] & 0x0F;
        if (tagIdReceived != (TAGID & 0x0F)) {
            PRINTF("TAG_ID mismatch for 0x%02X! Expected: %X, Got: %X\n\r",
                   regAddress, TAGID, tagIdReceived);
            return 0;
        }
    }else { uint8_t receivedRC = (rxFrame[1] >> 2) & 0x03;
    uint8_t expectedRC = (gRollingCounter - 1) & 0x03; // RC was incremented after frame was built
    if (receivedRC != rc) {
        PRINTF("RC mismatch! Expected: %u, Got: %u\n\r", expectedRC, receivedRC);
        return 0;
    }

    }

    // Add delay to ensure MC33664 timing requirements (e.g., 100 us between frames)
    for (volatile int i = 0; i < 1000; i++); // Adjust delay as needed

    return (rxFrame[3] << 8) | rxFrame[4]; // Combine high and low bytes
}


bool RegisterHasTagID(uint8_t regAddr)
{
    switch (regAddr)
    {
        case MEAS_ISENSE1_ADDR:
        case MEAS_ISENSE2_ADDR:
        case MEAS_STACK_ADDR:
        case MEAS_CELL1_ADDR: case MEAS_CELL2_ADDR: case MEAS_CELL3_ADDR: case MEAS_CELL4_ADDR:
        case MEAS_CELL5_ADDR: case MEAS_CELL6_ADDR: case MEAS_CELL7_ADDR:
        case MEAS_CELL8_ADDR: case MEAS_CELL9_ADDR: case MEAS_CELL10_ADDR:
        case MEAS_CELL11_ADDR: case MEAS_CELL12_ADDR: case MEAS_CELL13_ADDR: case MEAS_CELL14_ADDR:
        case MEAS_AN0_ADDR: case MEAS_AN1_ADDR: case MEAS_AN2_ADDR: case MEAS_AN3_ADDR:
        case MEAS_AN4_ADDR: case MEAS_AN5_ADDR: case MEAS_AN6_ADDR:
        case MEAS_IC_TEMP:
        case MEAS_VBG_DIAG_ADC1A:
        case MEAS_VBG_DIAG_ADC1B:
            return true;

        default:
            return false;
    }
}

/* ========================= System Wake up ========================= */
/**
 * @brief Sends a wake-up signal to the MC33771A via the MC33664 transceiver using SPI.
 * @details This function sends a simple "No Operation" (NOP) command over SPI to generate
 *          activity on the TPL line, which wakes up the MC33771B from Sleep Mode.
 *          The NOP command is used as it does not modify any registers and is safe to send.
 * @return bool True if the wake-up command was sent successfully, false otherwise.
 * @note This assumes the MC33664 is properly configured and the EN pin is HIGH.
 *       The MC33771B will wake up upon detecting any activity on the TPL line.
 */

void SlaveIF_wakeUp(void)
{
	PORT_SetPinMux(PORTC, 4U, kPORT_MuxAsGpio);
	/* First Low-to-High Transition */
	GPIO_WritePinOutput(GPIOC, 4U, false); /* CSB Low */
	delay_us(15); /* Wait ~19 µs */
	GPIO_WritePinOutput(GPIOC, 4U, true);  /* CSB High */
	delay_us(600); /* Wait ~600 µs */

	/* Second Low-to-High Transition */
	GPIO_WritePinOutput(GPIOC, 4U, false); /* CSB Low */
	delay_us(15); /* Wait ~19 µs */
	GPIO_WritePinOutput(GPIOC, 4U, true);  /* CSB High */
	delay_us(1000); /* Wait ~1000 µs */
	PORT_SetPinMux(PORTC, 4U, kPORT_MuxAlt2);
}

/* ========================= System Configuration APIs ========================= */

/*
 * Function: SlaveIF_setupSystem
 * Description: Configures the initialization register with Cluster ID and bus switch settings.
 * Parameters:
 *   - channelSwitch: True to enable bus switch, false to disable
 */
bool SlaveIF_setupSystem(bool channelSwitch)
{
	uint8_t regAddress = INIT_REGISTER;
	uint16_t data = (DEFAULT_CID & 0x000F); // Set CID (bits 0-3)
	if (channelSwitch) {
		SET_BIT(data, 4); // Enable BUS_SW (bit 4)
	}
	return (SlaveIF_writeRegister(regAddress, data));
}

/**
 * @brief Sets the MC33771A device to enter or exit sleep mode.
 * @details Modifies the GO2SLEEP bit in the SYS_CFG_GLOBAL register.
 * @param enterSleepMode Set to true to command the device to enter sleep mode
 *                       after the current operation/conversion completes.
 *                       Set to false is typically not used to exit sleep (wake-up
 *                       is usually triggered by communication activity or specific pins),
 *                       but could potentially clear a pending sleep request.
 * @note Writes to the SYS_CFG_GLOBAL register ($02). Only affects bit 0 (GO2SLEEP).
 */
void SlaveIF_setSleepMode(bool sleepMode) {
	uint8_t regAddress = SYS_CFG_GLOBAL_ADDR;
	uint16_t data = 0;
	if (sleepMode) {
		SET_BIT(data, 0); // Set SLEEP bit
	}
	SlaveIF_writeRegister(regAddress, data);
}

/*
 * Function: SlaveIF_configSystem1
 * Description: Configures System Configuration Register 1 with predefined settings.
 */
bool SlaveIF_configSystem1(void) {
	uint8_t regAddress = SYS_CFG1_ADDR;
	uint16_t data = 0x9381; // Example configuration (adjust as needed)
	return (SlaveIF_writeRegister(regAddress, data));
}

/*
 * Function: SlaveIF_configSystem2
 * Description: Configures System Configuration Register 2 with predefined settings.
 */
bool SlaveIF_configSystem2(void) {
	uint8_t regAddress = SYS_CFG2_ADDR;
	uint16_t data = 0x6231; // Example configuration (adjust as needed)
	return (SlaveIF_writeRegister(regAddress, data));
}

/* ========================= ADC Configuration APIs ========================= */
/**
 * @brief Configures the ADC resolutions and the current sense PGA settings.
 * @details Writes to the ADC_CFG register ($06). Sets all ADCs (ADC1A, ADC1B, ADC2)
 *          to 16-bit resolution and configures the ADC2 PGA for automatic gain
 *          control, starting at 4x. Ensures SOC bit remains 0.
 * @note Assumes ADC_CFG_ADDR, ADC_CFG_RESOLUTION_16BIT, ADC_CFG_PGA_GAIN_AUTO_4X,
 *       ADC_CFG_PGA_GAIN_SHIFT macros/constants are defined. Refer to Table 50.
 */
bool SlaveIF_configAdc(void)
{
	uint8_t regAddress = ADC_CFG_ADDR; // $06
	uint16_t data = 0; // Initialize with all bits 0
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
 * @brief Configures the ADC2 offset compensation register.
 * @details Writes to the ADC2_OFFSET_COMP ($07) register. Sets the PCB offset
 *          and configures Coulomb Counter behavior (using bit manipulation macros).
 * @param pcbOffset Signed 8-bit value (-128 to 127) for PCB offset compensation.
 *                  Usually requires system calibration. Use 0 if unsure.
 * (Based on Datasheet Rev 8.0, Table 51)
 */
bool SlaveIF_configAdcOffset(int8_t pcbOffset) {
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
	if (!SlaveIF_writeRegister(regAddress, data)) {
		PRINTF("Error writing to ADC2_OFFSET_COMP register!\n\r\r");
		return false;
		// Handle error appropriately
	} else {
		PRINTF("ADC2_OFFSET_COMP register configured successfully.\n\r\r");
		return true;
	}
}


/*
 * Function: SlaveIF_init
 * Description: Initializes the MC33771C slave device by calling all necessary configuration functions.
 *              This function sets up the system, configures registers, ADC, OV/UV protection,
 *              and GPIOs for temperature sensors. It is intended to be called at the start of the main function.
 * Returns: bool - Returns true if all configuration steps succeed, false if any step fails.
 */
bool SlaveIF_init(void) {
	bool success = true;

	PRINTF("Configuring MC33771B Slave Device...\n\r\r");

	// Step 1: Set up the system with bus switch enabled
	if (!SlaveIF_setupSystem(true))
	{
		PRINTF("Error: Failed to set up system configuration!\n\r\r");
		success = false;
	}

	// Step 2: Configure System Configuration Register 1
	if (! SlaveIF_configSystem1())
	{
		PRINTF("Error: Failed to configure SYS_CFG1!\n\r\r");
		success = false;
	}

	// Step 3: Configure System Configuration Register 2

	if (!SlaveIF_configSystem2())
	{
		PRINTF("Error: Failed to configure SYS_CFG2!\n\r\r");
		success = false;
	}

	// Step 4: Configure ADC settings
	if (!SlaveIF_configAdc())
	{
		PRINTF("Error: Failed to configure ADC!\n\r\r");
		success = false;
	}

	// Step 5: Configure ADC offset
	if (!SlaveIF_configAdcOffset(1))
	{
		PRINTF("Error: Failed to configure ADC offset!\n\r\r");
		success = false;
	}

	// Step 6: Enable Over-Voltage and Under-Voltage protection
	if (!SlaveIF_enableOvUv())
	{
		PRINTF("Error: Failed to enable OV/UV protection!\n\r\r");
		success = false;
	}

	// Step 7: Configure GPIOs for temperature sensors
	if (!SlaveIF_configAllGpiosForTempSensors())
	{
		PRINTF("Error: Failed to configure GPIOs for temperature sensors!\n\r\r");
		success = false;
	}

	if (success)
		PRINTF("MC33771C Basic Configuration Complete.\n\r\r");
	else
		PRINTF("MC33771C Configuration Failed!\n\r\r");

	return success;
}


/*
 * Function: SlaveIF_startMeasurementCycle
 * Description: Triggers a single on-demand measurement cycle for the MC33771C with an incremented Tag ID.
 *              Reads the current ADC_CFG, preserves settings, sets the Start of Conversion (SOC) bit,
 *              and updates the Tag ID.
 * Returns: bool - Returns true if the SOC command is sent successfully, false otherwise
 */
bool SlaveIF_startMeasurementCycle(void)
{
    uint16_t currentConfig;
    uint16_t newConfig;
    TAGID = (TAGID + 1) & 0x0F;

    // Read the current ADC_CFG register to preserve settings
    currentConfig = SlaveIF_readRegister(ADC_CFG_ADDR);

    // Modify the TAGID configuration
    newConfig &= ~(0x0F << 12);
    newConfig |= ((uint16_t)(TAGID & 0x0F) << 12);

    // Set the SOC bit (Bit 11) to 1 to start the conversion
    SET_BIT(newConfig,11);

    // Step 3: Write the modified configuration back to trigger the measurement
    bool writeSuccess = SlaveIF_writeRegister(ADC_CFG_ADDR, newConfig);
    if (!writeSuccess) {
        PRINTF("ERROR: Failed to write ADC_CFG (0x%02X) to trigger SOC!\n\r\r", ADC_CFG_ADDR);
        return false; // SOC command failed
    }
    //delaymillisseconds(14);
    newConfig = SlaveIF_readRegister(ADC_CFG_ADDR);
    if ((newConfig & 0x0800) == 0) {
           PRINTF("Conversion is done");
       }else {
           PRINTF("Error While Converting");
           return false;
       }

    // Optional success message for debugging:
    // PRINTF("SOC command sent successfully to ADC_CFG (0x%02X) with Tag ID 0.\n\r\r", ADC_CFG_ADDR);

    // If write was successful, the command was sent
    return true;
}
/* ========================= Voltage Fault Monitoring APIs ========================= */

/*
 * Function: SlaveIF_enableOvUv
 * Description: Enables overvoltage and undervoltage detection for all cells.
 */
bool SlaveIF_enableOvUv(void) {
	uint8_t regAddress = OV_UV_EN_ADDR;
	uint16_t data = 0xFFFF; // Enable all cells
	return (SlaveIF_writeRegister(regAddress, data));
}

/*
 * Function: SlaveIF_readCellOverVoltageStatus
 * Description: Reads the overvoltage status for all cells.
 * Returns: 16-bit value with overvoltage flags (bits 0-13)
 */
uint16_t SlaveIF_readCellOverVoltageStatus(void) {
	uint8_t regAddress = CELL_OV_FLT_ADDR;
	uint16_t rawData = SlaveIF_readRegister(regAddress);
	return rawData & 0x3FFF; // Mask bits 0-13
}

/*
 * Function: SlaveIF_readCellUnderVoltageStatus
 * Description: Reads the undervoltage status for all cells.
 * Returns: 16-bit value with undervoltage flags (bits 0-13)
 */
uint16_t SlaveIF_readCellUnderVoltageStatus(void) {
	uint8_t regAddress = CELL_UV_FLT_ADDR;
	uint16_t rawData = SlaveIF_readRegister(regAddress);
	return rawData & 0x3FFF; // Mask bits 0-13
}

/* ========================= Cell Balancing APIs ========================= */

/*
 * Function: SlaveIF_enableCellBalancing
 * Description: Enables or disables cell balancing for a specific cell with a timer.
 * Parameters:
 *   - cellNumber: Cell number (1-14)
 *   - enable: True to enable, false to disable
 *   - timerValueInMinutes: Balancing duration in minutes
 */
void SlaveIF_enableCellBalancing(uint8_t cellNumber, bool enable, float timerValueInMinutes) {
	uint8_t regAddress;
	switch (cellNumber) {
	case 1:  regAddress = CB1_CFG_ADDR;  break;
	case 2:  regAddress = CB2_CFG_ADDR;  break;
	case 3:  regAddress = CB3_CFG_ADDR;  break;
	case 4:  regAddress = CB4_CFG_ADDR;  break;
	case 5:  regAddress = CB5_CFG_ADDR;  break;
	case 6:  regAddress = CB6_CFG_ADDR;  break;
	case 7:  regAddress = CB7_CFG_ADDR;  break;
	case 8:  regAddress = CB8_CFG_ADDR;  break;
	case 9:  regAddress = CB9_CFG_ADDR;  break;
	case 10: regAddress = CB10_CFG_ADDR; break;
	case 11: regAddress = CB11_CFG_ADDR; break;
	case 12: regAddress = CB12_CFG_ADDR; break;
	case 13: regAddress = CB13_CFG_ADDR; break;
	case 14: regAddress = CB14_CFG_ADDR; break;
	default: return; // Invalid cell number
	}

	uint16_t data = 0;
	if (enable) {
		SET_BIT(data, 9); // Enable CB_EN (bit 9)
	}

	uint16_t timerValueInHalfMinutes = (uint16_t)(timerValueInMinutes / 0.5);
	if (timerValueInHalfMinutes > CB_CFG_DURATION_MASK) {
		timerValueInHalfMinutes = CB_CFG_DURATION_MASK; // Cap at max value
	}
	data |= (timerValueInHalfMinutes & CB_CFG_DURATION_MASK); // Set timer bits

	SlaveIF_writeRegister(regAddress, data);
}

/*
 * Function: SlaveIF_readCellBalancingDriverStatus
 * Description: Reads and prints the status of cell balancing drivers for all cells.
 */
void SlaveIF_readCellBalancingDriverStatus(void) {
	uint8_t regAddress = CB_DRV_STS_ADDR;
	uint16_t driverStatus = SlaveIF_readRegister(regAddress);

	PRINTF("Cell Balancing Driver Status:\n");
	for (int i = 1; i <= 14; i++) {
		uint8_t isEnabled = READ_BIT(driverStatus, (i - 1));
		PRINTF("Cell %d: %s\n", i, isEnabled ? "ON" : "OFF");
	}
}

/*
 * Function: SlaveIF_readCellBalancingShortedStatus
 * Description: Reads the shorted status of cell balancing circuits.
 * Returns: 16-bit value with shorted flags
 */
uint16_t SlaveIF_readCellBalancingShortedStatus(void) {
	uint8_t regAddress = CB_SHORT_FLT_ADDR;
	return SlaveIF_readRegister(regAddress);
}

/*
 * Function: SlaveIF_readCellBalancingOpenLoadStatus
 * Description: Reads the open load status of cell balancing circuits.
 * Returns: 16-bit value with open load flags
 */
uint16_t SlaveIF_readCellBalancingOpenLoadStatus(void) {
	uint8_t regAddress = CB_OPEN_FLT_ADDR;
	return SlaveIF_readRegister(regAddress);
}

/* ========================= GPIO Configuration APIs ========================= */

/*
 * Function: SlaveIF_configAllGpiosForTempSensors
 * Description: Configures all GPIO pins (0-6) as analog inputs for temperature sensors.
 */
bool SlaveIF_configAllGpiosForTempSensors(void) {
	uint8_t regAddress = GPIO_CFG1_ADDR;
	uint16_t gpioCfg1Value = 0x0000; // All GPIOs as analog inputs (00 per pair)
	return (SlaveIF_writeRegister(regAddress, gpioCfg1Value));
}

/* ========================= GPIO Flag APIs ========================= */

/*
 * Function: SlaveIF_readGpioAnStatus
 * Description: Reads the GPIO analog status (short and open load flags).
 * Returns: Structure containing GPIO short and open load flags
 */
GPIO_AN_Flags SlaveIF_readGpioAnStatus(void) {
	uint8_t regAddress = GPIO_SH_AN_OL_STS_ADDR;
	uint16_t faultStatus = SlaveIF_readRegister(regAddress);

	GPIO_AN_Flags flags;
	flags.GPIO_SH_Flags = (uint8_t)((faultStatus & 0x7F00) >> 8); // Bits 8-15
	flags.AN_OL_Flags = (uint8_t)(faultStatus & 0x7F);           // Bits 0-7
	return flags;
}

/* ========================= Temperature APIs ========================= */

/*
 * Function: SlaveIF_readOtUtStatus
 * Description: Reads over-temperature and under-temperature status for all GPIO sensors.
 * Returns: Structure containing OT and UT flags
 */
Temperature_Flags SlaveIF_readOtUtStatus(void) {
	uint8_t regAddress = AN_OT_UT_FLT_STS_ADDR;
	uint16_t faultStatus = SlaveIF_readRegister(regAddress);

	Temperature_Flags flags;
	flags.Over_Temp_Flags = (uint8_t)((faultStatus & 0x7F00) >> 8); // Bits 8-15
	flags.Under_Temp_Flags = (uint8_t)(faultStatus & 0x7F);         // Bits 0-7
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
	case 0: regAddress = MEAS_AN0_ADDR; break;
	case 1: regAddress = MEAS_AN1_ADDR; break;
	case 2: regAddress = MEAS_AN2_ADDR; break;
	case 3: regAddress = MEAS_AN3_ADDR; break;
	case 4: regAddress = MEAS_AN4_ADDR; break;
	case 5: regAddress = MEAS_AN5_ADDR; break;
	case 6: regAddress = MEAS_AN6_ADDR; break;
	default: return -999.0; // Should not reach here due to prior check
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

/* ========================= Coulomb Counter APIs ========================= */

/*
 * Function: SlaveIF_readNumberCoulombSamples
 * Description: Reads the number of coulomb counter samples accumulated.
 * Returns: Number of samples
 */
uint16_t SlaveIF_readNumberCoulombSamples(void) {
	uint8_t regAddress = CC_NB_SAMPLES_ADDR;
	return SlaveIF_readRegister(regAddress);
}

/*
 * Function: SlaveIF_isCoulombSamplesSufficient
 * Description: Checks if there are sufficient coulomb counter samples.
 * Returns: True if samples >= 1, false otherwise
 */
bool SlaveIF_isCoulombSamplesSufficient(void) {
	return (SlaveIF_readNumberCoulombSamples() >= 1);
}

/* ========================= Measurement APIs ========================= */

/*
 * Function: SlaveIF_readCellVoltage
 * Description: Reads the voltage of a specified cell.
 * Parameters:
 *   - cellNumber: Cell number (1-14)
 * Returns: Voltage in volts, or 0 if invalid
 */
float SlaveIF_readCellVoltage(uint8_t cellNumber) {
	uint8_t regAddress;
	switch (cellNumber) {
	case 1: regAddress = MEAS_CELL1_ADDR; break;
	case 2: regAddress = MEAS_CELL2_ADDR; break;
	case 3: regAddress = MEAS_CELL3_ADDR; break;
	case 4: regAddress = MEAS_CELL4_ADDR; break;
	case 5: regAddress = MEAS_CELL5_ADDR; break;
	case 6: regAddress = MEAS_CELL6_ADDR; break;
	case 7: regAddress = MEAS_CELL7_ADDR; break;
	case 8: regAddress = MEAS_CELL8_ADDR; break;
	case 9: regAddress = MEAS_CELL9_ADDR; break;
	case 10: regAddress = MEAS_CELL10_ADDR; break;
	case 11: regAddress = MEAS_CELL11_ADDR; break;
	case 12: regAddress = MEAS_CELL12_ADDR; break;
	case 13: regAddress = MEAS_CELL13_ADDR; break;
	case 14: regAddress = MEAS_CELL14_ADDR; break;
	default:
		PRINTF("Invalid cell number!\n\r\r");
		return 0.0;
	}

	uint16_t rawData = SlaveIF_readRegister(regAddress);
	if (!READ_BIT(rawData, MEAS_DATA_READY_BIT)) {
		PRINTF("Cell Voltage Data Not Ready for Cell %d!\n\r\r", cellNumber);
		return 0.0;
	}

	float voltageInMicroVolts = (float)(rawData & 0x7FFF) * VCT_ANX_RES_V;
	return voltageInMicroVolts / 1000000.0f; // Convert to volts
}

/*
 * Function: SlaveIF_readPackVoltage
 * Description: Reads the total stack voltage of the battery pack.
 * Returns: Voltage in volts, or -1.0 if data not ready
 */
float SlaveIF_readPackVoltage(void) {
	uint8_t regAddress = MEAS_STACK_ADDR;
	uint16_t rawData = SlaveIF_readRegister(regAddress);

	if (!(READ_BIT(rawData, MEAS_DATA_READY_BIT))) {
		PRINTF("Stack Voltage Data Not Ready!\n\r\r");
#ifdef DebugInfoManager
		DebugInfo;
#endif
		return -1.0;
	}

	float voltageInMicroVolts = (float)(rawData & 0x7FFF) * VVPWR_RES;
	return voltageInMicroVolts / 1000000.0f; // Convert to volts
}

/**
 * @brief Reads current combining 14 MSB (+$30 bit 14 as sign) + 4 LSB ($31) -> 18-bit signed value (USER INTERPRETATION).
 * @details This function implements a user-specified interpretation of current reading:
 *          - Reads $30 (MEAS_ISENSE1) & $31 (MEAS_ISENSE2).
 *          - Checks Data Ready ($30[15]).
 *          - Uses $30[14] as the SIGN BIT for the combined value.
 *          - Uses $30[13:0] as the 14 MSBs.
 *          - Uses $31[3:0] as the 4 LSBs.
 *          - Combines into an 18-bit value and sign-extends to 32 bits.
 *          - Determines PGA gain from $06 (ADC_CFG).
 *          - Scales the 18-bit signed value relative to ±150mV input range *and* PGA gain to find VIND.
 *          - Calculates current using Ohm's law.
 *          !!! WARNING: This method STRONGLY DEVIATES from the standard interpretation
 *              of datasheet Rev 8.0 (Tables 80, 81, 9). Use with extreme caution and validation. !!!
 * @return Current in amperes (A), or a distinct negative value signifying an error.
 */
float SlaveIF_readCurrent(void) {
	uint16_t reg_30_data;            // Value from $30 (MEAS_ISENSE1)
	uint16_t reg_31_data;            // Value from $31 (MEAS_ISENSE2)
	uint16_t adc_cfg_data;           // Value from $06 (ADC_CFG)
	uint16_t msb_part_14bit;         // Extracted bits 0-13 from $30
	bool sign_bit;                   // State of bit 14 from $30
	uint16_t lsb_part_4bit;          // Extracted bits 0-3 from $31
	uint32_t combined_raw_value_18bit; // Combined 18-bit value
	int32_t signed_value_18bit;      // Combined 18-bit value with sign
	uint8_t pga_gain_setting;        // PGA setting field from ADC_CFG
	uint8_t pga_gain_s_setting;      // PGA actual settled gain if Auto-Gain
	float pga_gain_factor = 0.0f;    // Numeric gain factor
	double VIND_calculated;          // Input voltage at ISENSE pins [Volts]
	float current_A = -1000.0f;      // Initialize current

	// --- 1. Read necessary registers ---
	reg_30_data = SlaveIF_readRegister(MEAS_ISENSE1_ADDR); // Read $30
	reg_31_data = SlaveIF_readRegister(MEAS_ISENSE2_ADDR); // Read $31
	adc_cfg_data = SlaveIF_readRegister(ADC_CFG_ADDR);    // Read $06

	// --- 2. Check Data Ready ---
	if (!READ_BIT(reg_30_data, MEAS_DATA_READY_BIT)) { // Check $30[15]
		PRINTF("Error: Current Data Not Ready! MEAS_ISENSE1 ($%02X) = 0x%04X\n\r\r",
				MEAS_ISENSE1_ADDR, reg_30_data);
		return -999.0f; // Error code
	}

	// --- 3. Extract LSB Part (User: $31 bits 0-3) ---
#define USER_LSB_MASK_4BIT 0x000F
	lsb_part_4bit = reg_31_data & USER_LSB_MASK_4BIT;

	// --- 4. Extract MSB Part (User: $30 bits 0-13) ---
#define USER_MSB_MASK_14BIT 0x3FFF
	msb_part_14bit = reg_30_data & USER_MSB_MASK_14BIT;

	// --- 5. Extract Sign Bit (User: $30 bit 14) ---
	sign_bit = (bool)READ_BIT(reg_30_data, 14);

	// --- 6. Combine into 18-bit Raw Value ---
	combined_raw_value_18bit = ((uint32_t)msb_part_14bit << 4) | (uint32_t)lsb_part_4bit;

	// --- 7. Create Signed 32-bit Value with Sign Extension from $30[14] ---
	signed_value_18bit = (int32_t)combined_raw_value_18bit; // Cast first
	if (sign_bit) {
		// Sign extend from bit 17 upwards (since it's now an 18-bit value in a 32-bit container)
		signed_value_18bit |= 0xFFFC0000; // Set bits 18 through 31 to 1
		// Additionally, apply two's complement logic if interpreting the magnitude directly
		// However, scaling relative to full range handles negative values implicitly
		// For direct two's complement value (if needed elsewhere):
		// if (signed_value_18bit > 0) { // If still positive after sign bit indicated negative
		// Perform 2's complement manually for 18 bits if interpreting this way
		//    signed_value_18bit = -( (~combined_raw_value_18bit + 1) & 0x3FFFF ); // ~invert, +1, mask 18 bits
		// }
	}

	// --- 8. Determine PGA Gain ---
	pga_gain_setting = (uint8_t)((adc_cfg_data >> ADC_CFG_PGA_GAIN_SHIFT) & 0x0F);
	switch (pga_gain_setting) {
	case ADC_CFG_PGA_GAIN_4X:   pga_gain_factor = 4.0f; break;
	case ADC_CFG_PGA_GAIN_16X:  pga_gain_factor = 16.0f; break;
	case ADC_CFG_PGA_GAIN_64X:  pga_gain_factor = 64.0f; break;
	case ADC_CFG_PGA_GAIN_256X: pga_gain_factor = 256.0f; break;
	case ADC_CFG_PGA_GAIN_AUTO_4X: // Includes other Auto starts potentially
	default:
		pga_gain_s_setting = (uint8_t)((adc_cfg_data >> 8) & 0x03); // PGA_GAIN_S field
		switch(pga_gain_s_setting) {
		case 0: pga_gain_factor = 4.0f;   break;
		case 1: pga_gain_factor = 16.0f;  break;
		case 2: pga_gain_factor = 64.0f;  break;
		case 3: pga_gain_factor = 256.0f; break;
		}
		break;
	}

	if (pga_gain_factor == 0.0f) {
		PRINTF("Error: Invalid PGA Gain. ADC_CFG($06)=0x%04X\n\r\r", adc_cfg_data);
		return -666.0f; // Gain error
	}


	// --- 9. Calculate VIND (Voltage before PGA) ---
	// Scaling approach: relate the 18-bit signed value to the ADC's full range,
	// map it to the VIND full range (±150mV = ±0.150V), and divide by PGA gain.
	// Max absolute value for 18-bit signed is 2^17.
	double max_adc_magnitude_18bit = pow(2.0, 17);
	// Calculate voltage proportional to input range AFTER gain stage
	VIND_calculated = ((double)signed_value_18bit / max_adc_magnitude_18bit) * 0.150;
	// Calculate voltage BEFORE gain stage (actual voltage at ISENSE pins)
	VIND_calculated = VIND_calculated / (double)pga_gain_factor;

	// --- 10. Calculate Current ---
	if (SHUNT_RESISTANCE_OHMS <= 0.0f) {
		PRINTF("Error: Invalid Shunt resistance defined (%.6f Ohms)!\n\r\r", SHUNT_RESISTANCE_OHMS);
		return -777.0f; // Config error
	}
	current_A = (float)(VIND_calculated / SHUNT_RESISTANCE_OHMS);

	// --- 11. Debug Print (Optional) ---
	PRINTF("DEBUG (User 14+4): $30=0x%04X, $31=0x%04X -> Sign=%d, MSB=0x%X, LSB=0x%X -> Signed18b=%ld -> Gain=%.1f -> VIND=%.6f V -> I=%.4f A\n\r\r",
			reg_30_data, reg_31_data, sign_bit, msb_part_14bit, lsb_part_4bit,
			signed_value_18bit, pga_gain_factor, VIND_calculated, current_A);

	// --- 12. Return Calculated Current ---
	return current_A;
}


/* ========================= Private Functions to calculate Thresholds ========================= */
/**
 * @brief Calculates the register value for setting Over voltage (OV) and Under voltage (UV) thresholds.
 */
static uint16_t SlaveIf_CalculateOvUvThresholdReg(float ov_volts, float uv_volts) {
	float ov_mv = ov_volts * 1000.0f;
	float uv_mv = uv_volts * 1000.0f;
	float ovScaledValueF = ov_mv / OVUV_RESOLUTION_MV_PER_LSB;
	float uvScaledValueF = uv_mv / OVUV_RESOLUTION_MV_PER_LSB;
	uint8_t ovInteger = (ovScaledValueF < 0.0f) ? 0 : ((ovScaledValueF > 255.0f) ? 255 : (uint8_t)(ovScaledValueF + 0.5f));
	uint8_t uvInteger = (uvScaledValueF < 0.0f) ? 0 : ((uvScaledValueF > 255.0f) ? 255 : (uint8_t)(uvScaledValueF + 0.5f));
	uint16_t registerValue = ((uint16_t)ovInteger << 8) | (uint16_t)uvInteger;
	return registerValue;
}

/**
 * @brief Calculates the register value for setting the Over temperature (OT) threshold.
 */
static uint16_t SlaveIf_CalculateOverTempThresholdReg(float overTemp_inVolt) {
	float ot_mv = overTemp_inVolt * 1000.0f;
	float otScaledValueF = ot_mv / OTUT_RESOLUTION_MV_PER_LSB;
	uint16_t otInteger = (otScaledValueF < 0.0f) ? 0 : ((otScaledValueF > 1023.0f) ? 1023 : (uint16_t)(otScaledValueF + 0.5f));
	return otInteger; // Value occupies bits 9-0
}

/**
 * @brief Calculates the register value for setting the Under temperature (UT) threshold.
 */
static uint16_t SlaveIf_CalculateUnderTempThresholdReg(float underTemp_inVolt) {
	float ut_mv = underTemp_inVolt * 1000.0f;
	float utScaledValueF = ut_mv / OTUT_RESOLUTION_MV_PER_LSB;
	uint16_t utInteger = (utScaledValueF < 0.0f) ? 0 : ((utScaledValueF > 1023.0f) ? 1023 : (uint16_t)(utScaledValueF + 0.5f));
	return utInteger; // Value occupies bits 9-0
}

/**
 * @brief Calculates the register value for setting the Over current (OC) threshold.
 */
static uint16_t SlaveIf_CalculateOverCurrentThresholdReg(float overCurrent_amps, float shunt_resistance_micro_ohms) {
	float overVolt_in_shuntResistor_uV = overCurrent_amps * shunt_resistance_micro_ohms;
	float ocScaledValueF = overVolt_in_shuntResistor_uV / OC_RESOLUTION_UV_PER_LSB;
	uint16_t ocInteger = (ocScaledValueF < 0.0f) ? 0 : ((ocScaledValueF > 4095.0f) ? 4095 : (uint16_t)(ocScaledValueF + 0.5f));
	return ocInteger; // Value occupies bits 11-0
}


/* ========================= Public Functions to calculate Thresholds ========================= */
/**
 * @brief Calculates and sets the global Over voltage (OV) and Under voltage (UV) threshold register (TH_ALL_CT).
 */
bool SlaveIf_setGlobalOvUvThreshold(float ov_volts, float uv_volts) {
	uint16_t regData = SlaveIf_CalculateOvUvThresholdReg(ov_volts, uv_volts);
	PRINTF("Setting Global OV/UV Threshold (Reg 0x%02X) to: 0x%04X (OV: %.3fV, UV: %.3fV)\n\r\r", TH_ALL_CT_ADDR, regData, ov_volts, uv_volts);
	return SlaveIF_writeRegister(TH_ALL_CT_ADDR, regData);
}

/**
 * @brief Calculates and sets the Over voltage (OV) and Under voltage (UV) threshold register for a specific cell (TH_CTx).
 */
bool SlaveIf_setCellOvUvThreshold(uint8_t cell_index, float ov_volts, float uv_volts) {
	if (cell_index < 1 || cell_index > 14) {
		PRINTF("Error: Invalid cell index %d for OV/UV threshold.\n\r\r", cell_index);
		return false;
	}
	// Datasheet addresses are descending: TH_CT1 = 0x59, TH_CT2=0x58, ..., TH_CT14=0x4C
	uint8_t regAddress = TH_CT1_ADDR - (cell_index - 1);
	uint16_t regData = SlaveIf_CalculateOvUvThresholdReg(ov_volts, uv_volts);
	PRINTF("Setting Cell %d OV/UV Threshold (Reg 0x%02X) to: 0x%04X (OV: %.3fV, UV: %.3fV)\n\r\r", cell_index, regAddress, regData, ov_volts, uv_volts);
	return SlaveIF_writeRegister(regAddress, regData);
}

/**
 * @brief Calculates and sets the Over temperature (OT) threshold register for a specific ANx input (TH_ANx_OT).
 */
bool SlaveIf_setOverTempThreshold(uint8_t anx_index, float overTemp_inVolt) {
	if (anx_index > 6) {
		PRINTF("Error: Invalid ANx index %d for OT threshold.\n\r\r", anx_index);
		return false;
	}
	// Datasheet addresses are descending: TH_ANO_OT_ADDR = 0x60, TH_AN1_OT_ADDR=0x5F, ..., TH_AN6_OT_ADDR=0x5A
	uint8_t regAddress = TH_ANO_OT_ADDR - anx_index;
	uint16_t regData = SlaveIf_CalculateOverTempThresholdReg(overTemp_inVolt);
	PRINTF("Setting AN%d OT Threshold (Reg 0x%02X) to: 0x%04X (Input V: %.3fV)\n\r\r", anx_index, regAddress, regData, overTemp_inVolt);
	return SlaveIF_writeRegister(regAddress, regData);
}

/**
 * @brief Calculates and sets the Under temperature (UT) threshold register for a specific ANx input (TH_ANx_UT).
 */
bool SlaveIf_setUnderTempThreshold(uint8_t anx_index, float underTemp_inVolt) {
	if (anx_index > 6) {
		PRINTF("Error: Invalid ANx index %d for UT threshold.\n\r\r", anx_index);
		return false;
	}
	// Datasheet addresses are descending: TH_ANO_UT_ADDR = 0x67, TH_AN1_UT_ADDR=0x66, ..., TH_AN6_UT_ADDR=0x61
	uint8_t regAddress = TH_AN0_UT_ADDR - anx_index;
	uint16_t regData = SlaveIf_CalculateUnderTempThresholdReg(underTemp_inVolt);
	PRINTF("Setting AN%d UT Threshold (Reg 0x%02X) to: 0x%04X (Input V: %.3fV)\n\r\r", anx_index, regAddress, regData, underTemp_inVolt);
	return SlaveIF_writeRegister(regAddress, regData);
}

/**
 * @brief Calculates and sets the Over current (OC) threshold register (TH_ISENSE_OC).
 */
bool SlaveIf_setOverCurrentThreshold(float overCurrent_amps, float shunt_resistance_micro_ohms) {
	uint16_t regData = SlaveIf_CalculateOverCurrentThresholdReg(overCurrent_amps, shunt_resistance_micro_ohms);
	PRINTF("Setting Over current Threshold (Reg 0x%02X) to: 0x%04X (Current: %.3fA, Shunt: %.1fuOhm)\n\r\r", TH_ISENSE_OC_ADDR, regData, overCurrent_amps, shunt_resistance_micro_ohms);
	return SlaveIF_writeRegister(TH_ISENSE_OC_ADDR, regData);
}
