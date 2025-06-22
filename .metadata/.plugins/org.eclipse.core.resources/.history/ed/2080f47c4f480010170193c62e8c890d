/**
 * @file        SlaveIF.c
 * @brief       Implementation of the Slave Interface driver for MC33771B communication.
 *
 * @details     This file contains the implementation for interacting with the MC33771B
 *              battery monitoring IC via the MC33664 TPL transceiver using SPI
 *              on an NXP KL25Z microcontroller. It handles low-level SPI/DMA
 *              communication, frame construction, register access, and provides
 *              APIs for configuration, measurement, fault monitoring, and cell balancing.
 *
 * @note        Project: Graduation Project - Battery Management System
 * @note        Engineer: Abdullah Mohamed
 * @note        Component: Slave_Control_IF driver
 */

//=============================================================================
// Includes
//=============================================================================
#include <COTs/SlaveControlIF/Inc/slaveIF.h>
#include <COTs/SlaveControlIF/Inc/slaveIF_Cfg.h>

//=============================================================================
// Constant Definitions
//=============================================================================
/**
 * @brief       TagID register list for BCC14 cut 2.
 * @details     Defines registers using TagID format for MC33771 (14 cells, cut 2).
 */
const uint16_t _TAGID_BCC14p2[8] = {0x0000, 0x0000, 0x0000, 0xFFFF, 0x07FF, 0x0000, 0x0000, 0x0000};

/**
 * @brief       TagID register list for BCC14.
 * @details     Defines registers using TagID format for MC33771 (14 cells).
 */
const uint16_t _TAGID_BCC14[8] = {0x0020, 0x0000, 0xE070, 0xFFFF, 0x07FF, 0x0000, 0x0000, 0x0000};

/**
 * @brief       TagID register list for BCC6.
 * @details     Defines registers using TagID format for MC33772 (6 cells).
 */
const uint16_t _TAGID_BCC6[8] = {0x0020, 0x0000, 0xE070, 0xF807, 0x07FF, 0x0000, 0x0000, 0x0000};

/**
 * @brief       Rolling Counter (RC) values list.
 * @details     Defines the sequence of RC values for frame verification.
 */
volatile const uint8_t RCVALUELIST[] = {0x0, 0x1, 0x3, 0x2};

//=============================================================================
// Configuration Arrays
//=============================================================================
/**
 * @brief       Configuration array for MC33771 via TPL.
 * @details     Defines register values for configuring MC33771 (14 cells) via TPL.
 * @note        Last entry must be {0, 0} to indicate end of list.
 * @warning      Adjust thresholds and masks based on your application requirements.
 */
const SsysConf_t CONF33771TPL[] = {
    // Over/Under Voltage Thresholds
    {TH_CT1, TH_OVUV_VALUE(3.5, 1.8)},
    {TH_CT2, TH_OVUV_VALUE(3.5, 1.8)},
    {TH_CT3, TH_OVUV_VALUE(3.5, 1.8)},
    {TH_CT4, TH_OVUV_VALUE(3.5, 1.8)},
    {TH_CT5, TH_OVUV_VALUE(3.5, 1.8)},
    {TH_CT6, TH_OVUV_VALUE(3.5, 1.8)},
    {TH_CT7, TH_OVUV_VALUE(3.5, 1.8)},
    {TH_CT8, TH_OVUV_VALUE(3.5, 1.8)},
    {TH_CT9, TH_OVUV_VALUE(3.5, 1.8)},
    {TH_CT10, TH_OVUV_VALUE(3.5, 1.8)},
    {TH_CT11, TH_OVUV_VALUE(3.5, 1.8)},
    {TH_CT12, TH_OVUV_VALUE(3.5, 1.8)},
    {TH_CT13, TH_OVUV_VALUE(3.5, 1.8)},
    {TH_CT14, TH_OVUV_VALUE(3.5, 1.8)},
    // Over/Under Temperature Thresholds
    {TH_AN0_OT, DEG30C},
    {TH_AN1_OT, DEG30C},
    {TH_AN2_OT, DEG30C},
    {TH_AN3_OT, DEG30C},
    {TH_AN4_OT, DEG30C},
    {TH_AN5_OT, DEG30C},
    {TH_AN6_OT, DEG30C},
    {TH_AN0_UT, DEG0C},
    {TH_AN1_UT, DEG0C},
    {TH_AN2_UT, DEG0C},
    {TH_AN3_UT, DEG0C},
    {TH_AN4_UT, DEG0C},
    {TH_AN5_UT, DEG0C},
    {TH_AN6_UT, DEG0C},
    // Fault Handling Masks
    {FAULT_MASK1, 0x1FF0}, // CT OT/UT/OV/UV enabled
    {FAULT_MASK2, 0xFE7F}, // All masked
    {FAULT_MASK3, 0xFFFF}, // All masked
    // Wakeup Sources Masks
    {WAKEUP_MASK1, 0x199F}, // All masked
    {WAKEUP_MASK2, 0xFF36}, // All masked
    {WAKEUP_MASK3, 0xBFFF}, // All masked
    // Configuration Settings
    {OV_UV_EN, 0x3FFF},         // Enable OV & UV handling
    {SYS_CFG1, 0x9280},         // System configuration 1
    {SYS_CFG2, 0x6330},         // System configuration 2
    {FAULT1_STATUS, 0xC000},    // Clear all bits except POR, Reset
    {FAULT2_STATUS, 0},         // Clear all bits
    {FAULT3_STATUS, 0},         // Clear all bits
    {ADC_CFG, ADC_CFG_SETTING}, // Set ADC configuration
    {0, 0}                      // End symbol
};

//=============================================================================
// Global Variables
//=============================================================================
/**
 * @brief       Index to access RCVALUELIST.
 */
uint8_t gu8RCIdx = 0;

/**
 * @brief       Local storage of TagID for each CID.
 */
uint8_t gTagID[15];

/**
 * @brief       Local structure storing the interface (SPI or TPL).
 */
static TYPE_INTERFACE *P_interface_;

/**
 * @brief       Local structure storing the cluster information (e.g., enabled TagID, TagIDLIST).
 */
static SclusterInfo_t *_cluster_;

/**
 * @brief       Storage of last error for error handling.
 */
static TypeReturn_t _errLast;

//=============================================================================
// Static Function Prototypes
//=============================================================================
/**
 * @brief       Checks if a register uses TagID format.
 * @param regAdr Register address.
 * @param tagIDList Pointer to TagID list.
 * @return bool True if register uses TagID format, false if RC format.
 */
static bool bRegIsTagID(uint8_t regAdr, const uint16_t *tagIDList);

/**
 * @brief       Packs a 40-bit message frame into a 5-byte buffer.
 * @details     Packs data, address, CID, command, and CRC for SPI/TPL transmission.
 * @param 	pu8Buf Pointer to 5-byte buffer for packed frame.
 * @param 	data 16-bit data to pack.
 * @param 	addr 7-bit register address.
 * @param 	CID 4-bit Cluster ID.
 * @param 	cmd 4-bit command.
 */
static void slaveIF_packFrame(uint8_t *pu8Buf, uint16_t data, uint8_t addr, uint8_t CID, uint8_t cmd);

/**
 * @brief       Calculates the CRC8 for a data array.
 * @details     Uses a lookup table with polynomial 0x2F to calculate CRC8.
 * @param 	    data Pointer to data byte array.
 * @param 	    length Number of bytes to process.
 * @return 	    uint8_t Calculated CRC8 value.
 */
static uint8_t slaveIF_crcCalc(uint8_t *data, uint16_t length);

/**
 * @brief       Sets the TPL enable signal.
 * @details     Enables or disables the TPL signal for Arduino-type EVB.
 * @param 	    bEnable 0 to disable, non-zero to enable.
 */
static void slaveIF_TplEnable(uint8_t bEnable);
//=============================================================================
// Static Function Definitions
//=============================================================================
/**
 * @brief       Checks if a register uses TagID format.
 * @details     Determines if the specified register address uses TagID or RC format based on the TagID list.
 * @param       regAdr Register address (0..0x7F).
 * @param       tagIDList Pointer to TagID list.
 * @return      bool True if register uses TagID format, false if RC format.
 */
static bool bRegIsTagID(uint8_t regAdr, const uint16_t *tagIDList)
{
    regAdr &= 0x7F; // Clear response bit (Master/Slave bit)
    return (tagIDList[(regAdr >> 4) & 0x07] & BIT(regAdr & 0x0F)) != 0;
}

/**
 * @brief       Packs a 40-bit message frame into a 5-byte buffer.
 * @details     Packs data, address, CID, command, and CRC for SPI/TPL transmission.
 * \code
 * ------------------------------------------------------------------------------
 * |  Byte 4  |  Byte 3  |  Byte 2            |       Byte 1         |  Byte 0  |
 * |----------|----------|--------------------|----------------------|----------|
 * | bit39:31 | bit30:24 | bit23   | bit22:16 | bit15:12   | bit11:8 | bit7:0   |
 * |     memory data     | response| Register | cluster id | command |          |
 * |        data         | bResp   |   Addr   |   CID      | cmd     |  CRC     |
 * ------------------------------------------------------------------------------
 * \endcode
 * @param       pu8Buf Pointer to 5-byte buffer for packed frame.
 * @param       data 16-bit data to pack.
 * @param       addr 7-bit register address.
 * @param       CID 4-bit Cluster ID.
 * @param       cmd 4-bit command.
 */
static void slaveIF_packFrame(uint8_t *pu8Buf, uint16_t data, uint8_t addr, uint8_t CID, uint8_t cmd)
{
    *(pu8Buf + 4) = (uint8_t)(data >> 8);
    *(pu8Buf + 3) = (uint8_t)(data);
    *(pu8Buf + 2) = (uint8_t)(addr & 0x7F);
    *(pu8Buf + 1) = (uint8_t)((CID & 0x0F) << 4) | (cmd & 0x0F);
    *(pu8Buf + 0) = slaveIF_crcCalc(pu8Buf + 1, 4);
#ifdef SLAVEIF_DEBUG_COMM
    PRINTF("SlaveIF: Packed frame - CID: %d, Addr: 0x%02X, Cmd: %d, Data: 0x%04X, CRC: 0x%02X\n",
           CID, addr, cmd, data, *(pu8Buf + 0));
#endif
}

/**
 * @brief       Calculates CRC8 for a data array.
 * @details     Uses a lookup table with polynomial 0x2F to calculate CRC8.
 * @param       data Pointer to data byte array.
 * @param       length Number of bytes to process.
 * @return      uint8_t Calculated CRC8 value.
 */
static uint8_t slaveIF_crcCalc(uint8_t *data, uint16_t length)
{
    uint8_t crc = 0x42; // Seed value
    while (length--)
    {
        uint8_t tbl_idx = crc ^ *(data + length);
        crc = crc_table[tbl_idx];
    }
#ifdef SLAVEIF_DEBUG_COMM
    PRINTF("SlaveIF: CRC calculated: 0x%02X\n", crc);
#endif
    return crc;
}

/**
 * @brief       Sets the TPL enable signal.
 * @details     Enables or disables the TPL signal for Arduino-type EVB.
 * @param       bEnable 0 to disable, non-zero to enable.
 */
static void slaveIF_TplEnable(uint8_t bEnable)
{
    if ((_interface == IntTPL) && (_evb == EVB_TypeArd))
    { // EVBs via TPL
        if (bEnable == 0)
            GPIOE_PCOR = BIT(0);
        else
            GPIOE_PSOR = BIT(0);
    }
#ifdef SLAVEIF_DEBUG_COMM
    PRINTF("SlaveIF: TPL enable set to %d\n", bEnable);
#endif
}
//=============================================================================
// Public Function Definitions - Error Handling
//=============================================================================
/**
 * @brief       Sets an internal error code.
 * @details     Allows simulation of errors for testing.
 * @param       res Error code to set.
 * @return      bool Always false for easy error handling.
 */
bool slaveIF_setError(TypeReturn_t res)
{
    if (res == RETURN_OK)
        res = ERR_WrongParam;
    _errLast = res;
    return false;
}

/**
 * @brief       Retrieves the last error code.
 * @details     Returns the current error status.
 * @param       errorCode Pointer to store error code (or NULL).
 * @return      bool True if error exists, false otherwise.
 */
bool slaveIF_getError(TypeReturn_t *errorCode)
{
    if (NULL != errorCode)
        *errorCode = _errLast;
    return (_errLast != RETURN_OK);
}

/**
 * @brief       Clears the internal error status.
 * @details     Resets the error status to RETURN_OK.
 */
void slaveIF_clearError(void)
{
    _errLast = RETURN_OK;
}

//=============================================================================
// Public Function Definitions - Initialization
//=============================================================================
/**
 * @brief       Initializes the SlaveIF driver.
 * @details     Clears errors and stores interface information.
 * @param       interface Pointer to interface type.
 * @return      bool True if successful.
 */
bool slaveIF_initDriver(TYPE_INTERFACE *interface)
{
    _errLast = RETURN_OK;
    P_interface_ = interface;
#ifdef SLAVEIF_DEBUG_INIT
    PRINTF("SlaveIF: Driver initialized with interface %d\n", *interface);
#endif
    return true;
}

/**
 * @brief       Initializes the cluster structure.
 * @details     Stores cluster information for use in other functions.
 * @param       cluster Pointer to cluster data.
 * @return      bool True if successful.
 */
bool slaveIF_initCluster(SclusterInfo_t *cluster)
{
    _cluster_ = cluster;
#ifdef SLAVEIF_DEBUG_INIT
    PRINTF("SlaveIF: Cluster initialized with chip type %d\n", cluster->Chip);
#endif
    return true;
}

//=============================================================================
// Public Function Definitions - Status Monitoring
//=============================================================================
/**
 * @brief       Reads the status of the Fault pin.
 * @details     Returns the Fault pin status for TPL interface on Arduino-type EVB.
 * @return      uint8_t 1 if Fault, 0 if no Fault.
 */
uint8_t slaveIF_faultPinStatus(void)
{
    if ((_interface == IntTPL) && (_evb == EVB_TypeArd))
    {                                                   // Arduino type connection
        uint8_t status = (GPIOD_PDIR & BIT(4)) ? 1 : 0; // PTD4
#ifdef SLAVEIF_DEBUG_FAULT
        PRINTF("SlaveIF: Fault pin status: %d\n", status);
#endif
        return status;
    }
    return 1;
}

/**
 * @brief       Reads the status of the INTB pin.
 * @details     Returns the INTB pin status for TPL interface on Arduino-type EVB.
 * @return      uint8_t 1 if INTB is high, 0 if low.
 */
uint8_t slaveIF_IntbPinStatus(void)
{
    if ((_interface == IntTPL) && (_evb == EVB_TypeArd))
    {
        uint8_t status = (GPIOA_PDIR & BIT(12)) ? 1 : 0; // PTA12
#ifdef SLAVEIF_DEBUG_FAULT
        PRINTF("SlaveIF: INTB pin status: %d\n", status);
#endif
        return status;
    }
    return 0;
}

//=============================================================================
// Public Function Definitions - Communication Setup
//=============================================================================
/**
 * @brief       Sets the CSB level for SPI/TPL communication.
 * @details     Sets the CSB pin to high (non-zero) or low (zero) for Arduino-type EVB.
 * @param       u8Level 0 for low, non-zero for high.
 */
void slaveIF_SPICS(uint8_t u8Level)
{
    if ((_interface == IntSPI) && (_evb == EVB_TypeArd))
    {
        if (u8Level == 0)
            GPIOC_PCOR = BIT(8); // clear PTC8
        else
            GPIOC_PSOR = BIT(8); // set PTC8
    }
    if ((_interface == IntTPL) && (_evb == EVB_TypeArd))
    {
        if (u8Level == 0)
            GPIOC_PCOR = BIT(8); // clear PTC8
        else
            GPIOC_PSOR = BIT(8); // set PTC8
    }
#ifdef SLAVEIF_DEBUG_COMM
    PRINTF("SlaveIF: CSB set to %d\n", u8Level);
#endif
}

/**
 * @brief       Enables TPL interface.
 * @details     Sets TPL enable signal and waits for INTB acknowledge.
 * @return      bool True if successful, false on error.
 */
bool slaveIF_transceiverEnable(void)
{
    uint32_t u32Cnt;

    if (slaveIF_getError(NULL))
        return false; // only execute if no errors are pending

    slaveIF_TplEnable(0); // disable
    Delay(DELAY_150us);
    // start timeout >>110us
    slaveIF_TplEnable(1); // enable

    u32Cnt = 0x10000;
    while (slaveIF_IntbPinStatus() == 1)
    { // wait for a low
        if (u32Cnt-- == 0)
            return slaveIF_setError(ERR_Timeout);
    }

    u32Cnt = 0x10000;
    while (slaveIF_IntbPinStatus() == 0)
    { // wait for a high
        if (u32Cnt-- == 0)
            return slaveIF_setError(ERR_Timeout);
    }

#ifdef SLAVEIF_DEBUG_COMM
    PRINTF("SlaveIF: TPL enabled successfully\n");
#endif
    return true;
}

/**
 * @brief       Disables TPL interface.
 * @details     Clears the TPL enable signal.
 * @return      bool True if successful, false on error.
 */
bool slaveIF_transceiverDisable(void)
{
    if (slaveIF_getError(NULL))
        return false; // only execute if no errors are pending

    slaveIF_TplEnable(0); // disable
#ifdef SLAVEIF_DEBUG_COMM
    PRINTF("SlaveIF: TPL disabled\n");
#endif
    return true;
}

//=============================================================================
// Public Function Definitions - Communication Utilities
//=============================================================================
/**
 * @brief       Generates a new Rolling Counter (RC) value.
 * @details     Returns the next RC value from RCVALUELIST.
 * @return      uint8_t New RC value.
 */
uint8_t slaveIF_generateNewRC(void)
{
    gu8RCIdx = (gu8RCIdx + 1) % sizeof(RCVALUELIST);
    uint8_t rc = RCVALUELIST[gu8RCIdx];
#ifdef SLAVEIF_DEBUG_COMM
    PRINTF("SlaveIF: New RC value: %d\n", rc);
#endif
    return rc;
}

/**
 * @brief       Sets the TagID for a given CID.
 * @details     Assigns a new TagID to the specified CID.
 * @param       CID Cluster ID (1..14).
 * @param       NewTagId New TagID value (0..15).
 * @return      bool True if successful, false on error.
 */
bool slaveIF_setTagID(uint8_t CID, uint8_t NewTagId)
{
    if (NewTagId > 15 || CID > 14 || CID == 0)
        return slaveIF_setError(ERR_WrongParam);
    gTagID[CID] = NewTagId;
#ifdef SLAVEIF_DEBUG_CONFIG
    PRINTF("SlaveIF: Set TagID %d for CID %d\n", NewTagId, CID);
#endif
    return true;
}

//=============================================================================
// Public Function Definitions - Device Control
//=============================================================================
/**
 * @brief       Wakes up the MC33771B device.
 * @details     Issues a wake-up sequence via CSB toggle and waits for response.
 * @return      bool True if successful, false on error.
 */
bool slaveIF_wakeUp(void)
{
    if (slaveIF_getError(NULL))
        return false; // only execute if no errors are pending
    if (*P_interface_ == IntTPL)
    {
        slaveIF_SPICS(0);
        Delay(DELAY_22us);
        slaveIF_SPICS(1);
        Delay(DELAY_600us);
        slaveIF_SPICS(0);
        Delay(DELAY_22us);
        slaveIF_SPICS(1);
        Delayms(WAIT_AFTER_WAKEUP);
#ifdef SLAVEIF_DEBUG_INIT
        PRINTF("SlaveIF: TPL wake-up sequence completed\n");
#endif
        return true;
    }
    else
        return slaveIF_setError(ERR_WrongInterface);
}

//=============================================================================
// Public Function Definitions - Register Access
//=============================================================================
/**
 * @brief       Reads registers from MC33771B.
 * @details     Reads up to 50 registers in a burst via SPI/TPL.
 * @param       CID Cluster ID (0..15).
 * @param       Register Register address (0..0x7F).
 * @param       noRegs2Read Number of registers to read (1..50).
 * @param       readData Pointer to array for read data (or NULL).
 * @return      bool True if successful, false on error.
 */
bool slaveIF_readReg(uint8_t CID, uint8_t Register, uint8_t noRegs2Read, uint16_t *readData)
{
    uint8_t txFrame[MSGLEN];
    uint8_t rxFrame[MSGLEN];
    uint8_t idx = 0;
    uint8_t RC;
    uint8_t u8RxBytes;

    if (slaveIF_getError(NULL))
        return false;
    if ((noRegs2Read == 0) || (noRegs2Read > 50))
        return slaveIF_setError(ERR_WrongParam);

    if (*P_interface_ == IntTPL)
    {
        RC = slaveIF_generateNewRC();                                                   // get next RC value from list
        slaveIF_packFrame(txFrame, noRegs2Read, Register, CID, (RC << 2) | CmdRdLocal); // pack data (read noRegs2Read registers)
        SPIRxFlushBuffer();                                                             // flush receiver
        SPITxSendBuffer(txFrame, MSGLEN);                                               // transmit read request

        if (MSGLEN != SPIRxReadBytes(rxFrame, MSGLEN)) // read request "echo" and check
            return slaveIF_setError(ERR_TX);
        if (memcmp(txFrame, rxFrame, MSGLEN))
            return slaveIF_setError(ERR_TX);

        while (noRegs2Read-- > 0)
        {
            u8RxBytes = SPIRxReadBytes(rxFrame, MSGLEN); // read response
            if (u8RxBytes == 0)
                return slaveIF_setError(ERR_NoResponse);
            if (u8RxBytes != MSGLEN)
                return slaveIF_setError(ERR_ResponseLen);
            if (slaveIF_crcCalc(rxFrame, MSGLEN) != 0)
                return slaveIF_setError(ERR_ResponseCRC);

            if (UNPACK_REGADR(rxFrame) != Register) // check response address
                return slaveIF_setError(ERR_ResponseAdr);
            if (CID > 0)
            { // no checks for unassigned CIDs reads
                if (_cluster_[CID - 1].pTagIdList != NULL)
                { // use of TagIDs "enabled"?
                    if (bRegIsTagID(Register, _cluster_[CID - 1].pTagIdList))
                    {
                        if (UNPACK_TAGID(rxFrame) != gTagID[CID])
                        { // check TagID
                            return slaveIF_setError(ERR_ResponseTagId);
                        }
                    }
                    else
                    {
                        if (UNPACK_RC(rxFrame) != RC)
                        { // check RC
                            return slaveIF_setError(ERR_ResponseRC);
                        }
                    }
                }
            }
            if (readData != NULL)
            {
                readData[idx] = UNPACK_DATA(rxFrame); // read data
#ifdef SLAVEIF_DEBUG_COMM
                PRINTF("SlaveIF: Read register 0x%02X, Data: 0x%04X\n", Register, readData ? readData[idx] : 0);
#endif
            }
            idx++;
            Register++;
        }
        return true;
    }
    else
        return slaveIF_setError(ERR_WrongInterface);
}

/**
 * @brief       Writes a register to MC33771B.
 * @details     Writes data to a specified register via SPI/TPL.
 * @param       CID Cluster ID (0..15).
 * @param       Register Register address (0..0x7F).
 * @param       writeData Data to write.
 * @param       returnData Pointer to returned data (or NULL).
 * @return      bool True if successful, false on error.
 */
bool slaveIF_writeReg(uint8_t CID, uint8_t Register, uint16_t writeData, uint16_t *returnData)
{
    uint8_t txFrame[MSGLEN];
    uint8_t rxFrame[MSGLEN];
    uint8_t u8RxBytes;
    uint8_t RC;

    if (slaveIF_getError(NULL))
        return false; // only execute if no errors are pending

    if (*P_interface_ == IntTPL)
    {
        RC = slaveIF_generateNewRC();                                                 // get next RC value from list
        slaveIF_packFrame(txFrame, writeData, Register, CID, (RC << 2) | CmdWrLocal); // pack data
        SPIRxFlushBuffer();                                                           // flush receiver
        SPITxSendBuffer(txFrame, MSGLEN);                                             // transmit

        if (MSGLEN != SPIRxReadBytes(rxFrame, MSGLEN)) // read request "echo" and check
            return slaveIF_setError(ERR_TX);
        if (memcmp(txFrame, rxFrame, MSGLEN))
            return slaveIF_setError(ERR_TX);

        u8RxBytes = SPIRxReadBytes(rxFrame, MSGLEN); // read response
        if (u8RxBytes == 0)
            return slaveIF_setError(ERR_NoResponse);
        if (u8RxBytes != MSGLEN)
            return slaveIF_setError(ERR_ResponseLen);
        if (slaveIF_crcCalc(rxFrame, MSGLEN) != 0)
            return slaveIF_setError(ERR_ResponseCRC);
        if (UNPACK_RC(rxFrame) != RC)
            return slaveIF_setError(ERR_ResponseRC);
        if (returnData != NULL)
        {
            *returnData = UNPACK_DATA(rxFrame); // return data
        }

        return RETURN_OK;
    }
    else
        return slaveIF_setError(ERR_WrongInterface);
}

/**
 * @brief       Writes a global register to all CIDs.
 * @details     Writes data to a register for all clusters via TPL.
 * @param       Register Register address (0..0x7F).
 * @param       writeData Data to write.
 * @return      bool True if successful, false on error.
 */
bool slaveIF_writeGlobalReg(uint8_t Register, uint16_t writeData)
{
    uint8_t txFrame[MSGLEN];
    uint8_t rxFrame[MSGLEN];

    if (slaveIF_getError(NULL))
        return false; // only execute if no errors are pending

    if (*P_interface_ == IntTPL)
    {
        slaveIF_packFrame(txFrame, writeData, Register, GLOBAL_CID, CmdWrGlobal); // 1. pack data

        SPIRxFlushBuffer();
        SPITxSendBuffer(txFrame, MSGLEN); // 2. transmit

        if (MSGLEN != SPIRxReadBytes(rxFrame, MSGLEN)) // 3. read back transmit and check
            return slaveIF_setError(ERR_TX);
        if (memcmp(txFrame, rxFrame, MSGLEN))
            return slaveIF_setError(ERR_TX);

        return true;
    }
    else
        return slaveIF_setError(ERR_WrongInterface);
}

/**
 * @brief      Enables or disables cell balancing for a specified cell.
 * @details    Configures cell balancing for a given cell (1-14) by setting the
 *             register address, enabling/disabling (bit 9), and setting a timer
 *             in half-minute increments (up to 511 minutes) via I2C write.
 * @param      cellNumber The cell number (1-14) to enable balancing for.
 * @param      enable Boolean flag to enable (true) or disable (false) balancing.
 * @param      timerValueInMinutes Float value for balancing duration (capped at 511 min).
 * @param      cid Chip ID of the slave device to write to.
 * @return     None (void).
 */
void SlaveIF_enableCellBalancing(uint8_t cellNumber, bool enable, float timerValueInMinutes, uint8_t cid)
{
    uint8_t regAddress;
    switch (cellNumber) 
    {
    case 1:  regAddress = CB1_CFG;  break;
    case 2:  regAddress = CB2_CFG;  break;
    case 3:  regAddress = CB3_CFG;  break;
    case 4:  regAddress = CB4_CFG;  break;
    case 5:  regAddress = CB5_CFG;  break;
    case 6:  regAddress = CB6_CFG;  break;
    case 7:  regAddress = CB7_CFG;  break;
    case 8:  regAddress = CB8_CFG;  break;
    case 9:  regAddress = CB9_CFG;  break;
    case 10: regAddress = CB10_CFG; break;
    case 11: regAddress = CB11_CFG; break;
    case 12: regAddress = CB12_CFG; break;
    case 13: regAddress = CB13_CFG; break;
    case 14: regAddress = CB14_CFG; break;
    default: return; 
#ifdef SLAVEIF_DEBUG_ERROR
             PRINTF("SlaveIF: Invalid cell number %d\n", cellNumber);
#endif
    }

    uint16_t data = 0;
    if (enable)
    {
        SET_BIT(data, 9); // Enable CB_EN (bit 9)
#ifdef SLAVEIF_DEBUG_CONFIG
        PRINTF("SlaveIF: Enabling cell balancing for cell %d\n", cellNumber);
#endif
    }

    uint16_t timerValueInHalfMinutes = (uint16_t)(timerValueInMinutes / 0.5);
    if (timerValueInHalfMinutes > 0x1FF)
    {
        timerValueInHalfMinutes = 0x1FF; // Cap at max value
#ifdef SLAVEIF_DEBUG_CONFIG
        PRINTF("SlaveIF: Timer value capped to 511 minutes for cell %d\n", cellNumber);
#endif
    }
    data |= (timerValueInHalfMinutes & 0x1FF); // Set timer bits

    slaveIF_writeReg(cid, regAddress, data, NULL);
#ifdef SLAVEIF_DEBUG_CONFIG
    PRINTF("SlaveIF: Wrote data 0x%04x to reg 0x%02x for cell %d (CID: %d)\n",
           data, regAddress, cellNumber, cid);
#endif
}
//=============================================================================
// End of File
//=============================================================================
