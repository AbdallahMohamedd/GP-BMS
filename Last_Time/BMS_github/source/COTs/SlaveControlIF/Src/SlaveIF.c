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
#include "source/COTs/SlaveControlIF/Inc/SlaveIF.h"
#include "source/COTs/SlaveControlIF/Inc/SlaveIF_Cfg.h"

//=============================================================================
// Constant Definitions
//=============================================================================
/**
 * @brief TagID register list for BCC14 cut 2.
 * @details Defines registers using TagID format for MC33771 (14 cells, cut 2).
 */
const uint16_t _TAGID_BCC14p2[8] = {0x0000, 0x0000, 0x0000, 0xFFFF, 0x07FF, 0x0000, 0x0000, 0x0000};

/**
 * @brief TagID register list for BCC14.
 * @details Defines registers using TagID format for MC33771 (14 cells).
 */
const uint16_t _TAGID_BCC14[8] = {0x0020, 0x0000, 0xE070, 0xFFFF, 0x07FF, 0x0000, 0x0000, 0x0000};

/**
 * @brief TagID register list for BCC6.
 * @details Defines registers using TagID format for MC33772 (6 cells).
 */
const uint16_t _TAGID_BCC6[8] = {0x0020, 0x0000, 0xE070, 0xF807, 0x07FF, 0x0000, 0x0000, 0x0000};

/**
 * @brief Rolling Counter (RC) values list.
 * @details Defines the sequence of RC values for frame verification.
 */
volatile const uint8_t RCVALUELIST[] = {0x0, 0x1, 0x3, 0x2};

//=============================================================================
// Configuration Arrays
//=============================================================================
/**
 * @brief Configuration array for MC33771 via TPL.
 * @details Defines register values for configuring MC33771 (14 cells) via TPL.
 * @note Last entry must be {0, 0} to indicate end of list.
 * @warning Adjust thresholds and masks based on your application requirements.
 */
const TYPE_BCC_CONF CONF33771TPL[] = {
    // Over/Under Voltage Thresholds
    {TH_CT1, TH_OVUV_VALUE(3.0, 1.8)},
    {TH_CT2, TH_OVUV_VALUE(3.0, 1.8)},
    {TH_CT3, TH_OVUV_VALUE(3.0, 1.8)},
    {TH_CT4, TH_OVUV_VALUE(3.0, 1.8)},
    {TH_CT5, TH_OVUV_VALUE(3.0, 1.8)},
    {TH_CT6, TH_OVUV_VALUE(3.0, 1.8)},
    {TH_CT7, TH_OVUV_VALUE(3.0, 1.8)},
    {TH_CT8, TH_OVUV_VALUE(3.0, 1.8)},
    {TH_CT9, TH_OVUV_VALUE(3.0, 1.8)},
    {TH_CT10, TH_OVUV_VALUE(3.0, 1.8)},
    {TH_CT11, TH_OVUV_VALUE(3.0, 1.8)},
    {TH_CT12, TH_OVUV_VALUE(3.0, 1.8)},
    {TH_CT13, TH_OVUV_VALUE(3.0, 1.8)},
    {TH_CT14, TH_OVUV_VALUE(3.0, 1.8)},
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
    {SYS_CFG1, 0x9000},         // System configuration 1
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
 * @brief Index to access RCVALUELIST.
 */
uint8_t gu8RCIdx = 0;

/**
 * @brief Local storage of TagID for each CID.
 */
uint8_t gTagID[15];

/**
 * @brief Local structure storing the interface (SPI or TPL).
 */
static TYPE_INTERFACE *P_interface_;

/**
 * @brief Local structure storing the cluster information (e.g., enabled TagID, TagIDLIST).
 */
static LLD_TYPE_CLUSTER *_cluster_;

/**
 * @brief Storage of last error for error handling.
 */
static LLD_TYPE_RETURN _errLast;

//=============================================================================
// Static Function Prototypes
//=============================================================================
/**
 * @brief Checks if a register uses TagID format.
 * @param regAdr Register address.
 * @param tagIDList Pointer to TagID list.
 * @return bool True if register uses TagID format, false if RC format.
 */
static bool bRegIsTagID(uint8_t regAdr, const uint16_t *tagIDList);

//=============================================================================
// Static Function Definitions
//=============================================================================
/**
 * @brief Checks if a register uses TagID format.
 * @details Determines if the specified register address uses TagID or RC format based on the TagID list.
 * @param regAdr Register address (0..0x7F).
 * @param tagIDList Pointer to TagID list.
 * @return bool True if register uses TagID format, false if RC format.
 */
static bool bRegIsTagID(uint8_t regAdr, const uint16_t *tagIDList)
{
    regAdr &= 0x7F; // Clear response bit (Master/Slave bit)
    return (tagIDList[(regAdr >> 4) & 0x07] & BIT(regAdr & 0x0F)) != 0;
}

//=============================================================================
// Public Function Definitions - Error Handling
//=============================================================================
/**
 * @brief Sets an internal error code.
 * @details Allows simulation of errors for testing.
 * @param res Error code to set.
 * @return bool Always false for easy error handling.
 */
bool _lld3377xSetError(LLD_TYPE_RETURN res)
{
    if (res == RETURN_OK)
        res = ERR_WrongParam;
    _errLast = res;
    return false;
}

/**
 * @brief Retrieves the last error code.
 * @details Returns the current error status.
 * @param errorCode Pointer to store error code (or NULL).
 * @return bool True if error exists, false otherwise.
 */
bool lld3377xGetError(LLD_TYPE_RETURN *errorCode)
{
    if (NULL != errorCode)
        *errorCode = _errLast;
    return (_errLast != RETURN_OK);
}

/**
 * @brief Clears the internal error status.
 * @details Resets the error status to RETURN_OK.
 */
void lld3377xClearError(void)
{
    _errLast = RETURN_OK;
}

//=============================================================================
// Public Function Definitions - Initialization
//=============================================================================
/**
 * @brief Initializes the SlaveIF driver.
 * @details Clears errors and stores interface information.
 * @param interface Pointer to interface type.
 * @return bool True if successful.
 */
bool lld3377xInitDriver(TYPE_INTERFACE *interface)
{
    _errLast = RETURN_OK;
    P_interface_ = interface;
#ifdef SLAVEIF_DEBUG_INIT
    printf("SlaveIF: Driver initialized with interface %d\n", *interface);
#endif
    return true;
}

/**
 * @brief Initializes the cluster structure.
 * @details Stores cluster information for use in other functions.
 * @param cluster Pointer to cluster data.
 * @return bool True if successful.
 */
bool lld3377xInitCluster(LLD_TYPE_CLUSTER *cluster)
{
    _cluster_ = cluster;
#ifdef SLAVEIF_DEBUG_INIT
    printf("SlaveIF: Cluster initialized with chip type %d\n", cluster->Chip);
#endif
    return true;
}

//=============================================================================
// Public Function Definitions - Status Monitoring
//=============================================================================
/**
 * @brief Reads the status of the Fault pin.
 * @details Returns the Fault pin status for TPL interface on Arduino-type EVB.
 * @return uint8_t 1 if Fault, 0 if no Fault.
 */
uint8_t FaultPinStatus(void)
{
    if ((_interface == IntTPL) && (_evb == EVB_TypeArd))
    {                                                   // Arduino type connection
        uint8_t status = (GPIOD_PDIR & BIT(4)) ? 1 : 0; // PTD4
#ifdef SLAVEIF_DEBUG_FAULT
        printf("SlaveIF: Fault pin status: %d\n", status);
#endif
        return status;
    }
    return 1;
}

/**
 * @brief Reads the status of the INTB pin.
 * @details Returns the INTB pin status for TPL interface on Arduino-type EVB.
 * @return uint8_t 1 if INTB is high, 0 if low.
 */
uint8_t IntbPinStatus(void)
{
    if ((_interface == IntTPL) && (_evb == EVB_TypeArd))
    {
        uint8_t status = (GPIOA_PDIR & BIT(12)) ? 1 : 0; // PTA12
#ifdef SLAVEIF_DEBUG_FAULT
        printf("SlaveIF: INTB pin status: %d\n", status);
#endif
        return status;
    }
    return 0;
}

//=============================================================================
// Public Function Definitions - Communication Setup
//=============================================================================
/**
 * @brief Sets the CSB level for SPI/TPL communication.
 * @details Sets the CSB pin to high (non-zero) or low (zero) for Arduino-type EVB.
 * @param u8Level 0 for low, non-zero for high.
 */
void slaveIF_SPISC(uint8_t u8Level)
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
    printf("SlaveIF: CSB set to %d\n", u8Level);
#endif
}

/**
 * @brief Sets the TPL enable signal.
 * @details Enables or disables the TPL signal for Arduino-type EVB.
 * @param bEnable 0 to disable, non-zero to enable.
 */
void TplEnable(uint8_t bEnable)
{
    if ((_interface == IntTPL) && (_evb == EVB_TypeArd))
    { // EVBs via TPL
        if (bEnable == 0)
            GPIOE_PCOR = BIT(0);
        else
            GPIOE_PSOR = BIT(0);
    }
#ifdef SLAVEIF_DEBUG_COMM
    printf("SlaveIF: TPL enable set to %d\n", bEnable);
#endif
}

/**
 * @brief Enables TPL interface.
 * @details Sets TPL enable signal and waits for INTB acknowledge.
 * @return bool True if successful, false on error.
 */
bool lld3377xTPLEnable(void)
{
    uint32_t u32Cnt;

    if (lld3377xGetError(NULL))
        return false; // only execute if no errors are pending

    TplEnable(0); // disable
    Delay(DELAY_150us);
    // start timeout >>110us
    TplEnable(1); // enable

    u32Cnt = 0x10000;
    while (IntbPinStatus() == 1)
    { // wait for a low
        if (u32Cnt-- == 0)
            return _lld3377xSetError(ERR_Timeout);
    }

    u32Cnt = 0x10000;
    while (IntbPinStatus() == 0)
    { // wait for a high
        if (u32Cnt-- == 0)
            return _lld3377xSetError(ERR_Timeout);
    }

#ifdef SLAVEIF_DEBUG_COMM
    printf("SlaveIF: TPL enabled successfully\n");
#endif
    return true;
}

/**
 * @brief Disables TPL interface.
 * @details Clears the TPL enable signal.
 * @return bool True if successful, false on error.
 */
bool lld3377xTPLDisable(void)
{
    if (lld3377xGetError(NULL))
        return false; // only execute if no errors are pending

    TplEnable(0); // disable
#ifdef SLAVEIF_DEBUG_COMM
    printf("SlaveIF: TPL disabled\n");
#endif
    return true;
}

//=============================================================================
// Public Function Definitions - Communication Utilities
//=============================================================================
/**
 * @brief Calculates CRC8 for a data array.
 * @details Uses a lookup table with polynomial 0x2F to calculate CRC8.
 * @param data Pointer to data byte array.
 * @param length Number of bytes to process.
 * @return uint8_t Calculated CRC8 value.
 */
uint8_t lld3377xCrcCalc(uint8_t *data, uint16_t length)
{
    uint8_t crc = 0x42; // Seed value
    while (length--)
    {
        uint8_t tbl_idx = crc ^ *(data + length);
        crc = crc_table[tbl_idx];
    }
#ifdef SLAVEIF_DEBUG_COMM
    printf("SlaveIF: CRC calculated: 0x%02X\n", crc);
#endif
    return crc;
}

/**
 * @brief Tests CRC patterns from MC33771 datasheet.
 * @details Verifies CRC8 calculation with predefined patterns.
 * @return uint32_t 0 if all tests pass, else bitmask of failed tests.
 */
uint32_t crc8_test(void)
{
    uint32_t u32Result = 0;
    uint8_t u8Pat1[] = {0x10, 0x08, 0x01, 0x01};
    uint8_t u8Pat2[] = {0xA1, 0x01, 0x0A, 0x0A};
    uint8_t u8Pat3[] = {0x22, 0x0F, 0xC4, 0x01};
    uint8_t u8Pat4[] = {0x53, 0x01, 0x57, 0x72};
    uint8_t u8Pat5[] = {0x00, 0x00, 0x00, 0x00};
    uint8_t u8Pat6[] = {0xBD, 0x10, 0x09, 0x01, 0x11};
    uint8_t u8Pat7[] = {0x66, 0x50, 0x09, 0x02, 0x20};
    uint8_t u8Pat8[] = {0xFB, 0xA5, 0x09, 0x03, 0x51};
    uint8_t u8Pat9[] = {0xC0, 0x62, 0x09, 0x04, 0xFF};
    uint8_t u8Pat10[] = {0xB2, 0x00, 0x00, 0x00, 0x00};

    if (lld3377xCrcCalc(u8Pat1, 4) != 0x22)
        u32Result |= BIT(1);
    if (lld3377xCrcCalc(u8Pat2, 4) != 0xF6)
        u32Result |= BIT(2);
    if (lld3377xCrcCalc(u8Pat3, 4) != 0x6A)
        u32Result |= BIT(3);
    if (lld3377xCrcCalc(u8Pat4, 4) != 0x71)
        u32Result |= BIT(4);
    if (lld3377xCrcCalc(u8Pat5, 4) != 0xB2)
        u32Result |= BIT(5);
    if (lld3377xCrcCalc(u8Pat6, 5) != 0)
        u32Result |= BIT(6);
    if (lld3377xCrcCalc(u8Pat7, 5) != 0)
        u32Result |= BIT(7);
    if (lld3377xCrcCalc(u8Pat8, 5) != 0)
        u32Result |= BIT(8);
    if (lld3377xCrcCalc(u8Pat9, 5) != 0)
        u32Result |= BIT(9);
    if (lld3377xCrcCalc(u8Pat10, 5) != 0)
        u32Result |= BIT(10);

#ifdef SLAVEIF_DEBUG_COMM
    printf("SlaveIF: CRC test result: 0x%08lX\n", u32Result);
#endif
    return u32Result;
}

/**
 * @brief Packs a 40-bit message frame into a 5-byte buffer.
 * @details Packs data, address, CID, command, and CRC for SPI/TPL transmission.
 * \code
 * ------------------------------------------------------------------------------
 * |  Byte 4  |  Byte 3  |  Byte 2            |       Byte 1         |  Byte 0  |
 * |----------|----------|--------------------|----------------------|----------|
 * | bit39:31 | bit30:24 | bit23   | bit22:16 | bit15:12   | bit11:8 | bit7:0   |
 * |     memory data     | response| Register | cluster id | command |          |
 * |        data         | bResp   |   Addr   |   CID      | cmd     |  CRC     |
 * ------------------------------------------------------------------------------
 * \endcode
 * @param pu8Buf Pointer to 5-byte buffer for packed frame.
 * @param data 16-bit data to pack.
 * @param addr 7-bit register address.
 * @param CID 4-bit Cluster ID.
 * @param cmd 4-bit command.
 */
void lld3377xPackFrame(uint8_t *pu8Buf, uint16_t data, uint8_t addr, uint8_t CID, uint8_t cmd)
{
    *(pu8Buf + 4) = (uint8_t)(data >> 8);
    *(pu8Buf + 3) = (uint8_t)(data);
    *(pu8Buf + 2) = (uint8_t)(addr & 0x7F);
    *(pu8Buf + 1) = (uint8_t)((CID & 0x0F) << 4) | (cmd & 0x0F);
    *(pu8Buf + 0) = lld3377xCrcCalc(pu8Buf + 1, 4);
#ifdef SLAVEIF_DEBUG_COMM
    printf("SlaveIF: Packed frame - CID: %d, Addr: 0x%02X, Cmd: %d, Data: 0x%04X, CRC: 0x%02X\n",
           CID, addr, cmd, data, *(pu8Buf + 0));
#endif
}

/**
 * @brief Generates a new Rolling Counter (RC) value.
 * @details Returns the next RC value from RCVALUELIST.
 * @return uint8_t New RC value.
 */
uint8_t lld3377xNewRCValue(void)
{
    gu8RCIdx = (gu8RCIdx + 1) % sizeof(RCVALUELIST);
    uint8_t rc = RCVALUELIST[gu8RCIdx];
#ifdef SLAVEIF_DEBUG_COMM
    printf("SlaveIF: New RC value: %d\n", rc);
#endif
    return rc;
}

/**
 * @brief Sets the TagID for a given CID.
 * @details Assigns a new TagID to the specified CID.
 * @param CID Cluster ID (1..14).
 * @param NewTagId New TagID value (0..15).
 * @return bool True if successful, false on error.
 */
bool lld3377xSetTagID(uint8_t CID, uint8_t NewTagId)
{
    if (NewTagId > 15 || CID > 14 || CID == 0)
        return _lld3377xSetError(ERR_WrongParam);
    gTagID[CID] = NewTagId;
#ifdef SLAVEIF_DEBUG_CONFIG
    printf("SlaveIF: Set TagID %d for CID %d\n", NewTagId, CID);
#endif
    return true;
}

//=============================================================================
// Public Function Definitions - Device Control
//=============================================================================
/**
 * @brief Wakes up the MC3377x device.
 * @details Issues a wake-up sequence via CSB toggle and waits for response.
 * @return bool True if successful, false on error.
 */
bool lld3377xWakeUp(void)
{
    if (lld3377xGetError(NULL))
        return false; // only execute if no errors are pending
    if (*P_interface_ == IntTPL)
    {
        slaveIF_SPISC(0);
        Delay(DELAY_22us);
        slaveIF_SPISC(1);
        Delay(DELAY_600us);
        slaveIF_SPISC(0);
        Delay(DELAY_22us);
        slaveIF_SPISC(1);
        Delayms(WAIT_AFTER_WAKEUP);
#ifdef SLAVEIF_DEBUG_INIT
        printf("SlaveIF: TPL wake-up sequence completed\n");
#endif
        return true;
    }
    else
        return _lld3377xSetError(ERR_WrongInterface);
}

//=============================================================================
// Public Function Definitions - Register Access
//=============================================================================
/**
 * @brief Reads registers from MC3377x.
 * @details Reads up to 50 registers in a burst via SPI/TPL.
 * @param CID Cluster ID (0..15).
 * @param Register Register address (0..0x7F).
 * @param noRegs2Read Number of registers to read (1..50).
 * @param readData Pointer to array for read data (or NULL).
 * @return bool True if successful, false on error.
 */
bool lld3377xReadRegisters(uint8_t CID, uint8_t Register, uint8_t noRegs2Read, uint16_t *readData)
{
    uint8_t txFrame[MSGLEN];
    uint8_t rxFrame[MSGLEN];
    uint8_t idx = 0;
    uint8_t RC;
    uint8_t u8RxBytes;

    if (lld3377xGetError(NULL))
        return false;
    if ((noRegs2Read == 0) || (noRegs2Read > 50))
        return _lld3377xSetError(ERR_WrongParam);

    if (*P_interface_ == IntTPL)
    {
        RC = lld3377xNewRCValue();                                                      // get next RC value from list
        lld3377xPackFrame(txFrame, noRegs2Read, Register, CID, (RC << 2) | CmdRdLocal); // pack data (read noRegs2Read registers)
        SPIRxFlushBuffer();                                                             // flush receiver
        SPITxSendBuffer(txFrame, MSGLEN);                                               // transmit read request

        if (MSGLEN != SPIRxReadBytes(rxFrame, MSGLEN)) // read request "echo" and check
            return _lld3377xSetError(ERR_TX);
        if (memcmp(txFrame, rxFrame, MSGLEN))
            return _lld3377xSetError(ERR_TX);

        while (noRegs2Read-- > 0)
        {
            u8RxBytes = SPIRxReadBytes(rxFrame, MSGLEN); // read response
            if (u8RxBytes == 0)
                return _lld3377xSetError(ERR_NoResponse);
            if (u8RxBytes != MSGLEN)
                return _lld3377xSetError(ERR_ResponseLen);
            if (lld3377xCrcCalc(rxFrame, MSGLEN) != 0)
                return _lld3377xSetError(ERR_ResponseCRC);

            if (UNPACK_REGADR(rxFrame) != Register) // check response address
                return _lld3377xSetError(ERR_ResponseAdr);
            if (CID > 0)
            { // no checks for unassigned CIDs reads
                if (_cluster_[CID - 1].pTagIdList != NULL)
                { // use of TagIDs "enabled"?
                    if (bRegIsTagID(Register, _cluster_[CID - 1].pTagIdList))
                    {
                        if (UNPACK_TAGID(rxFrame) != gTagID[CID])
                        { // check TagID
                            return _lld3377xSetError(ERR_ResponseTagId);
                        }
                    }
                    else
                    {
                        if (UNPACK_RC(rxFrame) != RC)
                        { // check RC
                            return _lld3377xSetError(ERR_ResponseRC);
                        }
                    }
                }
            }
            if (readData != NULL)
            {
                readData[idx] = UNPACK_DATA(rxFrame); // read data
#ifdef SLAVEIF_DEBUG_COMM
                printf("SlaveIF: Read register 0x%02X, Data: 0x%04X\n", Register, readData ? readData[idx] : 0);
#endif
            }
            idx++;
            Register++;
        }
        return true;
    }
    else
        return _lld3377xSetError(ERR_WrongInterface);
}

/**
 * @brief Writes a register to MC3377x.
 * @details Writes data to a specified register via SPI/TPL.
 * @param CID Cluster ID (0..15).
 * @param Register Register address (0..0x7F).
 * @param writeData Data to write.
 * @param returnData Pointer to returned data (or NULL).
 * @return bool True if successful, false on error.
 */
bool lld3377xWriteRegister(uint8_t CID, uint8_t Register, uint16_t writeData, uint16_t *returnData)
{
    uint8_t txFrame[MSGLEN];
    uint8_t rxFrame[MSGLEN];
    uint8_t u8RxBytes;
    uint8_t RC;

    if (lld3377xGetError(NULL))
        return false; // only execute if no errors are pending

    if (*P_interface_ == IntTPL)
    {
        RC = lld3377xNewRCValue();                                                    // get next RC value from list
        lld3377xPackFrame(txFrame, writeData, Register, CID, (RC << 2) | CmdWrLocal); // pack data
        SPIRxFlushBuffer();                                                           // flush receiver
        SPITxSendBuffer(txFrame, MSGLEN);                                             // transmit

        if (MSGLEN != SPIRxReadBytes(rxFrame, MSGLEN)) // read request "echo" and check
            return _lld3377xSetError(ERR_TX);
        if (memcmp(txFrame, rxFrame, MSGLEN))
            return _lld3377xSetError(ERR_TX);

        u8RxBytes = SPIRxReadBytes(rxFrame, MSGLEN); // read response
        if (u8RxBytes == 0)
            return _lld3377xSetError(ERR_NoResponse);
        if (u8RxBytes != MSGLEN)
            return _lld3377xSetError(ERR_ResponseLen);
        if (lld3377xCrcCalc(rxFrame, MSGLEN) != 0)
            return _lld3377xSetError(ERR_ResponseCRC);
        if (UNPACK_RC(rxFrame) != RC)
            return _lld3377xSetError(ERR_ResponseRC);
        if (returnData != NULL)
        {
            *returnData = UNPACK_DATA(rxFrame); // return data
        }

        return RETURN_OK;
    }
    else
        return _lld3377xSetError(ERR_WrongInterface);
}

/**
 * @brief Writes a global register to all CIDs (TPL only).
 * @details Writes data to a register for all clusters via TPL.
 * @param Register Register address (0..0x7F).
 * @param writeData Data to write.
 * @return bool True if successful, false on error.
 */
bool lld3377xWriteGlobalRegister(uint8_t Register, uint16_t writeData)
{
    uint8_t txFrame[MSGLEN];
    uint8_t rxFrame[MSGLEN];

    if (lld3377xGetError(NULL))
        return false; // only execute if no errors are pending

    if (*P_interface_ == IntTPL)
    {
        lld3377xPackFrame(txFrame, writeData, Register, GLOBAL_CID, CmdWrGlobal); // 1. pack data

        SPIRxFlushBuffer();
        SPITxSendBuffer(txFrame, MSGLEN); // 2. transmit

        if (MSGLEN != SPIRxReadBytes(rxFrame, MSGLEN)) // 3. read back transmit and check
            return _lld3377xSetError(ERR_TX);
        if (memcmp(txFrame, rxFrame, MSGLEN))
            return _lld3377xSetError(ERR_TX);

        return true;
    }
    else
        return _lld3377xSetError(ERR_WrongInterface);
}
//=============================================================================
// End of File
//=============================================================================
