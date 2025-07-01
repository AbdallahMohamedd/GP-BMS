/**
 * @file        database.h
 * @brief       Public interface header for the BMS Database module.
 *
 * @details     This header defines the data structures, enums, and function prototypes
 *              used for managing Battery Management System (BMS) data, including
 *              measurement results, status information, configuration, and thresholds.
 *
 * @note        Project: Graduation Project - Battery Management System
 * @note        Engineer: Abdullah Mohamed
 * @note        Component: BMS DataBase module
 */

#ifndef COTS_BMSDATABASE_INC_DATABASE_H_
#define COTS_BMSDATABASE_INC_DATABASE_H_

//=============================================================================
// Includes
//=============================================================================
#include <COTs/DebugInfoManager/Inc/debugInfo.h> // for memcmp
#include <COTs/SlaveControlIF/Inc/slaveIF.h>
#include <COTs/ThermalManager/Inc/tempSens.h> // table for NTC resistor characteristics
#include <tpm1.h>							  // for Delay

//=============================================================================
// Defines and Macros
//=============================================================================
#define NO_CLUSTER (15u)												 //!< macro for readability
#define NO_CELLS (14u)													 //!< macro for readability
#define NO_AN (7u)														 //!< macro for readability
#define CELL1 (0U)														 //!< macro for readability
#define CELL2 (1U)														 //!< macro for readability
#define CELL3 (2U)														 //!< macro for readability
#define CELL4 (3U)														 //!< macro for readability
#define CELL5 (4U)														 //!< macro for readability
#define CELL6 (5U)														 //!< macro for readability
#define CELL7 (6U)														 //!< macro for readability
#define CELL8 (7U)														 //!< macro for readability
#define CELL9 (8U)														 //!< macro for readability
#define CELL10 (9U)														 //!< macro for readability
#define CELL11 (10U)													 //!< macro for readability
#define CELL12 (11U)													 //!< macro for readability
#define CELL13 (12U)													 //!< macro for readability
#define CELL14 (13U)													 //!< macro for readability
#define CheckCID(v) (((v) < 1) || ((v) > MAX_CLUSTER))					 //!< Macro to check if CID is within the valid range {1..MAX_CLUSTER}.
#define S19EXTENT (0xFFF80000UL)										 //!< macro to extend negative s19 to int32_t
#define S19SignExtend(s19) ((s19) & BIT(18)) ? (s19) | S19EXTENT : (s19) //!< macro to sign extend s19 bit values to int32_t
#define S19_NMAX 0x00040000UL											 //!< -2^18   = -262144
#define S19_MAX 0x0003FFFFUL											 //!< 2^18 -1 = 262143
#define CELL_BATTERY_CAPACITY_AH 1.5
#define CELL_OCV_FULL 4.2
#define CELL_OCV_EMPTY 3.0
#define PACK_BATTERY_CAPACITY_AH 21.0
#define PACK_OCV_FULL 58.8
#define PACK_OCV_EMPTY 25
//=============================================================================
// External Variables
//=============================================================================
extern uint16_t falg_temp;
extern const uint16_t _TAGID_BCC14p2[];
extern const uint16_t _TAGID_BCC14[];
extern const uint16_t _TAGID_BCC6[];
//=============================================================================
// Typedefs
//=============================================================================
/*! \brief enum for BMS modes

The graph below shows to typical BMS state machine and transistions:

\dot
digraph G {
	  BMS_Unknown -> BMS_Init;
	  BMS_Init -> BMS_Config -> BMS_Running;
	  BMS_Running -> BMS_Sleeping [label = "goto sleep"];
	  BMS_Sleeping -> BMS_Running [label = "wakeup"];
	  BMS_Init -> BMS_Error;
	  BMS_Running -> BMS_Error;
	  BMS_Sleeping -> BMS_Error;
}
\enddot

 */
typedef enum
{
	//	BMS_Unknown 	= 0,														//!< used by GUI
	BMS_Init = 1,	  //!< init phase is assigning CID
	BMS_Config = 2,	  //!< config phase applies initial loading of registers
	BMS_Running = 3,  //!< bms is running
	BMS_Sleeping = 4, //!< bms is sleeping
	BMS_Error = 5,	  //!< error phase
	BMS_Idle = 6	  //!< BMS_Idle
} TYPE_BMS_STATUS;

//! \brief structure to hold the BMS status
typedef struct
{
	TYPE_BMS_STATUS Status;	  //!< bms state (used by state machine)
	TYPE_INTERFACE Interface; //!< used interface
	TYPE_EVB EVB;			  //!< used EVB
	uint8_t NoClusters;		  //!< no of clusters attached (1..14)
	uint8_t CIDcurrent;		  //!< CID(S) which measure current (not used for demo)
} TYPE_BMS;

//! \brief structure to hold measurement results (read only) information.
typedef struct
{
	int32_t s32Current;				   //!< current reading
	uint16_t u16StackVoltage;		   //!< stack voltage reading
	uint16_t u16CellVoltage[NO_CELLS]; //!< cell voltage reading
	uint16_t u16ANVoltage[7];		   //!< ANx readings
	uint16_t u16ICTemp;				   //!< IC temperature reading
	uint16_t u16VbgADC1A;			   //!< band gap readings (diagnostics)
	uint16_t u16VbgADC1B;			   //!< band gap readings (diagnostics)
	uint16_t u16CCSamples;			   //!< number of CC samples
	int32_t s32CCCounter;			   //!< CC counter value
} TYPE_MEAS_RESULTS_RAW;

//! \brief structure to hold status (read only / clearable) information.
typedef struct
{
	uint16_t u16CellOV;		 //!< cell over voltage
	uint16_t u16CellUV;		 //!< cell under voltage
	uint16_t u16CBOpen;		 //!< cell balancing open
	uint16_t u16CBShort;	 //!< cell balancing short
	uint16_t u16CBStatus;	 //!< cell balancing status
	uint16_t u16GPIOStatus;	 //!< GPIO status
	uint16_t u16ANOtUt;		 //!< AN over/under temperature
	uint16_t u16GPIOOpen;	 //!< GPIO open
	uint16_t u16IStatus;	 //!< ISense Status
	uint16_t u16Comm;		 //!< Comm Status
	uint16_t u16Fault1;		 //!< Fault1 status
	uint16_t u16Fault2;		 //!< Fault2 status
	uint16_t u16Fault3;		 //!< Fault3 status
	uint16_t u16MeasIsense2; //!< 0x31 register
} TYPE_STATUS;

//! \brief structure to hold configuration information.
typedef struct
{
	uint16_t u16Init;			 //!<   0x01 register
	uint16_t u16SysCfgGlobal;	 //!<   0x02 register
	uint16_t u16SysCfg1;		 //!<   0x03 register
	uint16_t u16SysCfg2;		 //!<   0x04 register
	uint16_t u16SysDiag;		 //!<   0x05 register
	uint16_t u16AdcCfg;			 //!<   0x06 register
	uint16_t u16Adc2Comp;		 //!<   0x07 register
	uint16_t u16OvUvEn;			 //!<   0x08 register
	uint16_t u16GPIOCfg1;		 //!<   0x1D register
	uint16_t u16GPIOCfg2;		 //!<   0x1E register
	uint16_t u16GPIOSts;		 //!<   0x1F register
	uint16_t u16FaultMask1;		 //!<   0x27 register
	uint16_t u16FaultMask2;		 //!<   0x28 register
	uint16_t u16FaultMask3;		 //!<   0x29 register
	uint16_t u16WakeupMask1;	 //!<   0x2A register
	uint16_t u16WakeupMask2;	 //!<   0x2B register
	uint16_t u16WakeupMask3;	 //!<   0x2C register
	uint16_t u16CBCfg[NO_CELLS]; //!<   0x0C..0x19  registers
} TYPE_CONFIG;

//! \brief structure to hold threshold information.
typedef struct
{
	uint8_t u8ThAllOv;			 //!<  0x4B register
	uint8_t u8ThAllUv;			 //!<  0x4B register
	uint8_t u8ThCTxOv[NO_CELLS]; //!<  0x4C..0x59 registers
	uint8_t u8ThCTxUv[NO_CELLS]; //!<  0x4C..0x59 registers
	uint16_t u10ThANxOT[7];		 //!<  0x5A..0x60  Over Temperature  (NTC => Undervoltage) registers
	uint16_t u10ThANxUT[7];		 //!<  0x61..0x67  Under Temperature (NTC => Overvoltage)  registers
	uint16_t u12ThIsenseOC;		 //!<  0x68 register
	uint32_t u32ThCoulombCnt;	 //!<  0x69..6A registers
} TYPE_THRESHOLDS;

//! \brief structure to hold Fuse Mirror Memory data 32 x 16 bits
typedef struct
{
	uint16_t u16Data[32]; //!< ram buffer to store fuse mirror memory
} TYPE_FUSE_DATA;

//=============================================================================
// Function Prototypes
//=============================================================================
/**
 * @brief Calculation of SOC using OCV method at startup and before connection of load.
 * @param ocv_values Array of open circuit voltage values for each cell.
 * @return float Initial State of Charge (SOC) for the pack.
 */
float dataBase_initialSOC_Pack(float ocv_values[14]);

/**
 * @brief Calculation of SOC using OCV method for a single cell.
 * @param ocv Open circuit voltage of the cell.
 * @return float Initial State of Charge (SOC) for the cell.
 */
float dataBase_initialSOC_Cell(float ocv);

/**
 * @brief Calculation of SOC using Coulomb counting at runtime and after connection of load.
 * @param rawResults Raw measurement data including Coulomb counter.
 * @param voltage Current voltage reading of the battery.
 * @param current_time Current timestamp.
 * @return float Calculated State of Charge (SOC).
 */
float dataBase_calculateSOC(TYPE_MEAS_RESULTS_RAW rawResults, uint16_t voltage, uint32_t current_time);

/**
 * @brief Calculation of SOH using Coulomb counter data.
 * @param rawResults Raw measurement data including Coulomb counter.
 * @param voltage Current voltage reading of the battery.
 * @param nominal_capacity_mAh Nominal capacity of the battery in mAh.
 * @param v2res Reference voltage resolution.
 * @param current_time Current timestamp.
 * @param prev_time Previous timestamp.
 * @return float Calculated State of Health (SOH).
 */
float dataBase_calculateSOH(TYPE_MEAS_RESULTS_RAW rawResults, float voltage, float nominal_capacity_mAh,
							float v2res, uint32_t current_time, uint32_t prev_time);

/**
 * @brief Puts the MC3377x device into sleep mode.
 * @param interface The communication interface (e.g., SPI, TPL).
 * @return bool True if successful, false otherwise.
 */
bool dataBase_sleepMode(TYPE_INTERFACE interface);

/**
 * @brief Puts the MC3377x device into normal operating mode.
 * @param interface The communication interface (e.g., SPI, TPL).
 * @return bool True if successful, false otherwise.
 */
bool dataBase_normalMode(TYPE_INTERFACE interface);

/**
 * @brief Checks if the MC3377x device has woken up from sleep mode.
 * @param interface The communication interface (e.g., SPI, TPL).
 * @return bool True if the device has woken up, false otherwise.
 */
bool dataBase_check4Wakeup(TYPE_INTERFACE interface);

/**
 * @brief Starts an ADC conversion on the MC3377x device.
 * @param cid Cluster ID.
 * @param u4TagID TagID for ADC measurement.
 * @return bool True if successful, false otherwise.
 */
bool dataBase_startConvADC(uint8_t cid, uint8_t u4TagID);

/**
 * @brief Checks if the ADC conversion on the MC3377x device is still ongoing.
 * @param cid Cluster ID.
 * @return bool True if conversion is ongoing, false if complete or an error occurred.
 */
bool dataBase_ADCIsConverting(uint8_t cid);

/**
 * @brief Initializes the BMS system with a specified number of nodes.
 * @param NoOfNodes Number of nodes in the chain (1..15).
 * @return bool True if successful, false in case of errors.
 */
bool dataBase_BMSInit(uint8_t NoOfNodes);

/**
 * @brief Configures the MC3377x device with a list of settings.
 * @param cid Cluster ID to be configured.
 * @param conf Pointer to the configuration list.
 * @return bool True if successful, false otherwise.
 */
bool dataBase_bmsConfig(uint8_t cid, const SsysConf_t conf[]);

/**
 * @brief Reads the silicon revision of the MC3377x device.
 * @param cid Cluster ID.
 * @param pCluster Pointer to a structure to fill with cluster information.
 * @return bool True if successful, false otherwise.
 */
bool dataBase_getSiliconRevision(uint8_t cid, SclusterInfo_t *pCluster);

/**
 * @brief Reads the silicon type of the MC3377x device.
 * @param cid Cluster ID.
 * @param pCluster Pointer to a structure to fill with cluster information.
 * @return bool True if successful, false otherwise.
 */
bool dataBase_getSiliconType(uint8_t cid, SclusterInfo_t *pCluster);

/**
 * @brief Reads raw measurement data from the MC3377x device.
 * @param cid Cluster ID.
 * @param tagId TagID to be used to check reading against.
 * @param NoCTs Number of cell terminals to handle.
 * @param RawMeasResults Pointer to a structure to store raw measurement results.
 * @return bool True if successful, false otherwise.
 */
bool dataBase_getRawData(uint8_t cid, uint8_t tagId, uint8_t NoCTs, TYPE_MEAS_RESULTS_RAW *RawMeasResults);

/**
 * @brief Reads status registers from the MC3377x device.
 * @param cid Cluster ID.
 * @param Status Pointer to a structure to store status information.
 * @return bool True if successful, false otherwise.
 */
bool dataBase_getStatus(uint8_t cid, TYPE_STATUS *Status);

/**
 * @brief Reads threshold registers from the MC3377x device.
 * @param cid Cluster ID.
 * @param NoCTs Number of cell terminals to handle.
 * @param Threshold Pointer to a structure to store threshold information.
 * @return bool True if successful, false otherwise.
 */
bool dataBase_getThresholds(uint8_t cid, uint8_t NoCTs, TYPE_THRESHOLDS *Threshold);

/**
 * @brief Reads fuse mirror memory data from the MC3377x device.
 * @param cid Cluster ID.
 * @param fusedata Pointer to a structure to store fuse data.
 * @return bool True if successful, false otherwise.
 */
bool dataBase_readFuseMirror(uint8_t cid, TYPE_FUSE_DATA *fusedata);

/**
 * @brief Reads temperature data from thermal sensors.
 * @param RawMeasResults Pointer to a structure to store raw measurement results.
 * @return bool True if successful, false otherwise.
 */
bool dataBase_getTempRawData(TYPE_MEAS_RESULTS_RAW *RawMeasResults);

#endif /* COTS_BMSDATABASE_INC_DATABASE_H_ */
