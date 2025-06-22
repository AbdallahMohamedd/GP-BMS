/**
 * @file       TempSensorIF.c
 * @brief      Implementation of the Temperature Sensor Interface driver.
 *
 * @details    This file provides the implementation for converting raw ADC values
 *             from an NTC thermistor (10kOhm, Beta: 3900) to temperature in Celsius
 *             and Kelvin using a precomputed lookup table. The table covers a range
 *             from -40°C to 125°C with a 10000Ohm reference resistor.
 *
 * @note       Project: Graduation Project - Battery Management System
 * @note       Component: Temperature Sensor Interface driver
 */

//=============================================================================
// Includes
//=============================================================================
#include <COTs/TempSensorIF/Inc/tempSens.h>

//=============================================================================
// Definitions and Lookup Table
//=============================================================================
// Generated with CalcNTCTable.py
// NTC 10000Ohm  Beta: 3900
// Rpre 10000Ohm
// Temperature range NTC [-40..125]
const uint16_t tempSensorIf_ntcLookupTable[166] = {
	    32058, 31998, 31934, 31867, 31795, 31719, 31639, 31554, 31465, 31371,
	    31272, 31167, 31058, 30942, 30821, 30694, 30561, 30422, 30276, 30124,
	    29965, 29799, 29626, 29446, 29259, 29065, 28863, 28654, 28437, 28213,
	    27982, 27743, 27496, 27242, 26981, 26712, 26436, 26153, 25863, 25567,
	    25263, 24954, 24638, 24317, 23990, 23658, 23320, 22978, 22632, 22282,
	    21928, 21570, 21210, 20848, 20483, 20117, 19749, 19381, 19012, 18642,
	    18274, 17905, 17538, 17172, 16809, 16447, 16087, 15730, 15377, 15025,
	    14679, 14336, 13997, 13662, 13333, 13007, 12686, 12371, 12060, 11755,
	    11456, 11161, 10873, 10590, 10313, 10041, 9775, 9515, 9260, 9011,
	    8769, 8531, 8298, 8074, 7853, 7637, 7428, 7223, 7023, 6831,
	    6641, 6458, 6278, 6105, 5936, 5771, 5612, 5455, 5303, 5156,
	    5013, 4875, 4741, 4610, 4483, 4360, 4238, 4122, 4009, 3899,
	    3792, 3688, 3589, 3492, 3398, 3305, 3217, 3131, 3044, 2963,
	    2886, 2809, 2735, 2660, 2590, 2523, 2456, 2394, 2331, 2269,
	    2212, 2154, 2100, 2045, 1993, 1943, 1894, 1847, 1800, 1756,
	    1711, 1667, 1625, 1587, 1548, 1509, 1473, 1437, 1401, 1368,
	    1334, 1304, 1274, 1243, 1213, 1185
};

//=============================================================================
// Public Function Definitions
//=============================================================================
/**
 * @brief      Converts a raw ADC value to temperature in Celsius.
 * @details    Searches the NTC lookup table to find the corresponding temperature
 *             index and converts it to Celsius (-40°C to 125°C range). Note that
 *             error handling for out-of-range values is marked as a TODO.
 * @param      rawValue Raw ADC value from the NTC thermistor.
 * @return     Temperature in degrees Celsius (int8_t).
 */
int8_t tempSensorIf_Raw2Celsius(uint16_t rawValue)
{
	uint8_t u8Idx = 0;		// Start at beginning
    while ((rawValue < tempSensorIf_ntcLookupTable[u8Idx]) && (u8Idx < 166))
    {                                                           // Find entry lower than raw
        u8Idx++;
    }
#ifdef TEMPSENSORIF_DEBUG_CONVERT
    PRINTF("TempSensorIF: converted to %d°C\r\r\n", (int8_t)(u8Idx - 40));
#endif

    return (int8_t)(u8Idx - 40);                                    // In Celsius
}

/**
 * @brief      Converts a raw ADC value to temperature in Kelvin.
 * @details    Calls tempSensorIf_rawToCelsius and adds 273 to convert to Kelvin.
 * @param      rawValue Raw ADC value from the NTC thermistor.
 * @return     Temperature in degrees Kelvin (uint16_t).
 */
uint16_t tempSensorIf_Raw2Kelvin(uint16_t rawValue)
{
#ifdef TEMPSENSORIF_DEBUG_CONVERT
    PRINTF("TempSensorIF: Raw value %u converted to %uK\r\r\n", rawValue, (uint16_t)(tempSensorIf_Raw2Celsius(rawValue) + 273));
#endif

    return (uint16_t)(tempSensorIf_Raw2Celsius(rawValue) + 273);     // In Kelvin
}
//=============================================================================
// End of File
//=============================================================================
