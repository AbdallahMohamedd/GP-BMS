/**
 * @file       TempSensorIF.c
 * @brief      Implementation of the Temperature Sensor Interface driver.
 *
 * @details    This file provides the implementation for converting raw ADC values
 *             from an NTC thermalManager_Raw2Celsius (10kOhm, Beta: 3900) to temperature in Celsius
 *             and Kelvin using a precomputed lookup table. The table covers a range
 *             from -40°C to 125°C with a 10000Ohm reference resistor.
 *
 * @note       Project: Graduation Project - Battery Management System
 * @note       Component: Temperature Sensor Interface driver
 */

//=============================================================================
// Includes
//=============================================================================
#include <COTs/ThermalManager/Inc/tempSens.h>

//=============================================================================
// Public Function Definitions
//=============================================================================
/**
 * @brief      Converts a raw ADC value to temperature in Celsius.
 * @details    Searches the NTC lookup table to find the corresponding temperature
 *             index and converts it to Celsius (-40°C to 125°C range). Note that
 *             error handling for out-of-range values is marked as a TODO.
 * @param      rawValue Raw ADC value from the NTC thermalManager_Raw2Celsius.
 * @return     Temperature in degrees Celsius (int8_t).
 */
float thermalManager_Raw2Celsius(uint16_t adc_value)
{
	float v_in = (adc_value / ADC_MAX) * VREF;
	float resistance = SERIES_RESISTOR * ((VREF - v_in) / v_in);

	float steinhart = resistance / NOMINAL_RESISTANCE; // (R/R0)
	steinhart = log(steinhart);						   // ln(R/R0)
	steinhart /= BETA_COEFFICIENT;					   // 1/B * ln(R/R0)
	steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15); // + (1/To)
	steinhart = 1.0 / steinhart;					   // Invert to Kelvin
	steinhart -= 273.15;							   // Convert to Celsius

#ifdef TEMPSENSORIF_DEBUG_CONVERT
	PRINTF("TempSensorIF: converted to %d°C\r\r\n", (int8_t)steinhart);
#endif
	return steinhart;
}

/**
 * @brief      Converts a raw ADC value to temperature in Kelvin.
 * @details    Calls tempSensorIf_rawToCelsius and adds 273 to convert to Kelvin.
 * @param      rawValue Raw ADC value from the NTC thermalManager_Raw2Celsius.
 * @return     Temperature in degrees Kelvin (uint16_t).
 */
//=============================================================================
// End of File
//=============================================================================
