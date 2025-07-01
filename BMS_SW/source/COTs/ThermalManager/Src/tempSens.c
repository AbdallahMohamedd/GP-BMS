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
 * @note       Author: Amr Ahmed
 */

//=============================================================================
// Includes
//=============================================================================
#include <COTs/ThermalManager/Inc/tempSens.h>

//=============================================================================
// Public Function Definitions
//=============================================================================
/**
 * @brief      Initializes the ADC module for temperature sensing.
 * @details    Configures the ADC16 module with 16-bit resolution and external
 *             reference voltage (3.3V). Hardware triggering is disabled.
 */
void thermalManager_Init(void)
{
    adc16_config_t adc16ConfigStruct;

    // Retrieve default ADC configuration
    ADC16_GetDefaultConfig(&adc16ConfigStruct);

    // Set ADC to 16-bit single-ended resolution
    adc16ConfigStruct.resolution = kADC16_ResolutionSE16Bit;

    // Use external Vref as reference voltage source
    adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;

    // Initialize ADC with the configured settings
    ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);

    // Disable hardware triggering for software control
    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false);
}

/**
 * @brief      Reads raw ADC data from the specified channel.
 * @details    Configures the ADC channel, performs a conversion, and returns
 *             the raw value. Blocks until conversion is complete.
 * @param      channel The ADC channel number to read from.
 * @return     Raw ADC value (uint16_t).
 */
uint16_t thermalManager_readRawData(uint32_t channel)
{
    adc16_channel_config_t adc16ChannelConfigStruct = {0};

    // Configure ADC channel settings
    adc16ChannelConfigStruct.channelNumber = channel;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;

#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    // Disable differential conversion mode if supported
    adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif

    // Set the channel configuration for the ADC
    ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);

    // Wait until the conversion is complete
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP)))
    {
    }

    // Return the converted raw ADC value
    return ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP);
}

/**
 * @brief      Converts a raw ADC value to temperature in Celsius.
 * @details    Uses the Steinhart-Hart equation to convert the raw ADC value
 *             to temperature in Celsius based on NTC thermistor characteristics.
 *             Note that error handling for out-of-range values is pending.
 * @param      rawValue Raw ADC value from the NTC thermistor.
 * @return     Temperature in degrees Celsius (float).
 */
float thermalManager_Raw2Celsius(uint16_t adc_value)
{
    // Calculate input voltage from ADC value
    float v_in = (adc_value / ADC_MAX) * VREF;

    // Calculate thermistor resistance using voltage divider formula
    float resistance = SERIES_RESISTOR * ((VREF - v_in) / v_in);

    // Apply Steinhart-Hart equation steps
    float steinhart = resistance / NOMINAL_RESISTANCE;    // Ratio of resistance to nominal (R/R0)
    steinhart = log(steinhart);                           // Natural logarithm of the ratio
    steinhart /= BETA_COEFFICIENT;                        // Divide by Beta coefficient
    steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);    // Add inverse of nominal temperature in Kelvin
    steinhart = 1.0 / steinhart;                          // Invert to get temperature in Kelvin
    steinhart -= 273.15;                                  // Convert to Celsius

    // Optional debug output if enabled
#ifdef TEMPSENSORIF_DEBUG_CONVERT
    PRINTF("TempSensorIF: converted to %d°C\r\r\n", (int8_t)steinhart);
#endif

    // Return the calculated temperature
    return steinhart;
}

//=============================================================================
// End of File
//=============================================================================