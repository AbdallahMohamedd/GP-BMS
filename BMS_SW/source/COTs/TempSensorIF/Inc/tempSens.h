/**
 * @file       TempSensorIF.h
 * @brief      Public interface header for the Temperature Sensor Interface driver.
 *
 * @details    This header defines the public APIs for converting raw ADC values
 *             from an NTC thermistor (10kOhm, Beta: 3900) to temperature in Celsius
 *             and Kelvin using a precomputed lookup table. The table covers a range
 *             from -40°C to 125°C with a 10000Ohm reference resistor.
 *
 * @note       Project: Graduation Project - Battery Management System
 * @note       Component: Temperature Sensor Interface driver
 */

#ifndef TEMPSENSORIF_H_
#define TEMPSENSORIF_H_

//=============================================================================
// Includes
//=============================================================================
#include "COTs/DebugInfoManager/Inc/debugInfo.h"

//=============================================================================
// Definitions and Lookup Table
//=============================================================================
// Generated with CalcNTCTable.py
// NTC 10000Ohm  Beta: 3900
// Rpre 10000Ohm
// Temperature range NTC [-40..125]
#define DEGm40C  (1001)
#define DEGm39C  (999)
#define DEGm38C  (997)
#define DEGm37C  (995)
#define DEGm36C  (993)
#define DEGm35C  (991)
#define DEGm34C  (988)
#define DEGm33C  (986)
#define DEGm32C  (983)
#define DEGm31C  (980)
#define DEGm30C  (977)
#define DEGm29C  (973)
#define DEGm28C  (970)
#define DEGm27C  (966)
#define DEGm26C  (963)
#define DEGm25C  (959)
#define DEGm24C  (955)
#define DEGm23C  (950)
#define DEGm22C  (946)
#define DEGm21C  (941)
#define DEGm20C  (936)
#define DEGm19C  (931)
#define DEGm18C  (925)
#define DEGm17C  (920)
#define DEGm16C  (914)
#define DEGm15C  (908)
#define DEGm14C  (901)
#define DEGm13C  (895)
#define DEGm12C  (888)
#define DEGm11C  (881)
#define DEGm10C  (874)
#define DEGm9C  (866)
#define DEGm8C  (859)
#define DEGm7C  (851)
#define DEGm6C  (843)
#define DEGm5C  (834)
#define DEGm4C  (826)
#define DEGm3C  (817)
#define DEGm2C  (808)
#define DEGm1C  (798)
#define DEG0C  (789)
#define DEG1C  (779)
#define DEG2C  (769)
#define DEG3C  (759)
#define DEG4C  (749)
#define DEG5C  (739)
#define DEG6C  (728)
#define DEG7C  (718)
#define DEG8C  (707)
#define DEG9C  (696)
#define DEG10C  (685)
#define DEG11C  (674)
#define DEG12C  (662)
#define DEG13C  (651)
#define DEG14C  (640)
#define DEG15C  (628)
#define DEG16C  (617)
#define DEG17C  (605)
#define DEG18C  (594)
#define DEG19C  (582)
#define DEG20C  (571)
#define DEG21C  (559)
#define DEG22C  (548)
#define DEG23C  (536)
#define DEG24C  (525)
#define DEG25C  (513)
#define DEG26C  (502)
#define DEG27C  (491)
#define DEG28C  (480)
#define DEG29C  (469)
#define DEG30C  (458)
#define DEG31C  (448)
#define DEG32C  (437)
#define DEG33C  (426)
#define DEG34C  (416)
#define DEG35C  (406)
#define DEG36C  (396)
#define DEG37C  (386)
#define DEG38C  (376)
#define DEG39C  (367)
#define DEG40C  (358)
#define DEG41C  (348)
#define DEG42C  (339)
#define DEG43C  (330)
#define DEG44C  (322)
#define DEG45C  (313)
#define DEG46C  (305)
#define DEG47C  (297)
#define DEG48C  (289)
#define DEG49C  (281)
#define DEG50C  (274)
#define DEG51C  (266)
#define DEG52C  (259)
#define DEG53C  (252)
#define DEG54C  (245)
#define DEG55C  (238)
#define DEG56C  (232)
#define DEG57C  (225)
#define DEG58C  (219)
#define DEG59C  (213)
#define DEG60C  (207)
#define DEG61C  (201)
#define DEG62C  (196)
#define DEG63C  (190)
#define DEG64C  (185)
#define DEG65C  (180)
#define DEG66C  (175)
#define DEG67C  (170)
#define DEG68C  (165)
#define DEG69C  (161)
#define DEG70C  (156)
#define DEG71C  (152)
#define DEG72C  (148)
#define DEG73C  (144)
#define DEG74C  (140)
#define DEG75C  (136)
#define DEG76C  (132)
#define DEG77C  (128)
#define DEG78C  (125)
#define DEG79C  (121)
#define DEG80C  (118)
#define DEG81C  (115)
#define DEG82C  (112)
#define DEG83C  (109)
#define DEG84C  (106)
#define DEG85C  (103)
#define DEG86C  (100)
#define DEG87C  (97)
#define DEG88C  (95)
#define DEG89C  (92)
#define DEG90C  (90)
#define DEG91C  (87)
#define DEG92C  (85)
#define DEG93C  (83)
#define DEG94C  (80)
#define DEG95C  (78)
#define DEG96C  (76)
#define DEG97C  (74)
#define DEG98C  (72)
#define DEG99C  (70)
#define DEG100C  (69)
#define DEG101C  (67)
#define DEG102C  (65)
#define DEG103C  (63)
#define DEG104C  (62)
#define DEG105C  (60)
#define DEG106C  (59)
#define DEG107C  (57)
#define DEG108C  (56)
#define DEG109C  (54)
#define DEG110C  (53)
#define DEG111C  (52)
#define DEG112C  (50)
#define DEG113C  (49)
#define DEG114C  (48)
#define DEG115C  (47)
#define DEG116C  (46)
#define DEG117C  (44)
#define DEG118C  (43)
#define DEG119C  (42)
#define DEG120C  (41)
#define DEG121C  (40)
#define DEG122C  (39)
#define DEG123C  (38)
#define DEG124C  (37)
#define DEG125C  (37)

//=============================================================================
// Public Function Prototypes
//=============================================================================

/**
 * @brief      Converts a raw ADC value to temperature in Celsius.
 * @details    Searches the NTC lookup table to find the corresponding temperature
 *             index and converts it to Celsius (-40°C to 125°C range).
 * @param      u16RawValue Raw ADC value from the NTC thermistor.
 * @return     Temperature in degrees Celsius (int8_t).
 */
int8_t tempSensorIf_Raw2Celsius(uint16_t rawValue);

/**
 * @brief      Converts a raw ADC value to temperature in Kelvin.
 * @details    Converts the raw ADC value to Celsius and adds 273 to get Kelvin.
 * @param      u16RawValue Raw ADC value from the NTC thermistor.
 * @return     Temperature in degrees Kelvin (uint16_t).
 */
uint16_t tempSensorIf_Raw2Kelvin(uint16_t rawValue);

#endif /* TEMPSENSORIF_H_ */
//=============================================================================
// End of File
//=============================================================================
