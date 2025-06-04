// --------------------------------------------------------------------
//  Copyright (c) 2015, NXP Semiconductors.
//  All rights reserved.
// 
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
// 
//  o Redistributions of source code must retain the above copyright notice, this list
//    of conditions and the following disclaimer.
// 
//  o Redistributions in binary form must reproduce the above copyright notice, this
//    list of conditions and the following disclaimer in the documentation and/or
//    other materials provided with the distribution.
// 
//  o Neither the name of NXP Semiconductors nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
// 
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
//  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
//  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// --------------------------------------------------------------------
#include <COTs/NTC/Inc/ntc.h>
#include <math.h>
// --------------------------------------------------------------------
// generated with CalcNTCTable.py
// NTC 10000Ohm  Beta:3900
// Rpre 10000Ohm
// temperature range NTC[-40..125]
const u16 NTC[166] =  {
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
// --------------------------------------------------------------------
/*! \brief Converts the rawValue to degree Celsius 
 * 
 * @param rawValue
 * @return converted value in degree Celsius
 */

int8_t NTCRaw2Celsius(uint16_t rawValue) {
	if (rawValue >= 65535 || rawValue == 0) return -40.0; // Invalid
	float V_AN0 = (rawValue / 65535.0) * 5.0;
	float R_NTC = 10.0 * (5.0 - V_AN0) / V_AN0; // 10 kΩ pull-up
	if (R_NTC <= 0.0) return 125.0;
	float beta = 3950; // Adjust to your NTC’s Beta
	float T_K = 1.0 / ((1.0 / 298.15) + (1.0 / beta) * log(R_NTC / 100.0));
	float T_C = T_K - 273.15;
	return (T_C < -40.0) ? -40.0 : (T_C > 125.0) ? 125.0 : T_C;
}
// --------------------------------------------------------------------
/*! \brief Converts the rawValue to degree Kelvin 
 * 
 * @param rawValue
 * @return converted value in degree Kelvin
 */
u16 NTCRaw2Kelvin(u16 rawValue) {

	return (u16)(NTCRaw2Celsius(rawValue)+273);									// in Kelvin	
}
