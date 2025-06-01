// ----------------------------------------------------------------------------
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
// ----------------------------------------------------------------------------
//! \addtogroup platform 
// @{
// ----------------------------------------------------------------------------
/*! \brief communication IDs ("INDEX")
 * 
 */
//! \addtogroup comm_id 
// ----------------------------------------------------------------------------
// @{
// ----------------------------------------------------------------------------
#ifndef COMMID_H_
#define COMMID_H_
// ----------------------------------------------------------------------------
// commIds used for DAC control
#define TID_DAC_CT1          4060												//!< data is 32bit IEEE754
#define TID_DAC_CT2          4061												//!< data is 32bit IEEE754
#define TID_DAC_CT3          4062												//!< data is 32bit IEEE754
#define TID_DAC_CT4          4063												//!< data is 32bit IEEE754
#define TID_DAC_CT5          4064												//!< data is 32bit IEEE754
#define TID_DAC_CT6          4065												//!< data is 32bit IEEE754
#define TID_DAC_CT7          4066												//!< data is 32bit IEEE754
#define TID_DAC_CT8          4067												//!< data is 32bit IEEE754
#define TID_DAC_CT9          4068												//!< data is 32bit IEEE754
#define TID_DAC_CT10         4069												//!< data is 32bit IEEE754
#define TID_DAC_CT11         4070												//!< data is 32bit IEEE754
#define TID_DAC_CT12         4071												//!< data is 32bit IEEE754
#define TID_DAC_CT13         4072												//!< data is 32bit IEEE754
#define TID_DAC_CT14         4073												//!< data is 32bit IEEE754
#define TID_DAC_AN0          4074												//!< data is 32bit IEEE754
#define TID_DAC_AN1          4075												//!< data is 32bit IEEE754
#define TID_DAC_AN2          4076												//!< data is 32bit IEEE754
#define TID_DAC_AN3          4077												//!< data is 32bit IEEE754
#define TID_DAC_AN4          4078												//!< data is 32bit IEEE754
#define TID_DAC_AN5          4079												//!< data is 32bit IEEE754
#define TID_DAC_AN6          4080												//!< data is 32bit IEEE754
#define TID_DAC_ISD          4081												//!< data is 32bit IEEE754
#define TID_DAC_ISN          4082												//!< data is 32bit IEEE754
#define TID_DAC_VDROP        4083												//!< data is 32bit IEEE754

#define TID_DAC_APPLY        4084												//!< no data
#define ID_DAC_READY         4085												//!< no data

// Fault injection mode ID
#define TID_FIMODE			 4088												//Command for switching FI mode

// ----------------------------------------------------------------------------
// Script Engine IDs transmitted by GUI
#define TID_DOWNLOAD_START   4091                                     			//!< data is u16 noOfByte
#define TID_DOWNLOAD_DATA    4092                                     			//!< data is u32  (4 bytes) 
#define TID_DOWNLOAD_STOP    4093                                     			//!< data is u16 checksum (not used yet)

#define TID_SEREQUEST        4094                                     			//!< data is u8
#define TID_COMMAND          4095                                     			//!< data is u8   0: Reset, 1: GotoSleep
// ----------------------------------------------------------------------------
// Script Engine IDs 
//#define ID_REG_VALUES       600  // .. 600+255 = 855
#define ID_READREG_00       600
#define ID_READREG_7F       727
#define ID_READREG_VCUR     728
#define ID_READREG_COULOMB  729


#define ID_STATE      		19
//#define ID_CODELINENO		20		
#define ID_PROGCOUNTER		20		
#define ID_READBIT          21

#define ID_CONTEXT_A		22
#define ID_CONTEXT_B		23
#define ID_CONTEXT_C		24
#define ID_CONTEXT_D		25
#define ID_CONTEXT_NODES	26
#define ID_CONTEXT_LSTRES	27
#define ID_CONTEXT_CCR 		28
#define ID_CONTEXT_NODE 	29
#define ID_CONTEXT_NAVERAGE 30
#define ID_CONTEXT_OP1      31
#define ID_CONTEXT_OP2      33

#define ID_COUNTER0 		90
#define ID_COUNTER1 		91

// ----------------------------------------------------------------------------
// ID numbering scheme
//#define ID_PACKCTRL_BASE 	   0
#define ID_PACKCTRL_SW 		   0												//!< SW info   byte 0 = SW-Type  byte2= SW-Ver byte3= SW-SubVer
#define ID_PACKCTRL_INTERFACE  1
#define ID_PACKCTRL_EVB        2
#define ID_PACKCTRL_NOCLUSTER  4

#define ID_CHIPREV        3

#define ID_NOCELL         5
#define ID_MODE           6
#define ID_CIDCURRMEAS    7
#define ID_MEASPERIOD     8
#define ID_TESTMODE       9



#define ID_BASE_BMSSTATUS   10
#define ID_BASE_FAULTPIN    11
#define ID_COMM_ERR         12
#define ID_CID_SELECTED     13           // used to indicate which CID is selected (and will provide register detail information)

#define ID_MESSAGE          14

// special use for TEST-SW
#define ID_VCOM             15		
#define ID_EVENT			16
#define ID_BCCSTATE			17
#define ID_TIMESTAMP        18 

#define ID_GUID			    32
#define ID_READREG_DONE     34

#define ID_BASE_THRESHOLD   400
#define ID_BASE_DIAG       	500

#define ID_CELL_OV              100
#define ID_CELL_UV              101
#define ID_CB_OPEN_FAULT        102
#define ID_CB_SHORT_FAULT       103
#define ID_CB_DRV_STATUS        104
#define ID_GPIO_STATUS          105
#define ID_AN_OT_UT             106
#define ID_GPIO_SHORT_OPEN      107
#define ID_I_STATUS             108
#define ID_COM_STATUS           109
#define ID_FAULT_STATUS1        110
#define ID_FAULT_STATUS2        111
#define ID_FAULT_STATUS3        112
#define ID_MEAS_ISENSE2         113

#define ID_INIT                 200
#define ID_SYS_CFG_GLOBAL       201
#define ID_SYS_CFG1             202
#define ID_SYS_CFG2             203
#define ID_SYS_DIAG             204
#define ID_ADC_CFG              205
#define ID_OV_UV_EN             206
#define ID_GPIO_CFG1            207
#define ID_GPIO_CFG2            208
#define ID_GPIO_STS             209
#define ID_FAULT_MASK1          210
#define ID_FAULT_MASK2          211
#define ID_FAULT_MASK3          212
#define ID_WAKEUP_MASK1         213
#define ID_WAKEUP_MASK2         214
#define ID_WAKEUP_MASK3         215
#define ID_ADC2_OFFSET_COMP     219
#define ID_CB_CFG_1             220
#define ID_CB_CFG_2             221
#define ID_CB_CFG_3             222
#define ID_CB_CFG_4             223
#define ID_CB_CFG_5             224
#define ID_CB_CFG_6             225
#define ID_CB_CFG_7             226
#define ID_CB_CFG_8             227
#define ID_CB_CFG_9             228
#define ID_CB_CFG_10            229
#define ID_CB_CFG_11            230
#define ID_CB_CFG_12            231
#define ID_CB_CFG_13            232
#define ID_CB_CFG_14            233
#define ID_EEPROM_CTRL          235

#define ID_MEAS_VPWR            300
#define ID_MEAS_VCT1            301
#define ID_MEAS_VCT2            302
#define ID_MEAS_VCT3            303
#define ID_MEAS_VCT4            304
#define ID_MEAS_VCT5            305
#define ID_MEAS_VCT6            306
#define ID_MEAS_VCT7            307
#define ID_MEAS_VCT8            308
#define ID_MEAS_VCT9            309
#define ID_MEAS_VCT10           310
#define ID_MEAS_VCT11           311
#define ID_MEAS_VCT12           312
#define ID_MEAS_VCT13           313
#define ID_MEAS_VCT14           314
#define ID_MEAS_VCUR            315
#define ID_MEAS_VAN0            316
#define ID_MEAS_VAN1            317
#define ID_MEAS_VAN2            318
#define ID_MEAS_VAN3            319
#define ID_MEAS_VAN4            320
#define ID_MEAS_VAN5            321
#define ID_MEAS_VAN6            322
#define ID_MEAS_ICTEMP          323
#define ID_MEAS_VBG_DIAG_ADC1A  324
#define ID_MEAS_VBG_DIAG_ADC1B  325
#define ID_CC_NB_SAMPLE         326
#define ID_COULOMB_CNT          327
#define ID_SILIVON_REV          328
#define ID_DED_HAMMING_CNT      329


// IDs for Vcvfv CT functional verification 
#define ID_MEAS_VREF_ZD         330 			// CT2
#define ID_VERR_3               331
#define ID_VERR_4               332
#define ID_VERR_5               333
#define ID_VERR_6               334
#define ID_VERR_7               335
#define ID_VERR_8               336
#define ID_VERR_9               337
#define ID_VERR_10              338
#define ID_VERR_11              339
#define ID_VERR_12              340
#define ID_VERR_13              341
#define ID_VERR_14              342

#define ID_MEAS_VREF_ZD3        343 
#define ID_MEAS_VREF_ZD4        344 
#define ID_MEAS_VREF_ZD5        345 
#define ID_MEAS_VREF_ZD6        346 
#define ID_MEAS_VREF_ZD7        347 
#define ID_MEAS_VREF_ZD8        348 
#define ID_MEAS_VREF_ZD9        349 
#define ID_MEAS_VREF_ZD10       350 
#define ID_MEAS_VREF_ZD11       351 
#define ID_MEAS_VREF_ZD12       352 
#define ID_MEAS_VREF_ZD13       353 
#define ID_MEAS_VREF_ZD14       354

// IDs for Cell Terminal Functional Verification
#define ID_VLEAK_IND0_1		    355
#define ID_VLEAK_IND0_2		    356
#define ID_VLEAK_IND0_3		    357
#define ID_VLEAK_IND0_4		    358
#define ID_VLEAK_IND0_5		    359
#define ID_VLEAK_IND0_6		    360
#define ID_VLEAK_IND0_7		    361
#define ID_VLEAK_IND0_8		    362
#define ID_VLEAK_IND0_9		    363
#define ID_VLEAK_IND0_10	    364
#define ID_VLEAK_IND0_11	    365
#define ID_VLEAK_IND0_12	    366
#define ID_VLEAK_IND0_13	    367
#define ID_VLEAK_IND0_14	    368
#define ID_VLEAK_IND0_15		369

#define ID_VLEAK_IND1_1		    370
#define ID_VLEAK_IND1_2		    371
#define ID_VLEAK_IND1_3		    372
#define ID_VLEAK_IND1_4		    373
#define ID_VLEAK_IND1_5		    374
#define ID_VLEAK_IND1_6		    375
#define ID_VLEAK_IND1_7		    376
#define ID_VLEAK_IND1_8		    377
#define ID_VLEAK_IND1_9		    378
#define ID_VLEAK_IND1_10	    379
#define ID_VLEAK_IND1_11	    380
#define ID_VLEAK_IND1_12	    381
#define ID_VLEAK_IND1_13	    382
#define ID_VLEAK_IND1_14	    383
#define ID_VLEAK_IND1_15  		384


#define ID_BASE_EVALS      	    380

#define ID_TH_ALL_CT_OV         400 
#define ID_TH_ALL_CT_UV         401
#define ID_TH_CT1_OV            402
#define ID_TH_CT1_UV            403
#define ID_TH_CT2_OV            404
#define ID_TH_CT2_UV            405
#define ID_TH_CT3_OV            406
#define ID_TH_CT3_UV            407
#define ID_TH_CT4_OV            408
#define ID_TH_CT4_UV            409
#define ID_TH_CT5_OV            410
#define ID_TH_CT5_UV            411
#define ID_TH_CT6_OV            412
#define ID_TH_CT6_UV            413
#define ID_TH_CT7_OV            414
#define ID_TH_CT7_UV            415
#define ID_TH_CT8_OV            416
#define ID_TH_CT8_UV            417
#define ID_TH_CT9_OV            418
#define ID_TH_CT9_UV            419
#define ID_TH_CT10_OV           420
#define ID_TH_CT10_UV           421
#define ID_TH_CT11_OV           422
#define ID_TH_CT11_UV           423
#define ID_TH_CT12_OV           424
#define ID_TH_CT12_UV           425
#define ID_TH_CT13_OV           426
#define ID_TH_CT13_UV           427
#define ID_TH_CT14_OV           428
#define ID_TH_CT14_UV           429
#define ID_TH_AN0_OT            430
#define ID_TH_AN1_OT            431
#define ID_TH_AN2_OT            432
#define ID_TH_AN3_OT            433
#define ID_TH_AN4_OT            434
#define ID_TH_AN5_OT            435
#define ID_TH_AN6_OT            436
#define ID_TH_AN0_UT            437
#define ID_TH_AN1_UT            438
#define ID_TH_AN2_UT            439
#define ID_TH_AN3_UT            440
#define ID_TH_AN4_UT            441
#define ID_TH_AN5_UT            442
#define ID_TH_AN6_UT            443
#define ID_TH_ISENSE_OC         444
#define ID_TH_COULOMB_CNT       445


#endif /* COMMID_H_ */
//---------------------------------------------------------------------
// @}
// @}
//---------------------------------------------------------------------
