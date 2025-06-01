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
// ----------------------------------------------------------------------------
#include "config.h"
#include "mc3377x.h"           // low level access


#define ADC_CFG_DEFAULT     (PGA_GAIN_AUTO |ADC1_A_14bit|ADC1_B_14bit|ADC2_16bit)				// reset status
#define ADC_CFG_SETTING     (PGA_GAIN_AUTO |ADC1_A_16bit|ADC1_B_16bit|ADC2_16bit)


// ----------------------------------------------------------------------------
/*! \brief Example Configuration for MC33771 (14 cells) via SPI. 

Array of Configuration Register Values.  

For SPI its important to enable the CSB wakeup in the WAKEUP_MASK1 register!

The values are written into the BCC registers at Configuration. The writing 
order is top to bottom. So e.g. clearing of Fault conditions (FAULT_STATUS) should be done after
setting thresholds and masking. 
*/
const TYPE_BCC_CONF CONF33771SPI[] = {
	// register      	data
	// set OV & UV thresholds	
//	{ TH_ALL_CT,       	TH_OVUV_VALUE(3.0, 2.0)   },
	{ TH_CT1,      		TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT2,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT3,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT4,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT5,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT6,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT7,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT8,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT9,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT10,        	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT11,        	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT12,        	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT13,        	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT14,        	TH_OVUV_VALUE(3.0, 1.8)   }, 

	// set OT & UT thresholds	
	{ TH_AN0_OT,        DEG30C              },
	{ TH_AN1_OT,		DEG30C              },
	{ TH_AN2_OT,		DEG30C              },
	{ TH_AN3_OT,		DEG30C              },
	{ TH_AN4_OT,		DEG30C              },
	{ TH_AN5_OT,		DEG30C              },
	{ TH_AN6_OT,		DEG30C              },

	{ TH_AN0_UT,        DEG0C              },
	{ TH_AN1_UT,		DEG0C              },
	{ TH_AN2_UT,		DEG0C              },
	{ TH_AN3_UT,		DEG0C              },
	{ TH_AN4_UT,		DEG0C              },
	{ TH_AN5_UT,		DEG0C              },
	{ TH_AN6_UT,		DEG0C              },

	// define Fault Handling (mask = 1 "disable") 	
	{ FAULT_MASK1, 		0x1FF0				},// CT OT/UT/OV/UV enabled
	{ FAULT_MASK2, 		0xFE7F				},// 0xFE7F all masked
	{ FAULT_MASK3, 		0xFFFF 				},// 0xFFFF all masked

	// define Wakeup sources
	{ WAKEUP_MASK1, 	0x189F,				},	// 0x199F all masked   enabled: CSB WU 
	{ WAKEUP_MASK2, 	0xFF36,         	},	// 0xFF36 all masked
	{ WAKEUP_MASK3, 	0xBFFF,         	},	// 0xBFFF all masked

	// configuration (bits)
	{ OV_UV_EN,         0x3FFF				},	// enable OV & UV handling
	{ SYS_CFG1, 		0x9000  			},	
	{ SYS_CFG2, 		0x6330	         	},   
	{ FAULT1_STATUS, 	0xC000	 			},	// clear all bits, except POR, Reset
	{ FAULT2_STATUS, 	0 					},	// clear all bits
	{ FAULT3_STATUS, 	0 					},	// clear all bits
	{ ADC_CFG,         	ADC_CFG_SETTING	    },     // set ADC
	{0 , 0 }		// end symbol
};

// ----------------------------------------------------------------------------
/*! \brief Example Configuration for MC33771 (14 cells) via TPL. 

Array of Configuration Register Values.  

For TPL typically the CSB wakeup is disabled (in the WAKEUP_MASK1 register)!
 
The values are written into the BCC registers at Configuration. The writing 
order is top to bottom. So e.g. clearing of Fault conditions (FAULT_STATUS) should be done after
setting thresholds and masking. 
*/
const TYPE_BCC_CONF CONF33771TPL[] = {
	// register      	data
	// set OV & UV thresholds	
//	{ TH_ALL_CT,       	TH_OVUV_VALUE(3.0, 2.0)   },
	{ TH_CT1,      		TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT2,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT3,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT4,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT5,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT6,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT7,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT8,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT9,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT10,        	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT11,        	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT12,        	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT13,        	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT14,        	TH_OVUV_VALUE(3.0, 1.8)   }, 


	// set OT & UT thresholds	
	{ TH_AN0_OT,        DEG30C              },
	{ TH_AN1_OT,		DEG30C              },
	{ TH_AN2_OT,		DEG30C              },
	{ TH_AN3_OT,		DEG30C              },
	{ TH_AN4_OT,		DEG30C              },
	{ TH_AN5_OT,		DEG30C              },
	{ TH_AN6_OT,		DEG30C              },

	{ TH_AN0_UT,        DEG0C              },
	{ TH_AN1_UT,		DEG0C               },
	{ TH_AN2_UT,		DEG0C               },
	{ TH_AN3_UT,		DEG0C               },
	{ TH_AN4_UT,		DEG0C               },
	{ TH_AN5_UT,		DEG0C               },
	{ TH_AN6_UT,		DEG0C               },

//	{ TH_ISENSE_OC,		0x000,              },
//	{ TH_COULOMB_H,		0x0000,             },
//	{ TH_COULOMB_L,		0x0000,             },
	
	
	// define Fault Handling (mask = 1 "disable") 	
	{ FAULT_MASK1, 		0x1FF0				},// CT OT/UT/OV/UV enabled
	{ FAULT_MASK2, 		0xFE7F				},// 0xFE7F all masked
	{ FAULT_MASK3, 		0xFFFF 				},// 0xFFFF all masked

	// define Wakeup sources
	{ WAKEUP_MASK1, 	0x199F,				},	// 0x199F all masked
	{ WAKEUP_MASK2, 	0xFF36,         	},	// 0xFF36 all masked
	{ WAKEUP_MASK3, 	0xBFFF,         	},	// 0xBFFF all masked

	// configuration (bits)
	{ OV_UV_EN,         0x3FFF				},	// enable OV & UV handling
	{ SYS_CFG1, 		0x9000	 			},	
	{ SYS_CFG2, 		0x6330	         	},  
	{ FAULT1_STATUS, 	0xC000	 			},	// clear all bits, except POR, Reset
	{ FAULT2_STATUS, 	0 					},	// clear all bits
	{ FAULT3_STATUS, 	0 					},	// clear all bits
	{ ADC_CFG,         	ADC_CFG_SETTING	    },     // set ADC
	{0 , 0 }		// end symbol
};

// ----------------------------------------------------------------------------
/*! \brief Example Configuration for MC33772 (6 cells) via SPI. 

Array of Configuration Register Values.  

For SPI its important to enable the CSB wakeup in the WAKEUP_MASK1 register!

The values are written into the BCC registers at Configuration. The writing 
order is top to bottom. So e.g. clearing of Fault conditions (FAULT_STATUS) should be done after
setting thresholds and masking. 
*/
const TYPE_BCC_CONF CONF33772SPI[] = {
//	// register      	data
	// set OV & UV thresholds	
//	{ TH_ALL_CT,       	TH_OVUV_VALUE(3.0, 2.0)   },
	{ TH_CT1,      		TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT2,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT3,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT4,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT5,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT6,          	TH_OVUV_VALUE(3.0, 1.8)   },

	// set OT & UT thresholds	
	{ TH_AN0_OT,        DEG30C,                   },
	{ TH_AN1_OT,		DEG30C,                   },
	{ TH_AN2_OT,		DEG30C,                   },
	{ TH_AN3_OT,		DEG30C,                   },
	{ TH_AN4_OT,		DEG30C,                   },
	{ TH_AN5_OT,		DEG30C,                   },
	{ TH_AN6_OT,		DEG30C,                   },

	{ TH_AN0_UT,        DEG0C,                    },
	{ TH_AN1_UT,		DEG0C,                    },
	{ TH_AN2_UT,		DEG0C,                    },
	{ TH_AN3_UT,		DEG0C,                    },
	{ TH_AN4_UT,		DEG0C,                    },
	{ TH_AN5_UT,		DEG0C,                    },
	{ TH_AN6_UT,		DEG0C,                    },
	
//	{ TH_ISENSE_OC,		0x000,                    },
//	{ TH_COULOMB_H,		0x0000,                    },
//	{ TH_COULOMB_L,		0x0000,                    },
	

	// define Fault Handling (mask = 1 "disable") 	
	{ FAULT_MASK1, 		0x1FF0,				},// CT OT/UT/OV/UV enabled
	{ FAULT_MASK2, 		0xFE7F,				},// 0xFE7F all masked
	{ FAULT_MASK3, 		0xFFFF, 			},// 0xFFFF all masked

	// define Wakeup sources
	{ WAKEUP_MASK1, 	0x189F,				},	// 0x199F all masked   enabled: CSB WU 
	{ WAKEUP_MASK2, 	0xFF36,         	},	// 0xFF36 all masked
	{ WAKEUP_MASK3, 	0xA03F,         	},	// 0xA03F all masked

	// configuration (bits)
	{ OV_UV_EN,         0x003F,			},	// enable OV & UV handling
	{ SYS_CFG1, 		0x9000, 		},	
	{ SYS_CFG2, 		0x6330,         },  
	{ FAULT1_STATUS, 	0xC000, 		},	// clear all bits, except POR, Reset
	{ FAULT2_STATUS, 	0, 				},	// clear all bits
	{ FAULT3_STATUS, 	0, 				},	// clear all bits
	{ ADC_CFG,         	ADC_CFG_SETTING	    },     // set ADC
	{0 , 0 }		// end symbol
};
// ----------------------------------------------------------------------------
/*! \brief Example Configuration for MC33772 (6 cells) via TPL. 

Array of Configuration Register Values.  

For TPL typically the CSB wakeup is disabled (in the WAKEUP_MASK1 register)!

The values are written into the BCC registers at Configuration. The writing 
order is top to bottom. So e.g. clearing of Fault conditions (FAULT_STATUS) should be done after
setting thresholds and masking. 
*/
const TYPE_BCC_CONF CONF33772TPL[] = {
//	// register      	data
	// set OV & UV thresholds	
//	{ TH_ALL_CT,       	TH_OVUV_VALUE(3.0, 2.0)   },
	{ TH_CT1,      		TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT2,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT3,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT4,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT5,          	TH_OVUV_VALUE(3.0, 1.8)   },
	{ TH_CT6,          	TH_OVUV_VALUE(3.0, 1.8)   },

	// set OT & UT thresholds	
	{ TH_AN0_OT,        DEG30C,                   },
	{ TH_AN1_OT,		DEG30C,                   },
	{ TH_AN2_OT,		DEG30C,                   },
	{ TH_AN3_OT,		DEG30C,                   },
	{ TH_AN4_OT,		DEG30C,                   },
	{ TH_AN5_OT,		DEG30C,                   },
	{ TH_AN6_OT,		DEG30C,                   },

	{ TH_AN0_UT,        DEG0C,                    },
	{ TH_AN1_UT,		DEG0C,                    },
	{ TH_AN2_UT,		DEG0C,                    },
	{ TH_AN3_UT,		DEG0C,                    },
	{ TH_AN4_UT,		DEG0C,                    },
	{ TH_AN5_UT,		DEG0C,                    },
	{ TH_AN6_UT,		DEG0C,                    },

	// define Fault Handling (mask = 1 "disable") 	
	{ FAULT_MASK1, 		0x1FF0,				},// CT OT/UT/OV/UV enabled
	{ FAULT_MASK2, 		0xFE7F,				},// 0xFE7F all masked
	{ FAULT_MASK3, 		0xFFFF, 			},// 0xFFFF all masked

	// define Wakeup sources
	{ WAKEUP_MASK1, 	0x199F,				},	// 0x199F all masked
	{ WAKEUP_MASK2, 	0xFF36,         	},	// 0xFF36 all masked
	{ WAKEUP_MASK3, 	0xA03F,         	},	// 0xA03F all masked

	// configuration (bits)
	{ OV_UV_EN,         0x003F,			},	// enable OV & UV handling
	{ SYS_CFG1, 		0x9000, 		},	
	{ SYS_CFG2, 		0x6330,         },  
	{ FAULT1_STATUS, 	0xC000, 		},	// clear all bits, except POR, Reset
	{ FAULT2_STATUS, 	0, 				},	// clear all bits
	{ FAULT3_STATUS, 	0, 				},	// clear all bits
	{ ADC_CFG,         	ADC_CFG_SETTING	    },     // set ADC
	{0 , 0 }		// end symbol
}; 
