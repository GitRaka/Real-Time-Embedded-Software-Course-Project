/*
 * rtd.h
 *
 *  Created on: 25-Jul-2020
 *      Author: Rakesh
 */

#ifndef APPLICATION_USER_RTD_H_
#define APPLICATION_USER_RTD_H_

#include "adchal.h"

/**
 * @addtogroup group_RTD
 *
 * @{
 */

/**
 * @brief Describes the RTD Types
 */
typedef enum RTDtypeDef {
	Pt,			// Platinum Resistance Thermometer
	Ni,			// Nickel Resistance Thermometer
	Cu,			// Copper Resistance Thermometer
	NiFe		// Nickel Iron Resistance Thermometer
} RTD_Type;

/**
 * @brief Describes the RTD circuit connection topology
 */
typedef enum RTDWiringConfDef {
	Two_Wire_Low_Side_Ref,					// http://www.ti.com/lit/an/sbaa329/sbaa329.pdf
	Two_Wire_High_Side_Ref,					// http://www.ti.com/lit/an/sbaa336/sbaa275.pdf
	Three_Wire_Low_Side_Ref_Two_IDAC,		// http://www.ti.com/lit/an/sbaa330a/sbaa330a.pdf
	Three_Wire_Low_Side_Ref_One_IDAC,		// http://www.ti.com/lit/an/sbaa334/sbaa334.pdf
	Three_Wire_High_Side_Ref_One_IDAC,		// http://www.ti.com/lit/an/sbaa336/sbaa275.pdf
	Three_Wire_High_Side_Ref_Two_IDAC,		// http://www.ti.com/lit/an/sbaa310/sbaa310.pdf
	Four_Wire_Low_Side_Ref,				    // http://www.ti.com/lit/an/sbaa336/sbaa336.pdf
	Four_Wire_High_Side_Ref,			    // http://www.ti.com/lit/an/sbaa336/sbaa275.pdf
} RTD_WiringConf;

/**
 * @brief Describes the RTD Characteristics structure
 */
#ifdef POLYNOMIAL
	typedef struct RTDsetDef {
		const float 	a;			// RTD Callendarâ€“Van Dusen coefficient a
		const float 	b;			// RTD Callendarâ€“Van Dusen coefficient b
		const float		c;			// RTD Callendarâ€“Van Dusen coefficient c
		const float		R0;			// RTD resistance at 0C
		RTD_WiringConf 	wiring;   	// RTD circuit wiring configuration
		float			Rref; 		// RTD circuit reference resistance
	} RTD_Set;
#else
	typedef struct RTDsetDef {
		const float		R0;			// RTD resistance at 0C
		const float 	incr;		// RTD LUT temperature increment between values
		const int		min;		// RTD LUT minimum temperature
		const int 		max;		// RTD LUT maximum temperature
		const float 	*table;		// RTD LUT
		const int   	size;		// Number of entries in RTD LUT
		RTD_WiringConf 	wiring;   	// RTD circuit wiring configuration
		float			Rref; 		// RTD circuit reference resistance
	} RTD_Set;
#endif


//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************
float  Convert_Code2RTD_Resistance( ADCchar_Set *adcChars, RTD_Set *rtdSet );
float RTD_Linearization( RTD_Set *rtdSet, float RTDres );


#endif /* APPLICATION_USER_RTD_H_ */
