/******************************************************************************
* File Name          : cic_filter_l_N2_M3.h
* Date First Issued  : 09/17/2011
* Board              : STM32F103VxT6_pod_mm
* Description        : General purpose cic (long) filter for N(delays)=2, M(sections)=3
*******************************************************************************/

#ifndef __CIC_FILTER_L_N2_M3
#define __CIC_FILTER_L_N2_M3

#include <stdint.h>

/* One of these for each value filtered will is required */
struct CICLN2M3
{
	unsigned short	usDecimateNum;	// Downsampling number
	unsigned short	usDiscard;	// Initial discard count
	int		nIn;		// New reading to be filtered
	long 		lIntegral[3];	// Three stages of Z^(-1) integrators
	long		lDiff[3][2];	// Three stages of Z^(-2) delay storage
	long		lout;		// Filtered/decimated data output
	unsigned short	usDecimateCt;	// Downsampling counter
	unsigned short	usFlag;		// Filtered/decimated data ready counter
};

/******************************************************************************/
unsigned short cic_filter_l_N2_M3 (struct CICLN2M3 *p, uint32_t new);
/* @brief	: Update one filtering for one adc channel
 * @param	: Pointer to struct with all the stuff
 * @param	: new = new value to added to filter
 * @return	: 0 = filtered output not ready; 1 = new filtered output ready
*******************************************************************************/

#endif 
