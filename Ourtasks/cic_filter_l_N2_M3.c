/******************************************************************************
* File Name          : cic_filter_l_N2_M3.c
* Date First Issued  : 09/17/2011
* Board              : STM32F103VxT6_pod_mm
* Description        : General purpose cic (long) filter for N(delays)=2, M(sections)=3
*******************************************************************************/
/*
Implement a three Cascaded Integrator Comb (CIC) filter.  (See _Digital Signal Processing in Communication Systems_,
 Marvin E. Frerking, (1994; Kluwer Academic Publishing), page 200, Figure 5.33 for a two stage CIC filter).   

This filter length is R*N, where R is the down-sampling as set in the struct and N = 2.

The number of bits required in each integrator requires three times (M=3) the number of bits
in the downsampling.  E.g. with a down-sampling of 32 (5 bits), each integrator requires 5 more bits 
than the largest data.  Three cascaded integrators add 15 bits.  If the largest input size is 24 bits
(e.g. the AD7799 register), 39 bits are required, therefore 'long long' (64 bit) arithmetic is needed.
  
The scale factor is (R*N)^M, where R is the down-sampling ratio (32 in this case), N is the number of delays 
in the differentiators (2 in this case) and M is the number of cascaded sections (3 in this case); 
hence (32*2)^3 = 2^18. (See Frerking p 202.)  If N is not a power of 2 then the scale factor will not be a 
power of two and simple shifting of the output data will not de-scale the filter (if exact de-scaling is
important, which in this application it is not).

Since effective bits are added by the low pass filtering, adjustment of the output by
could drop some useful bits.

It is absolutely essential that the integrators be intialized to zero.  There is nothing
in the math to correct for a non-zero start up, and the error accumulates.

The three cascaded sections give an impulse response that is parabolic and therefore quite
close to Gaussian, which minimizes ringing.

NOTE: 

1) The struct must be initialized with the downsampling count, and discard number (if any
initial readings are to be discarded).

2) The new reading to go into the filter is added to the struct before calling the routine,

3) The 'usFlag' is incremented when the filter has a new output.

*/

#include "cic_filter_l_N2_M3.h"


/******************************************************************************
 * unsigned short cic_filter_l_N2_M3 (struct CICLN2M3 *p, uint32_t new);
 * @brief	: Update one filtering for one adc channel
 * @param	: Pointer to struct with all the stuff
 * @param	: new = new value to added to filter
 * @return	: 0 = filtered output not ready; 1 = new filtered output ready
*******************************************************************************/
unsigned short cic_filter_l_N2_M3 (struct CICLN2M3 *p, uint32_t new)
{

long	lX1,lX2;	// Intermediate differentiator value

	/* Three stages of integration */
	p->lIntegral[0] += new;		// Incoming data is 32 bits; add to 32 bit accumulator
	p->lIntegral[1] += p->lIntegral[0];	// 1st stage feeds 2nd stage
	p->lIntegral[2] += p->lIntegral[1];	// 2nd stage feeds 3rd stage
	
	/* Decimate (down-sample) */
	p->usDecimateCt += 1;			// Decimation count
	if (p->usDecimateCt >= p->usDecimateNum) 	// Time to decimate?
	{ // Here, yes.  Do three stages of differentiation of the down sampled data
		p->usDecimateCt = 0;		// Reset decimation counter
		lX1	          = p->lIntegral[2] - p->lDiff[0][1]; // 3rd stage integral minus 3rd stage value delayed by 2
		p->lDiff[0][1] = p->lDiff[0][0];	// Move the samples down one step
		p->lDiff[0][0] = p->lIntegral[2];	// Input goes into 1st delay of the two delays

		/* Repeat differentiator for 2nd stage */
		lX2               = lX1 - p->lDiff[1][1]; // 1st stage diff- value delayed by 3
		p->lDiff[1][1] = p->lDiff[1][0];
		p->lDiff[1][0] = lX1;

		/* Repeat for 3rd stage.  Output is the filtered/decimated output */
		p->lout        = lX2  - p->lDiff[2][1];
		p->lDiff[2][1] = p->lDiff[2][0];
		p->lDiff[2][0] = lX2;

		p->usFlag += 1;	// Flag mainline that there is new output data.
		return 1;

	}	
	return 0;
}

