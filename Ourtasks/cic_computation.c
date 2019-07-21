/******************************************************************************
* File Name          : cic_computation.c
* Date First Issued  : 07/17/2019
* Board              : --
* Description        : CIC N2 M3 filtering
*******************************************************************************/

#include "cic_computation.h"
#include "cic_filter_l_N2_M3.h"
#include "adcparams.h"

#define DECIMATION 32
#define DISCARD  2

/******************************************************************************
 * static void init(struct CICLN2M3 *p);
 * @brief 	: Initialize CIC struct
 * @param	: p = pointer to CIC struct 
*******************************************************************************/
/*
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
*/

static void init(struct CICLN2M3 *p)
{
	int j;

	/* Initialize the structs that hold the CIC filtering intermediate values. */
		p->usDecimateNum = DECIMATION; // Decimation number
		p->usDecimateCt = 0;		// Decimation counter
		p->usDiscard = DISCARD;	// Initial discard count
		p->usFlag = 0;			// 1/2 buffer flag
		for (j = 0; j < 3; j++)
		{ // Very important that the integrators begin with zero.
			p->lIntegral[j] = 0;
			p->lDiff[j][0] = 0;
			p->lDiff[j][1] = 0;
		}	
	return;
}

/******************************************************************************
 * void cic_computation_init(struct ADCFUNCTION* p);
 * @brief 	: Initialize CIC struct for all ADC channels
 * @param	: p = pointer to ADC struct with "everything"
*******************************************************************************/
void cic_computation_init(struct ADCFUNCTION* p)
{
	int i;
	for (i = 0; i < ADC1IDX_ADCSCANSIZE; i++)
	{
		init(&p->chan[i].cic);
	}
	return;
}

/******************************************************************************
 * void cic_computation_filtering(struct ADCFUNCTION* padc);
 * @brief 	: Run the iir filtered sums through cic w decimation 
*******************************************************************************/
void cic_computation_filtering(struct ADCFUNCTION* padc)
{
	cic_filter_l_N2_M3(&padc->chan[ADC1IDX_5VOLTSUPPLY ].cic, padc->v5.adcfil);
	cic_filter_l_N2_M3(&padc->chan[ADC1IDX_12VRAWSUPPLY].cic, padc->v12.adcfil);
	cic_filter_l_N2_M3(&padc->chan[ADC1IDX_CURRENTTOTAL].cic, padc->cur1.adcfil);
	cic_filter_l_N2_M3(&padc->chan[ADC1IDX_CURRENTMOTOR].cic, padc->cur2.adcfil);
	cic_filter_l_N2_M3(&padc->chan[ADC1IDX_INTERNALTEMP].cic, padc->intern.adcfilvref);
	cic_filter_l_N2_M3(&padc->chan[ADC1IDX_INTERNALVREF].cic, padc->intern.adcfilvref);
	return;
}

