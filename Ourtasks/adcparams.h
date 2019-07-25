/******************************************************************************
* File Name          : adcparams.h
* Date First Issued  : 07/23/2019
* Board              : DiscoveryF4
* Description        : Parameters for ADC app configuration
*******************************************************************************/
/* CALIBRATION NOTES:
                    Min  Typ  Max 
Internal reference: 1.16 1.20 1.24 V

Temperature sensor specs
                 Min  Typ  Max
Average slope    4.0  4.3  4.6 mV/°C
Voltage at 25 °C 1.34 1.43 1.52 V
*/

#ifndef __ADCPARAMS
#define __ADCPARAMS

#include "iir_filter_lx.h"
#include "adc_idx_v_struct.h"
#include "cic_filter_l_N2_M3.h"

#define ADC1DMANUMSEQ        16 // Number of DMA scan sequences in 1/2 DMA buffer
#define ADC1IDX_ADCSCANSIZE   6 // Number ADC channels read
#define ADCSCALEbits         14 // 2^x scale large
#define ADCSCALEbitsy         7 // 2^x scale small
#define ADCSCALEbitsitmp      3 // 2^x scale just enough
#define ADCEXTENDSUMCT     1024 // Sum of 1/2 DMA sums for addition averaging

/* ADC reading sequence/array indices                         */
/* These indices -=>MUST<= match the hardware ADC scan sequence    */
#define ADC1IDX_HIGHVOLT1     0   // PA1 IN1  - Battery voltage
#define ADC1IDX_HIGHVOLT2     1   // PA2 IN2  - DMOC +
#define ADC1IDX_HIGHVOLT3     2   // PA3 IN3  - DMOC -
#define ADC1IDX_HIGHVOLT4     3   // PA4 IN4  - spare
#define ADC1IDX_INTERNALTEMP  4   // IN17     - Internal temperature sensor
#define ADC1IDX_INTERNALVREF  5   // IN18     - Internal voltage reference

/* Copied for convenience.
// IIR filter (int) parameters
struct IIR_L_PARAM
{
	int32_t k;		// Filter parameter for setting time constant
	int32_t scale;		// Scaling improve spare bits with integer math
};

struct IIRFILTERL
{
	double d_out;		// output as double
	int32_t z;		// Z^(-1)
	float f_out;		// output as float
	struct IIR_L_PARAM* pprm; // Pointer to k and scale for this filter
	uint8_t sw;		// Init switch
};
*/

/* Working values for internal Vref and temperature sensors. */
struct ADCINTERNAL
{
//	struct IIRFILTERL iiradcvref; // Intermediate filter params: vref 
//	struct IIRFILTERL iiradctemp; // Intermediate filter params: temperature sensor

//	uint32_t adcfilvref;  // Filtered ADC[Vref]
//	uint32_t adcfiltemp;  // Filtered ADC[temperature]

	uint32_t adcvref;    // Do I need this?
	uint32_t adccmpvref; // scaled vref compensated for temperature

	double dvref;        // (double) vref computed from calibration params
	uint32_t vref;       // (scaled) vref computed from calibration params

	uint32_t iRslope;    // (scaled) Reciprocal of temperature sensor slope
	double   V25;        // (double) Computed V25 (no)
	uint32_t irmtemp;    // (scaled) calibration temperature
	uint32_t itemp;      // (scaled) temperature (degC)
	uint32_t yRs;        // (smaller scaled) reciprocal of slope
	uint32_t iv25;       // (scaled) dvtemp (e.g. (1.43 << ADCSCALEbits))
};


struct ADCCHANNEL	
{
	struct IIRFILTERL iir;// Intermediate filter params
	uint32_t adcfil;      // Filtered ADC reading
	uint32_t ival;        // Reading: calibrated scaled int32_t
	uint16_t sum;         // Sum of 1/2 DMA buffer
	uint32_t xsum[2];     // Extended sum
	double k;             // Divider scale
	double dscale;
};

/* struct allows pointer to access raw and calibrated ADC1 data. */
struct ADCFUNCTION
{
	struct ADCCONTACTORLC lc;     // Local Copy of parameters
	struct ADCINTERNAL    intern; // Vref & temperature
//	struct ADCABSOLUTE    hv[4];  // High voltages
	struct ADCCHANNEL	 chan[ADC1IDX_ADCSCANSIZE]; // ADC sums
	uint32_t ctr; // Running count of updates.
	uint32_t idx_xsum;
};

/* *************************************************************************/
void adcparams_init(void);
/*	@brief	: Copy parameters into structs
 * NOTE: => ASSUMES ADC1 ONLY <==
 * *************************************************************************/
void adcparams_cal(void);
/*	@brief	: calibrate and filter ADC readings
 * *************************************************************************/

extern struct ADCFUNCTION adc1;

#endif
