/******************************************************************************
* File Name          : adcparams.h
* Date First Issued  : 06/15/2019
* Board              : DiscoveryF4
* Description        : Parameters for ADC app configuration
*******************************************************************************/
/* CALIBRATION NOTES:

5 volt supply calibration:
[Do this first: This calibration is applied to all 5v sensors]
- Measure 5v supply
- Measure Vdd
- Display ADCsum[5v supply]
Compute--
Ratio 'sensor5vcal' = (Vdd/V5volt) * (ADCsum[5v supply]/(4095*numdmaseq)
 where numdmaseq = number of ADC scans per 1/2 dma (i.e. number readings in sum)

Ratiometric 5v sensor calibration:
- With 5v supply calibration compiled-in:
- Measure Sensor voltage: Vx
- Display:
  . ADCsum[sensor]
  . Va = (V5 * ratio)/ADCsum[5v]
Compute
  Calibration ratio = ADCsum[sensor]/Va

                    Min  Typ  Max 
Internal reference: 1.16 1.20 1.24 V

Temperature sensor specs
                 Min  Typ  Max
Average slope    4.0  4.3  4.6 mV/°C
Voltage at 25 °C 1.34 1.43 1.52 V

NOTE: 5v supply w LM2596 dc-dc switcher
60 mv peak=peak triangle wave w two 100u caps
20 mv peak-peal by adding 1000u cap

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
#define ADC1IDX_5VOLTSUPPLY   0   // PA0 IN0  - 5V sensor supply
#define ADC1IDX_CURRENTTOTAL  1   // PA5 IN5  - Current sensor: total battery current
#define ADC1IDX_CURRENTMOTOR  2   // PA6 IN6  - Current sensor: motor
#define ADC1IDX_12VRAWSUPPLY  3   // PA7 IN7  - +12 Raw power to board
#define ADC1IDX_INTERNALTEMP  4   // IN17     - Internal temperature sensor
#define ADC1IDX_INTERNALVREF  5   // IN18     - Internal voltage reference

/* Calibration option.                                    */
/* Calibration is applied after compensation adjustments. */
#define ADC1PARAM_CALIBTYPE_RAW_F  0    // No calibration applied: FLOAT
#define ADC1PARAM_CALIBTYPE_OFSC   1    // Offset & scale (poly ord 0 & 1): FLOAT
#define ADC1PARAM_CALIBTYPE_POLY2  2    // Polynomial 2nd ord: FLOAT
#define ADC1PARAM_CALIBTYPE_POLY3  3    // Polynomial 3nd ord: FLOAT
#define ADC1PARAM_CALIBTYPE_RAW_UI 4    // No calibration applied: UNSIGNED INT

/* Compensation type                                         */
/* Assumes 5v sensor supply is measured with an ADC channel. */
#define ADC1PARAM_COMPTYPE_NONE      0     // No supply or temp compensation applied
#define ADC1PARAM_COMPTYPE_RATIOVDD  1     // Vdd (3.3v nominal) ratiometric
#define ADC1PARAM_COMPTYPE_RATIO5V   2     // 5v ratiometric with 5->Vdd measurement
#define ADC1PARAM_COMPTYPE_RATIO5VNO 3     // 5v ratiometric without 5->Vdd measurement
#define ADC1PARAM_COMPTYPE_VOLTVDD   4     // Vdd (absolute), Vref compensation applied
#define ADC1PARAM_COMPTYPE_VOLTVDDNO 5     // Vdd (absolute), no Vref compensation applied
#define ADC1PARAM_COMPTYPE_VOLTV5    6     // 5v (absolute), with 5->Vdd measurement applied
#define ADC1PARAM_COMPTYPE_VOLTV5NO  7     // 5v (absolute), without 5->Vdd measurement applied

/* Filter type codes */
#define ADCFILTERTYPE_NONE		0  // Skip filtering
#define ADCFILTERTYPE_IIR1		1  // IIR single pole
#define ADCFILTERTYPE_IIR2		2  // IIR second order

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
	struct IIRFILTERL iiradcvref; // Intermediate filter params: vref 
	struct IIRFILTERL iiradctemp; // Intermediate filter params: temperature sensor

	uint32_t adcfilvref;  // Filtered ADC[Vref]
	uint32_t adcfiltemp;  // Filtered ADC[temperature]

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

/* Working values for absolute voltages adjusted using Vref. */
struct ADCABSOLUTE
{
	struct IIRFILTERL iir;// Intermediate filter params
	double dscale;        // Computed from measurements
	double k;             // divider ratio: (Vref/adcvref)*(adcvx/Vx)
	uint32_t adcfil;      // Filtered ADC reading
	uint32_t ival;        // scaled int computed value (not divider scaled)
};

/* Working values for ratiometric sensors using 5v supply. */
struct ADCRATIOMETRIC
{
	struct IIRFILTERL iir;    // Intermediate filter params
	double drk5ke;    // Ratio k5/ke resistor dividers ratio (~1.0)
	double drko;      // Offset ratio: double (~0.5)
	double dscale;    // Scale factor
	uint32_t adcfil;  // Filtered ADC reading
	int32_t irk5ke;   // Ratio k5/ke ratio: scale int (~32768)
	int32_t irko;     // Offset ratio: scale int (~32768)
	int32_t iI;       // integer result w offset, not final scaling
};

struct ADCCHANNEL	
{
	double dscale;    // Reading: final scaling
	uint32_t ival;    // Reading: calibrated scaled int32_t
	uint16_t sum;     // Sum of 1/2 DMA buffer
	uint32_t xsum[2];    // Extended sum
	struct CICLN2M3 cic;
};

/* struct allows pointer to access raw and calibrated ADC1 data. */
struct ADCFUNCTION
{
	struct ADCCONTACTORLC lc;    // Local Copy of parameters
	struct ADCINTERNAL    intern;// Vref & temperature
	struct ADCABSOLUTE    v12;   // Supply: raw 12v
	struct ADCABSOLUTE    v5;    // Supply: regulated 5v
	struct ADCRATIOMETRIC cur1;  // Current sensor #1
   struct ADCRATIOMETRIC cur2;  // Current sensor #2
	struct ADCCHANNEL	 chan[ADC1IDX_ADCSCANSIZE]; // ADC sums, calibrated endpt
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
