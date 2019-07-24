/******************************************************************************
* File Name          : adc_idx_v_struct.h
* Date First Issued  : 07/22/2019
* Board              :
* Description        : Translate parameter index into pointer into struct
*******************************************************************************/
/*
                     Min  Typ  Max 
Internal reference: 1.16 1.20 1.24 V
Vref temperautre co.  --   --  100 ppm/DegC

Temperature sensor specs
                 Min  Typ  Max
Average slope    4.0  4.3  4.6 mV/°C
Voltage at 25 °C 1.34 1.43 1.52 

*/
#include <stdint.h>
#include "common_can.h"
#include "iir_filter_lx.h"

#ifndef __ADC_IDX_V_STRUCT
#define __ADC_IDX_V_STRUCT

/* Parameters for ADC reading */
// Flointing pt params are converted to scaled integers during initialization


/* Internal sensor calibration. (Only applies to ADC1) */
struct ADC1CALINTERNAL
{
	struct IIR_L_PARAM iiradcvref; // Filter: adc readings: Vref 
	struct IIR_L_PARAM iiradctemp; // Filter: adc readings: temperature
	double drmtemp;    // (double) Room temp for reading (deg C)
	double dvtemp;     // (double) Voltage of temp sensor at rm temperature
	double dvdd;       // (double) measured Vdd (volts)
	double dslope;     // (double) mv/degC temperature sensor slope
	double dvreftmpco; // (double) Vref temperature coefficient (ppm/degC)
	uint32_t adcvdd;   // (ADC reading) for calibrating Vdd (3.3v)
	uint32_t adcrmtmp; // (ADC reading) room temperature temp sensor reading
};

/* Absolute (non-ratiometric) sensor calibration. */
/*
The calibrated results are adjusted for Vdd variations by using the
internal voltage reference, and the internal voltage reference is
adjusted for temperature by using the internal temperature reference.
*/
struct ADCCALABS
{
	struct IIR_L_PARAM iir; // Filter: Time constant, integer scaling
	uint32_t adcvn;    // (ADC reading) vn 
   double   dvn;      // (double) measured vn (volts)
};

/* Parameters for ADC. */
// LC = Local (sram) Copy of parameters
 struct ADCCONTACTORLC
 {
	uint32_t size;			// Number of items in struct
 	uint32_t crc;			// crc-32 placed by loader
	uint32_t version;		// struct version number
	uint32_t hbct;       // heartbeat count (ms)
	struct ADC1CALINTERNAL calintern; // Vref and Temp internal sensors
	struct ADCCALABS abs[4];  // High voltages
 };

/* **************************************************************************************/
int adc_idx_v_struct_hardcode_params(struct ADCCONTACTORLC* p);
/* @brief	: Hard-code load local copy with parameters
 * @return	: 0
 * ************************************************************************************** */
 
#endif

