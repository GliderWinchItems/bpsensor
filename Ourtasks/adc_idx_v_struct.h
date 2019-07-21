/******************************************************************************
* File Name          : adc_idx_v_struct.h
* Date First Issued  : 06/17/2019
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
#include "contactor_idx_v_struct.h"

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


/* 5v supply ratiometric calibration, e.g. Hall effect sensors. */
/*
Jumpering to 5v provides a measurement of the 5v->Vdd (3.3v) resistor
dividers to compute a ratio this adjusts the sensor readings for 
changes in the 5v sensor supply.

The sensor connected, with no current, provides the offset.  The 5v
adc reading may have changed from the jumpered reading.

Appling a known current and noting the adc reading calibrates the scale
for the sensor. The current direction that reduces the adc reading
from the no-current offset is negative.
*/
struct ADCCALHE
{
	struct IIR_L_PARAM iir; // Filter: Time constant, integer scaling
	double   scale;     // 
	uint32_t j5adcve;   // jumpered to 5v: adc reading HE input
	uint32_t j5adcv5;   // jumpered to 5v: adc reading 5v input
	uint32_t zeroadcve; // connected, no current: HE adc reading
	uint32_t zeroadc5;  // connected, no current: 5v adc reading 
	uint32_t caladcve;  // connected, cal current: adc reading
	double   dcalcur;   // connected, cal current: current
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
	struct ADCCALHE cal_cur1; // Hall-effect current calibration, battery string
	struct ADCCALHE cal_cur2; // Hall-effect current calibration, spare 
	struct ADCCALABS cal_5v;  // 5v regulated voltage 
	struct ADCCALABS cal_12v; // 12v raw CAN voltage
 };

/* **************************************************************************************/
int adc_idx_v_struct_hardcode_params(struct ADCCONTACTORLC* p);
/* @brief	: Hard-code load local copy with parameters
 * @return	: 0
 * ************************************************************************************** */
 
#endif

