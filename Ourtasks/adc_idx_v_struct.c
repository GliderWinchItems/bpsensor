/******************************************************************************
* File Name          : adc_idx_v_struct.c
* Date First Issued  : 06/17/2019
* Board              :
* Description        : Load sram local copy of parameters
*******************************************************************************/
#include "adc_idx_v_struct.h"

/* **************************************************************************************
 * int adc_idx_v_struct_hardcode_params(struct ADCCONTACTORLC* p);
 * @brief	: Hard-code load local copy with parameters
 * @return	: 0
 * ************************************************************************************** */
int adc_idx_v_struct_hardcode_params(struct ADCCONTACTORLC* p)
{
/* => Reproduced for convenience <= 
 struct ADCCONTACTORLC
 {
	uint32_t size;			// Number of items in struct
 	uint32_t crc;			// crc-32 placed by loader
	uint32_t version;		// struct version number
	struct ADC1CALINTERNAL calintern; // Vref and Temp internal sensors
	struct ADCCALHE cal_cur1; // Hall-effect current calibration, battery string
	struct ADCCALHE cal_cur2; // Hall-effect current calibration, spare 
	struct ADCCALABS cal_5v;  // 5v regulated voltage 
	struct ADCCALABS cal_12v; // 12v raw CAN voltage
 };
*/
	p->size     = 37; // Number of items in list
	p->crc      = 0;  // TODO
   p->version  = 1;
	p->hbct     = 1000;  // Time (ms) between HB msg

/* => Reproduced for convenience <= 
struct ADC1CALINTERNAL
{
	struct IIR_L_PARAM iiradcvref; // Filter: adc readings: Vref 
	struct IIR_L_PARAM iiradctemp; // Filter: adc readings: temperature
	uint32_t adcvdd;   // (ADC reading) for calibrating Vdd (3.3v)
	uint32_t adcrmtmp; // (ADC reading) room temperature reading
	uint32_t rmtmp;    // Room temp for reading (deg C)
	double dvdd;       // (double) measured Vdd (volts)
	double dslope;     // (double) mv/degC temperature sensor slope
};
*/
	p->calintern.iiradcvref.k     = 20;    // Filter time constant
	p->calintern.iiradcvref.scale = 64;

	p->calintern.iiradctemp.k     = 100;    // Filter time constant
	p->calintern.iiradctemp.scale = 4;

	// Internal voltage ref: ADC1IDX_INTERNALVREF  5   // IN18     - Internal voltage reference
	p->calintern.dvdd   = 3.29;	// Vdd for following Vref ADC reading
	p->calintern.adcvdd = 23928;  //(16*1495.5) ADC reading (DMA sum) for above Vdd

	// Internal temperature: ADC1IDX_INTERNALTEMP  4   // IN17     - Internal temperature sensor
	p->calintern.adcrmtmp  = 26990; // Room temp ADC (DMA sum) reading
	p->calintern.drmtemp   = 25.0;  // Room temp for ADC reading     
	p->calintern.dslope    = 4.3;   // mv/degC slope of temperature sensor
	p->calintern.dvreftmpco= 15;    // Vref temp coefficient (15 is based on similar parts)
	p->calintern.dvtemp    = 1.40;  // Vtemp voltage at 25 degC


/*  Reproduced for convenience 
struct ADCCALABS
{
	struct IIR_L_PARAM iir; // Filter: Time constant, integer scaling
	uint32_t adcvn;    // (ADC reading) vn 
   double   dvn;      // (double) measured vn (volts)
};
*/
	/* High voltage dividers and filter constants. */

	// Nominal dividers: 2.2M | 15K
	// HV1 Battery string
	p->abs[0].iir.k     = 10;    // Filter time constant
	p->abs[0].iir.scale = 2;     // Filter integer scaling
	p->abs[0].adcvn     = 65520; // hv1: ADC reading
	p->abs[0].dvn       = 487.3; // hv1: measured
	// HV2 DMOC+
	p->abs[1].iir.k     = 10;    // Filter time constant
	p->abs[1].iir.scale = 2;     // Filter integer scaling
	p->abs[1].adcvn     = 65520; // hv2: ADC reading
	p->abs[1].dvn       = 487.3; // hv2: measured
	// HV3 DMOC-
	p->abs[2].iir.k     = 10;    // Filter time constant
	p->abs[2].iir.scale = 2;     // Filter integer scaling
	p->abs[2].adcvn     = 65520; // hv3: ADC reading
	p->abs[2].dvn       = 487.3; // hv3: measured
	// HV4 spare
	p->abs[3].iir.k     = 10;    // Filter time constant
	p->abs[3].iir.scale = 2;     // Filter integer scaling
	p->abs[3].adcvn     = 65520; // hv4: ADC reading
	p->abs[3].dvn       = 487.3; // hv4: measured

	return 0;	
}
