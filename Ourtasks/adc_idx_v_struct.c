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
/* Copy for convenience--
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

/* Reproduced for convenience 
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
struct ADCCALHE
{
	struct IIR_L_PARAM iir; // Filter: Time constant, integer scaling
	double   scale;     // Resistor ratio to scale to desired units
	uint32_t j5adcve;   // jumpered to 5v: adc reading HE input
	uint32_t j5adcv5;   // jumpered to 5v: adc reading 5v input
	uint32_t zeroadcve; // connected, no current: HE adc reading
	uint32_t zeroadc5;  // connected, no current: 5v adc reading 
	uint32_t caladcve;  // connected, cal current: adc reading
	double   dcalcur;   // connected, cal current: current
};
*/
	// Battery current: ADC1IDX_CURRENTTOTAL  1   // PA5 IN5  - Current sensor: total battery current
	p->cal_cur1.iir.k     = 10;    // Filter time constant
	p->cal_cur1.iir.scale = 2;     // Filter integer scaling
	p->cal_cur1.j5adcve   = 59597; // jumpered to 5v: adc reading HE input
	p->cal_cur1.j5adcv5   = 59597; // jumpered to 5v: adc reading 5v input
	p->cal_cur1.zeroadcve = 29798; // connected, no current: HE adc reading
	p->cal_cur1.zeroadc5  = 59597; // connected, no current: 5v adc reading 
	p->cal_cur1.caladcve  = 30394; // connected, cal current: adc reading (400a unit)
	p->cal_cur1.dcalcur   = 10.0;  // connected, cal current: current (400a unit @ 10a)

	// Spare current: ADC1IDX_CURRENTMOTOR  2   // PA6 IN6  - Current sensor: motor
	p->cal_cur2.iir.k     = 10;    // Filter time constant
	p->cal_cur2.iir.scale = 2;     // Filter integer scaling
	p->cal_cur2.j5adcve   = 59597; // jumpered to 5v: adc reading HE input
	p->cal_cur2.j5adcv5   = 59597; // jumpered to 5v: adc reading 5v input
	p->cal_cur2.zeroadcve = 29798; // connected, no current: HE adc reading
	p->cal_cur2.zeroadc5  = 29798; // connected, no current: 5v adc reading 
	p->cal_cur2.caladcve  = 30394; // connected, cal current: adc reading (400a unit)
	p->cal_cur2.dcalcur   = 10.0;  // connected, cal current: current (400a unit @ 10a)

/*  Reproduced for convenience 
struct ADCCALABS
{
	struct IIR_L_PARAM iir; // Filter: Time constant, integer scaling
	uint32_t adcvn;    // (ADC reading) vn 
   double   dvn;      // (double) measured vn (volts)
};
*/
	// 5v supply: ADC1IDX_5VOLTSUPPLY   0   // PA0 IN0  - 5V sensor supply
	p->cal_5v.iir.k     = 10;    // Filter time constant
	p->cal_5v.iir.scale = 2;     // Filter integer scaling
	p->cal_5v.adcvn     = 64480; // (ADC reading) v5
	p->cal_5v.dvn       = 5.03;  // (double) measured v5 (volts)

	// Raw 12v CAN bus supply: ADC1IDX_12VRAWSUPPLY  3   // PA7 IN7  - +12 Raw power to board
	p->cal_12v.iir.k     = 10;    // Filter time constant
	p->cal_12v.iir.scale = 2;     // Filter integer scaling
	p->cal_12v.adcvn     = 24023; // (4095*1502); // (ADC reading) v12 
	p->cal_12v.dvn       = 13.68;  // (double) measured v12 (volts)

	return 0;	
}
