/******************************************************************************
* File Name          : adcparamsinit.c
* Date First Issued  : 03/09/2019
* Board              : DiscoveryF4
* Description        : Initialization of parameters for ADC app configuration
*******************************************************************************/
/* 
This is where hard-coded parameters for the ADC are entered.

Later, this may be replaced with a "copy" of the flat file in high flash, generated
by the java program from the sql database.
*/
#include "adcparamsinit.h"
#include "adcparams.h"
#include "ADCTask.h"
#include "morse.h"

/*                        Min  Typ  Max 
Internal reference F103: 1.16 1.20 1.26 V 
Internal reference F407: 1.18 1.20 1.24 V 
*/
// Define limits for initialization check
#define VREFMIN (1.15)
#define VREFMAX (1.27)

/* *************************************************************************
void adcparamsinit_init(struct ADCFUNCTION* p);
 *	@brief	: Load structs for compensation, calibration and filtering all ADC channels
 * @param	: p = Pointer to struct "everything" for this ADC module
 * *************************************************************************/

/* Reproduced for convenience

#define ADC1IDX_5VOLTSUPPLY   0   // PA0 IN0  - 5V sensor supply
#define ADC1IDX_CURRENTTOTAL  1   // PA5 IN5  - Current sensor: total battery current
#define ADC1IDX_CURRENTMOTOR1 2   // PA6 IN6  - Current sensor: motor #1
#define ADC1IDX_12VRAWSUPPLY  3   // PA7 IN7  - +12 Raw power to board
#define ADC1IDX_INTERNALTEMP  4   // IN17     - Internal temperature sensor
#define ADC1IDX_INTERNALVREF  5   // IN18     - Internal voltage reference

#define ADC1PARAM_COMPTYPE_NONE      0     // No supply or temp compensation applied
#define ADC1PARAM_COMPTYPE_RATIOVDD  1     // Vdd (3.3v nominal) ratiometric
#define ADC1PARAM_COMPTYPE_RATIO5V   2     // 5v ratiometric with 5->Vdd measurement
#define ADC1PARAM_COMPTYPE_RATIO5VNO 3     // 5v ratiometric without 5->Vdd measurement
#define ADC1PARAM_COMPTYPE_VOLTVDD   4     // Vdd (absolute), Vref compensation applied
#define ADC1PARAM_COMPTYPE_VOLTVDDNO 5     // Vdd (absolute), no Vref compensation applied
#define ADC1PARAM_COMPTYPE_VOLTV5    6     // 5v (absolute), with 5->Vdd measurement applied
#define ADC1PARAM_COMPTYPE_VOLTV5NO  7     // 5v (absolute), without 5->Vdd measurement applied

#define ADC1PARAM_CALIBTYPE_RAW_F  0    // No calibration applied: FLOAT
#define ADC1PARAM_CALIBTYPE_OFSC   1    // Offset & scale (poly ord 0 & 1): FLOAT
#define ADC1PARAM_CALIBTYPE_POLY2  2    // Polynomial 2nd ord: FLOAT
#define ADC1PARAM_CALIBTYPE_POLY3  3    // Polynomial 3nd ord: FLOAT
#define ADC1PARAM_CALIBTYPE_RAW_UI 4    // No calibration applied: UNSIGNED INT
*/
double adcdtmp;

void adcparamsinit_init(struct ADCFUNCTION* p)
{
/* Reproduced for convenience 
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
};
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
	uint32_t iv25s;      // (scaled) (V25 * iRslope)
	double   V25;        // (double) Computed V25 (no)
	uint32_t vrefRs;     // (scaled) Vref / slope
	uint32_t irmtemp;    // (scaled) calibration temperature
	uint32_t itemp;      // (scaled) temperature (degC)
};
*/

/* Internal sensors. */
	// Pointers to filter constants 
	p->intern.iiradcvref.pprm = &p->lc.calintern.iiradcvref;
	p->intern.iiradctemp.pprm = &p->lc.calintern.iiradctemp;

	// Compute a scaled integer vref from measurements
	double dadc  = p->lc.calintern.adcvdd; // ADC reading (~27360)
	p->intern.dvref = p->lc.calintern.dvdd * (dadc / 65520.0);
	p->intern.vref  = (p->intern.dvref * (1 << ADCSCALEbits) ); // Scaled uint32_t; 
	p->intern.adcvref = (65520.0 * p->intern.dvref) / p->lc.calintern.dvdd;

	// Check for out-of-datasheet Vref spec 
	if ((p->intern.dvref < (VREFMIN)) || (p->intern.dvref > (VREFMAX))) 
	{
		morse_trap(81);
	}
	p->chan[ADC1IDX_INTERNALTEMP].dscale = p->lc.calintern.dvdd / 65520.0;
	p->chan[ADC1IDX_INTERNALVREF].dscale = 1.0;

	// Reciprocal of temperature sensor slope ( ~65536/4.3E-3 = (232.55 << 16) )
	p->intern.iRslope  = (double)((1 << ADCSCALEbits) * (1000)) / p->lc.calintern.dslope;

	// Pre-compute the (V25 / slope) into a scaled integer.
//	double dtmp = ((1 << ADCSCALEbits) * (1000)) * p->lc.calintern.dvtemp / p->lc.calintern.dslope;
//	p->intern.iv25s = dtmp;
//adcdtmp = dtmp;

	// Pre-compute (Vref / slope) to scaled int (~65536*1.20/4.3E-3 = (279.07 << 16) = 18289116)
//	dtmp = ((1 << ADCSCALEbits) * (1000)) * p->intern.dvref / p->lc.calintern.dslope;
//	p->intern.vrefRs = dtmp;
//adcdtmp = dtmp;

	// Room temp calibration offset (7/17/19) Is this needed?)
	p->intern.irmtemp = ((double)(1 << ADCSCALEbits) * (double)(p->lc.calintern.drmtemp));// / p->lc.calintern.adcrmtmp);
	
	p->intern.yRs = ((double)((1 << ADCSCALEbitsy) * 1000) / p->lc.calintern.dslope);

	p->intern.iv25 = (uint32_t)((double)(1 << ADCSCALEbitsy) * p->lc.calintern.dvtemp);

/* Reproduced for convenience
struct ADCABSOLUTE
{
	struct IIRFILTERL iir;// Intermediate filter params
	double dscale;        // Computed from measurements
	uint32_t adcfil;      // Filtered ADC reading
	uint32_t ival;        // scaled int computed value (not divider scaled)
}; */	

/* Absolute: 12v supply. */
	p->v12.iir.pprm = &p->lc.cal_12v.iir; // Filter param pointer
	p->v12.k   = (p->lc.cal_12v.dvn / p->intern.dvref) * (dadc / p->lc.cal_12v.adcvn);
	p->chan[ADC1IDX_12VRAWSUPPLY].dscale = p->v12.dscale;

/* Absolute:  5v supply. */
	p->v5.iir.pprm = &p->lc.cal_5v.iir; // Filter param pointer
	p->v5.k   = (p->lc.cal_5v.dvn / p->intern.dvref) * (dadc / p->lc.cal_5v.adcvn);
	p->chan[ADC1IDX_5VOLTSUPPLY].dscale = p->v5.dscale;



/* Reproduced for convenience
struct ADCRATIOMETRIC
{
	struct IIRFILTERL iir;    // Intermediate filter params
	double drk5ke;    // Ratio k5/ke resistor dividers ratio (~1.0)
	double drko;      // Offset ratio: double (~0.5)
	double dscale;    // Scale factor
	uint32_t adcfil;  // Filtered ADC reading
	int32_t irk5ke;   // Ratio k5/ke ratio: scale int (~32768)
	int32_t irko;     // Offset ratio: scale int (~32768)
	int32_t iI;       // integer result w offset, not scaled 
}; */

/* Ratiometric: battery string current. */
	p->cur1.iir.pprm = &p->lc.cal_cur1.iir; // Filter param pointer
	
	// Jumpered readings -> resistor divider ratio (~ 1.00)
	p->cur1.drk5ke = (double)p->lc.cal_cur1.j5adcv5 / (double)p->lc.cal_cur1.j5adcve;
	p->cur1.irk5ke *= (p->cur1.drk5ke * (1 << ADCSCALEbits) );

	// Sensor connected, no current -> offset ratio (~ 0.50)
	p->cur1.drko  = (double)p->lc.cal_cur1.zeroadcve / (double)p->lc.cal_cur1.zeroadc5;
	p->cur1.irko *= (p->cur1.drko * (1 << ADCSCALEbits) );

	// Sensor connected, test current applied -> scale factor
	// dscale = (calibration ADC - offset) / calibration current; // adcticks/amp
	p->cur1.dscale = (p->lc.cal_cur1.caladcve - p->lc.cal_cur1.zeroadcve) / p->lc.cal_cur1.dcalcur;
	p->chan[ADC1IDX_CURRENTTOTAL].dscale = p->cur1.dscale; // For convenient access

/* Ratiometric: spare current. */
	p->cur2.iir.pprm = &p->lc.cal_cur2.iir; // Filter param pointer

	// Jumpered readings -> resistor divider ratio (~ 1.00)
	p->cur2.drk5ke = (double)p->lc.cal_cur2.j5adcv5 / (double)p->lc.cal_cur2.j5adcve;
	p->cur2.irk5ke *= (p->cur2.drk5ke * (1 << ADCSCALEbits) );

	// Sensor connected, no current -> offset ratio (~ 0.50)
	p->cur2.drko  = (double)p->lc.cal_cur2.zeroadcve / (double)p->lc.cal_cur2.zeroadc5;
	p->cur2.irko *= (p->cur2.drko * (1 << ADCSCALEbits) );

	// Sensor connected, test current applied -> scale factor
	// dscale = (calibration ADC - offset) / calibration current; // adcticks/amp
	p->cur2.dscale = (p->lc.cal_cur2.caladcve - p->lc.cal_cur2.zeroadcve) / p->lc.cal_cur2.dcalcur;
	p->chan[ADC1IDX_CURRENTMOTOR].dscale = p->cur2.dscale; // For convenient access
	
	return;
}
