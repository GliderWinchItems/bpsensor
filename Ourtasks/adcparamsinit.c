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
/* => Reproduced for convenience <= 
struct ADCFUNCTION
{
	struct ADCCONTACTORLC lc;     // Local Copy of parameters
	struct ADCINTERNAL    intern; // Vref & temperature
	struct ADCABSOLUTE    hv[4];  // High voltages
	struct ADCCHANNEL	 chan[ADC1IDX_ADCSCANSIZE]; // ADC sums
	uint32_t ctr; // Running count of updates.
	uint32_t idx_xsum;
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

/* => Reproduced for convenience <= 
struct ADCCALABS
{
	struct IIR_L_PARAM iir; // Filter: Time constant, integer scaling
	uint32_t adcvn;    // (ADC reading) vn 
   double   dvn;      // (double) measured vn (volts)
};

/* Absolute:  Battery string. */
	p->hv[0].iir.pprm = &p->lc.abs[0]iir; // Filter param pointer
	p->hv[0].k   = (p->lc.abs[0]dvn / p->intern.dvref) * (dadc / p->lc.abs[0]adcvn);
	p->chan[ADC1IDX_5VOLTSUPPLY].dscale = p->hv[0].dscale;

/* Absolute:  DMOC+ */
	p->hv[1].iir.pprm = &p->lc.abs[1]iir; // Filter param pointer
	p->hv[1].k   = (p->lc.abs[1]dvn / p->intern.dvref) * (dadc / p->lc.abs[1]adcvn);
	p->chan[ADC1IDX_5VOLTSUPPLY].dscale = p->hv[1].dscale;

/* Absolute:  DMOC- */
	p->hv[2].iir.pprm = &p->lc.abs[2]iir; // Filter param pointer
	p->hv[2].k   = (p->lc.abs[2]dvn / p->intern.dvref) * (dadc / p->lc.abs[2]adcvn);
	p->chan[ADC1IDX_5VOLTSUPPLY].dscale = p->hv[2].dscale;

/* Absolute:  spare */
	p->hv[3].iir.pprm = &p->lc.abs[3]iir; // Filter param pointer
	p->hv[3].k   = (p->lc.abs[3]dvn / p->intern.dvref) * (dadc / p->lc.abs[3]adcvn);
	p->chan[ADC1IDX_5VOLTSUPPLY].dscale = p->hv[3].dscale;

	return;
}
