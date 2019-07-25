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

 ADC reading sequence/array indices                         
 These indices -=>MUST<= match the hardware ADC scan sequence    *
#define ADC1IDX_HIGHVOLT1     0   // PA1 IN1  - Battery voltage
#define ADC1IDX_HIGHVOLT2     1   // PA2 IN2  - DMOC +
#define ADC1IDX_HIGHVOLT3     2   // PA3 IN3  - DMOC -
#define ADC1IDX_HIGHVOLT4     3   // PA4 IN4  - spare
#define ADC1IDX_INTERNALTEMP  4   // IN17     - Internal temperature sensor
#define ADC1IDX_INTERNALVREF  5   // IN18     - Internal voltage reference
*/

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
	p->chan[ADC1IDX_INTERNALVREF].iir.pprm = &p->lc.calintern.iiradcvref;
	p->chan[ADC1IDX_INTERNALTEMP].iir.pprm = &p->lc.calintern.iiradctemp;

	// Compute a scaled integer vref from measurements
	double dadcvdd     = p->lc.calintern.adcvdd; // ADC reading (~27360)
	p->intern.dvref = p->lc.calintern.dvdd * (dadcvdd / 65520.0);
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
*/
/* Absolute:  Battery string. */
	p->chan[ADC1IDX_HIGHVOLT1].iir.pprm = &p->lc.abs[0].iir; // Filter param pointer
	p->chan[ADC1IDX_HIGHVOLT1].k   = (p->lc.abs[0].dvn / p->intern.dvref) * (dadcvdd / p->lc.abs[0].adcvn);
	p->chan[ADC1IDX_HIGHVOLT1].dscale   = p->chan[ADC1IDX_HIGHVOLT1].dscale;

/* Absolute:  DMOC+ */
	p->chan[ADC1IDX_HIGHVOLT2].iir.pprm = &p->lc.abs[1].iir; // Filter param pointer
	p->chan[ADC1IDX_HIGHVOLT2].k   = (p->lc.abs[1].dvn / p->intern.dvref) * (dadcvdd / p->lc.abs[1].adcvn);
	p->chan[ADC1IDX_HIGHVOLT2].dscale   = p->chan[ADC1IDX_HIGHVOLT2].dscale;

/* Absolute:  DMOC- */
	p->chan[ADC1IDX_HIGHVOLT3].iir.pprm = &p->lc.abs[2].iir; // Filter param pointer
	p->chan[ADC1IDX_HIGHVOLT3].k   = (p->lc.abs[2].dvn / p->intern.dvref) * (dadcvdd / p->lc.abs[2].adcvn);
	p->chan[ADC1IDX_HIGHVOLT3].dscale   = p->chan[ADC1IDX_HIGHVOLT3].dscale;

/* Absolute:  spare */
	p->chan[ADC1IDX_HIGHVOLT4].iir.pprm = &p->lc.abs[3].iir; // Filter param pointer
	p->chan[ADC1IDX_HIGHVOLT4].k   = (p->lc.abs[3].dvn / p->intern.dvref) * (dadcvdd / p->lc.abs[3].adcvn);
	p->chan[ADC1IDX_HIGHVOLT4].dscale   = p->chan[ADC1IDX_HIGHVOLT4].dscale;

	return;
}
