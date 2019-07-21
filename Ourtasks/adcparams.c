/******************************************************************************
* File Name          : adcparams.c
* Date First Issued  : 03/09/2019
* Board              : DiscoveryF4
* Description        : Parameters for ADC app configuration
*******************************************************************************/
/*
Not thread safe.
*/
#include "adcparams.h"
#include "adcparamsinit.h"
#include "ADCTask.h"

#include "DTW_counter.h"

/*
  SCALING DETAIL

ADC readings are referred in the 1/2 DMA sum, i.e. the sum of 16 ADC readings.
The max sum is therefore, 65520 (16 * 4095).  

Vn = Vref * (ADC[n]/ADC[vref]) * ((R1+R2)/R2);
  Where: ((R1+R2)/R2) is resistor divider scale factor 

The resistor scale factor is applied when sending out readings for human consumption.

Since all values are positive, scaled uint32_t is used.  Ranges encounted--
 (1.20 * 65536) < Vref      < (1.27 * 65536) 
          39321 < Vref      < 41615
              0 < ADC[n]    < 65520
          22464 < ADC[Vref] < 29718
Max ADCVref = 65520 * 1.20v/Vdd 3.5v = 22464
Min ADCVref = 65520 * 1.27v/Vdd 2.8v = 29718


 Vref TEMPERATURE COMPENSATION

Temp(degree) = (V_sense - V_25)/Avg_slope + 25

                 Min  Typ  Max
Average slope    4.0  4.3  4.6  mV/°C
Voltage at 25 °C 1.34 1.43 1.52 V

ADC sampling time when reading the
temperature - - 17.1 μs

V 25 = V SENSE value for 25° C and
Avg_Slope = Average Slope for curve between Temperature vs. V SENSE (given in
   mV/° C or μV/ °C).
*/

/* Everything for ADC1. */
struct ADCFUNCTION adc1;

/* *************************************************************************
 * void adcparams_init(void);
 *	@brief	: Copy parameters into structs
 * NOTE: => ASSUMES ADC1 ONLY <==
 * *************************************************************************/
void adcparams_init(void)
{
	/* Load parameters, either hard coded, (or later implement from high flash). */
	adc_idx_v_struct_hardcode_params(&adc1.lc);

	/* Init working struct for ADC function. */
	adcparamsinit_init(&adc1);

	return;
}

/* *************************************************************************
 * static void internal(struct ADCFUNCTION* p, struct ADCINTERNAL* pi,uin8_t idx);
 *	@brief	: Update values used for compensation from Vref and Temperature
 * @param	: p = Pointer to array of ADC reading sums plus other stuff
 * @param	: idx = index into ADC sum for sensor
 * *************************************************************************/
uint32_t adcdbg1;
uint32_t adcdbg2;

static void internal(struct ADCFUNCTION* p)
{
	int32_t itmp;
/* 

Obtain the temperature using the following formula:
  Temperature (in °C) = {(V 25 - V SENSE ) / Avg_Slope} + 25.
Where,
   V 25 = V SENSE value for 25° C and
   Avg_Slope = Average Slope for curve between Temperature vs. V SENSE (given in
       mV/° C or μV/ °C).
Refer to the Electrical characteristics section for the actual values of V 25 and Avg_Slope

Average slope     4.0  4.3  4.6  mV/°C
Voltage at 25 °C  1.34 1.43 1.52 V

*/
	/* IIR filter internal adc sensor readings. */
	p->intern.adcfiltemp = iir_filter_lx_do(&p->intern.iiradctemp, &p->chan[ADC1IDX_INTERNALTEMP].sum);
	p->intern.adcfilvref = iir_filter_lx_do(&p->intern.iiradcvref, &p->chan[ADC1IDX_INTERNALVREF].sum);

	/* Skip temperature compensation for now. */
	p->intern.adccmpvref = p->intern.adcfilvref;

adcdbg1 = DTWTIME;
	/* Compute temperature */
	itmp = (p->intern.iv25 * p->intern.adcfilvref) - ((p->intern.vref * p->intern.adcfiltemp) >> (ADCSCALEbits-ADCSCALEbitsy));

	itmp = ((itmp >> ADCSCALEbitsitmp) * p->intern.yRs) / p->intern.adcfilvref;

	p->intern.itemp = (itmp << ADCSCALEbitsitmp) + p->intern.irmtemp;
adcdbg2 = DTWTIME - adcdbg1;

	return;
}
/* *************************************************************************
 * static void absolute(struct ADCFUNCTION* p, struct ADCABSOLUTE* pa,uint8_t idx);
 *	@brief	: Calibrate and filter absolute voltage readings
 * @param	: p = Pointer to array of ADC reading sums plus other stuff
 * @param	: pa = Pointer to absolute parameters for reading 'n'
 * @param	: idx = index into ADC sum for sensor for reading 'n'
 * *************************************************************************/
/*
Vn = Vref * (ADC[n]/ADC[vref]) * ((R1+R2)/R2);
  Where: ((R1+R2)/R2) is resistor divider scale factor 
*/
static void absolute(struct ADCFUNCTION* p, struct ADCABSOLUTE* pa,uint8_t idx)
{
	/* IIR filter adc reading. */
	pa->adcfil = iir_filter_lx_do(&pa->iir, &p->chan[idx].sum);

	pa->ival = (p->intern.vref * pa->adcfil) / p->intern.adccmpvref;
	return;
}
/* *************************************************************************
 * static void ratiometric5v(struct ADCFUNCTION* p, struct ADCRATIOMETRIC* pr,uint8_t idx);
 *	@brief	: Calibrate and filter 5v ratiometric (e.g. Hall-effect sensor) reading
 * @param	: p = Pointer to array of ADC reading sums plus other stuff
 * @param	: pr = Pointer to ratiometric working vars (within p->)
 * @param	: idx = index into ADC sum for sensor
 * *************************************************************************/
/*
Vr = (k5/ke) * ((ADC[ke]/ADC[k5]) - koffset) * scale
  scale is only applied for when used for human consumption
	

 Computation scaling--
-  K5 and Ke resistor dividers will be have a division ratio that is nearly
  the same. Therefore, assume k5/ke is less than 2

-  scale k5/ke by 2^15 so max less than 65536,

-  ADC[ke]/ADC[k5] is always less than 1,
   therefore,
    (k5/ke) * (ADC[ke]/ADC[k5]) is always less than 2^32

*/
static void ratiometric5v(struct ADCFUNCTION* p, struct ADCRATIOMETRIC* pr, uint8_t idx)
{
/* NOTE: Ratiometric is based on the ratio of the reading of the 5v supply 
   powering the sensor and the sensor reading.  The ratio is adjusted to 
   account for the differences in the resistor divider ratio for both
   inputs.

	The originating offset parameter in the 'lc struct is converted to a
   2^16 scaled fraction during the initialization of parameters.  Therefore,
   the offset is typically, either zero (sensor is positive going only), to
   0.5 (therefore 32767) when the offset is at 1/2 Vsensor (2.5v).

	The dividers for sensor and 5v supply are approximately equal, but the ratio
   is calibrated.  The ratio is therefore close to 1.0.
*/
	/* IIR filter adc reading. */
	pr->adcfil = iir_filter_lx_do(&pr->iir, &p->chan[idx].sum);

	/* Compute ratio of sensor reading to 5v supply reading. */
	uint64_t adcke = (pr->adcfil << ADCSCALEbits); // Scale before divide
	uint64_t adcratio64 = adcke / p->chan[ADC1IDX_5VOLTSUPPLY].sum;
	uint32_t adcratio = (adcratio64 >> ADCSCALEbits);

	/* Subtract offset (note result is now signed). */
	int32_t tmp = (adcratio - pr->irko); 

	/* Apply adjustment for unequal resistor dividers. */
	int64_t tmp64 = (pr->irk5ke * tmp);
	pr->iI = (tmp64 >> ADCSCALEbits);

	return;
}

/* *************************************************************************
 * void adcparams_cal(void);
 *	@brief	: calibrate and filter ADC readings
 * *************************************************************************/
void adcparams_cal(void)
{
	struct ADCFUNCTION* p = &adc1; // Convenience pointer

	/* First: Update Vref used in susequent computations. */
	internal(p); // Update Vref for temperature

	absolute(p, &p->v5 ,ADC1IDX_5VOLTSUPPLY); // 5v supplying Blue Pill, and sensors.
  
	absolute(p, &p->v12 ,ADC1IDX_12VRAWSUPPLY); // Raw CAN bus supply to board

/* Note: 5v supply should be processed before ratiometrics.  Otherwise,
   old readings will be used which is not a big deal for a slowly 
   changing 5v supply. */

	ratiometric5v(p, &p->cur1, ADC1IDX_CURRENTTOTAL); // Battery string sensor

	ratiometric5v(p, &p->cur2, ADC1IDX_CURRENTMOTOR); // Spare, or motor sensor

	return;
}
