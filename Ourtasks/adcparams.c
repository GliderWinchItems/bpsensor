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
	/* Skip temperature compensation for now. */
	p->intern.adccmpvref = p->chan[ADC1IDX_INTERNALVREF].adcfil;

	/* Compute temperature */
	itmp = (p->intern.iv25 * p->chan[ADC1IDX_INTERNALVREF].adcfil) - ((p->intern.vref * p->chan[ADC1IDX_INTERNALTEMP].adcfil) >> (ADCSCALEbits-ADCSCALEbitsy));

	itmp = ((itmp >> ADCSCALEbitsitmp) * p->intern.yRs) / p->chan[ADC1IDX_INTERNALVREF].adcfil;

	p->intern.itemp = (itmp << ADCSCALEbitsitmp) + p->intern.irmtemp;

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
static void absolute(struct ADCFUNCTION* p,uint8_t idx)
{
	/* IIR filter adc reading. */
	p->chan[idx].adcfil = iir_filter_lx_do(&p->chan[idx].iir, &p->chan[idx].sum);

	p->chan[idx].ival = (p->intern.vref * p->chan[idx].adcfil) / p->intern.adccmpvref;
	return;
}
/* *************************************************************************
 * void adcparams_cal(void);
 *	@brief	: calibrate and filter ADC readings
 * *************************************************************************/
void adcparams_cal(void)
{
	struct ADCFUNCTION* p = &adc1; // Convenience pointer

	/* Run each ADC sum through iir filter */
	// 0 PA1 IN1-Battery voltage
	p->chan[ADC1IDX_HIGHVOLT1].adcfil = iir_filter_lx_do(&p->chan[ADC1IDX_HIGHVOLT1].iir, &p->chan[ADC1IDX_HIGHVOLT1].sum);
	// 1 PA2 IN2-DMOC +
	p->chan[ADC1IDX_HIGHVOLT2].adcfil =	iir_filter_lx_do(&p->chan[ADC1IDX_HIGHVOLT2].iir, &p->chan[ADC1IDX_HIGHVOLT2].sum);
	// 2 PA3 IN3-DMOC -
	p->chan[ADC1IDX_HIGHVOLT3].adcfil =	iir_filter_lx_do(&p->chan[ADC1IDX_HIGHVOLT3].iir, &p->chan[ADC1IDX_HIGHVOLT3].sum);
	// 3 PA4 IN4-spare
	p->chan[ADC1IDX_HIGHVOLT4].adcfil =	iir_filter_lx_do(&p->chan[ADC1IDX_HIGHVOLT4].iir, &p->chan[ADC1IDX_HIGHVOLT4].sum);
	// 4 IN17-Internal temperature sensor
	p->chan[ADC1IDX_INTERNALTEMP].adcfil =	iir_filter_lx_do(&p->chan[ADC1IDX_INTERNALTEMP].iir, &p->chan[ADC1IDX_INTERNALTEMP].sum);
	// 5 IN18-Internal voltage reference
	p->chan[ADC1IDX_INTERNALVREF].adcfil =	iir_filter_lx_do(&p->chan[ADC1IDX_INTERNALVREF].iir, &p->chan[ADC1IDX_INTERNALVREF].sum);

	/* First: Update ADCvref used in subsequent computations. */
	internal(p); // Update Vref for temperature

   /* Compute high voltages, without resistor divider applied
	   With Vref = 1.200 the max input will yield 54065 which fits uint16_t.
      unit16_t is sent out on usart3.  The receiving end applies
      the resistor divider scaling. */
	absolute(p, ADC1IDX_HIGHVOLT1); // 
	absolute(p, ADC1IDX_HIGHVOLT2); // 
	absolute(p, ADC1IDX_HIGHVOLT3); // 
	absolute(p, ADC1IDX_HIGHVOLT4); // 
  
	return;
}
