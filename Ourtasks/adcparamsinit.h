/******************************************************************************
* File Name          : adcparamsinit.h
* Date First Issued  : 03/09/2019
* Board              : DiscoveryF4
* Description        : Initialization of parameters for ADC app configuration
*******************************************************************************/

#ifndef __ADCPARAMSINIT
#define __ADCPARAMSINIT

#include <stdint.h>
#include "adcparams.h"

/* *************************************************************************/
void adcparamsinit_init(struct ADCFUNCTION* p);
/*	@brief	: Load structs for compensation, calibration and filtering all ADC channels
 * @param	: p = Pointer to struct "everything" for this ADC module
 * *************************************************************************/

#endif

