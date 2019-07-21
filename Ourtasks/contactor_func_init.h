/******************************************************************************
* File Name          : contactor_func_init.h
* Date First Issued  : 07/14/2019
* Description        : hv uart conversion and recalibration
*******************************************************************************/

#ifndef __CONTACTORFUNCINIT
#define __CONTACTORFUNCINIT

#include "iir_filter_lx.h"
#include "adcparams.h"

/* *************************************************************************/
void contactor_func_init_init(struct CONTACTORFUNCTION* p, struct ADCFUNCTION* padc);
/*	@brief	: Initialize working struct for ContactorTask
 * @param	: p    = pointer to ContactorTask
 * @param	: padc = pointer to ADC working struct
 * *************************************************************************/

#endif

