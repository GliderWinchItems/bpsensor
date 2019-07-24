/******************************************************************************
* File Name          : bpsensor_idx_v_struct.c
* Date First Issued  : 06/26/2019
* Board              :
* Description        : Load parameter struct 
*******************************************************************************/

#include "bpsensor_idx_v_struct.h"
#include "SerialTaskReceive.h"


/* *************************************************************************
 * void bpsensor_idx_v_struct_hardcode_params(struct struct CONTACTORLC* p);
 * @brief	: Init struct from hard-coded parameters (rather than database params in highflash)
 * @return	: 0
 * *************************************************************************/
void bpsensor_idx_v_struct_hardcode_params(struct CONTACTORLC* p)
{ /*
Copied for convenience--

struct CONTACTORLC
 {
 };

*/
	p->size       = 30;
	p->crc        = 0;
   p->version    = 1;

	/* Bits that define the hw features. */
	p->hwconfig   = 0;  // None of the additional hw features!




	// Battery_minus-to-contactor #1
	p->calhv[IDXHV1].iir.k     = 3;
	p->calhv[IDXHV1].iir.scale = 2;
 	p->calhv[IDXHV1].adchv  = 420.0; // Applied voltage
	p->calhv[IDXHV1].offset = 53761; // ADC reading (received from uart)
	p->calhv[IDXHV1].dvcal  = p->calhv[IDXHV1].adchv / p->calhv[IDXHV1].offset; // volts/per ADCtick

	// Battery_minus-to-contactor #1 DMOC_plus
	p->calhv[IDXHV2].iir.k     = 3;
	p->calhv[IDXHV2].iir.scale = 2;
 	p->calhv[IDXHV2].adchv  = 420.0; // Applied voltage
	p->calhv[IDXHV2].offset = 53761; // ADC reading (received from uart)
	p->calhv[IDXHV2].dvcal  = p->calhv[IDXHV2].adchv / p->calhv[IDXHV2].offset; // volts/per ADCtick

	// Battery_minus-to-contactor #1 DMOC_minus
	p->calhv[IDXHV3].iir.k     = 3;
	p->calhv[IDXHV3].iir.scale = 2;
 	p->calhv[IDXHV3].adchv  = 420.0; // Applied voltage
	p->calhv[IDXHV3].offset = 53761; // ADC reading (received from uart)
	p->calhv[IDXHV3].dvcal  = p->calhv[IDXHV3].adchv / p->calhv[IDXHV3].offset; // volts/per ADCtick


	return;
}
