/******************************************************************************
* File Name          : bpsensor_idx_v_struct.c
* Date First Issued  : 06/26/2019
* Board              :
* Description        : Load parameter struct 
*******************************************************************************/

#include "bpsensor_idx_v_struct.h"
#include "SerialTaskReceive.h"


/* *************************************************************************
 * void bpsensor_idx_v_struct_hardcode_params(struct struct BPSENSORLC* p);
 * @brief	: Init struct from hard-coded parameters (rather than database params in highflash)
 * @return	: 0
 * *************************************************************************/
void bpsensor_idx_v_struct_hardcode_params(struct BPSENSORLC* p)
{ /*
Copied for convenience--

*/
	p->size       = 3;
	p->crc        = 0;
   p->version    = 1;
	return;
}
