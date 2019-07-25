/******************************************************************************
* File Name          : bpsensor_idx_v_struct.h
* Date First Issued  : 06/26/2019
* Board              :
* Description        : Load parameter struct
*******************************************************************************/

#include <stdint.h>
#include "common_can.h"
#include "iir_filter_lx.h"
#include "SensorTask.h"

#ifndef __BPSENSOR_IDX_V_STRUCT
#define __BPSENSOR_IDX_V_STRUCT

/* Hardware configuration option bit assignments. */

/* Calibration parameter, float */
// Use double for F103 to save float->double conversions
struct CNTCTCALHV
{
	uint32_t size;			// Number of items in struct
 	uint32_t crc;			// crc-32 placed by loader
	uint32_t version;		// struct version number
};

 struct BPSENSORLC
 {
	uint32_t size;			// Number of items in struct
 	uint32_t crc;			// crc-32 placed by loader
	uint32_t version;		// struct version number
};

/* *************************************************************************/
void bpsensor_idx_v_struct_hardcode_params(struct BPSENSORLC* p);
/* @brief	: Init struct from hard-coded parameters (rather than database params in highflash)
 * @return	: 0
 * *************************************************************************/
 
#endif

