/******************************************************************************
* File Name          : bpsensor_idx_v_struct.h
* Date First Issued  : 06/26/2019
* Board              :
* Description        : Load parameter struct
*******************************************************************************/

#include <stdint.h>
#include "common_can.h"
#include "iir_filter_lx.h"
#include "ContactorTask.h"

#ifndef __BPSENSOR_IDX_V_STRUCT
#define __BPSENSOR_IDX_V_STRUCT

/* Hardware configuration option bit assignments. */

/* High Voltage readings to send on uart line. */
#define NUMHV 3       // Number of hv readings
#define	IDXHV1  0 // High voltage reading: battery string side of contactor
#define	IDXHV2  1 // High voltage reading: DMOC+ side of contactor
#define	IDXHV3  2 // High voltage reading: across pre-charge resistor (two contactors)


/* Calibration parameter, float */
// Use double for F103 to save float->double conversions
struct CNTCTCALHV
{
	struct IIR_L_PARAM iir; // Filter: Time constant, integer scaling
	double dvcal;       // Calibration voltage applied
	uint32_t adchv;     // ADC reading for calibration voltage
	uint32_t offset;    // ADC reading for Sensor zero
};
struct CNTCTCALF
{
	double offset;
	double scale;
};
/* Calibration parameters, scaled integer */
struct CNTCTCALSI
{
	int32_t offset;
	int32_t scale;
};

/* Parameters contactor instance */
struct BPSENSORLC
 {
	uint32_t size;			// Number of items in struct
 	uint32_t crc;			// crc-32 placed by loader
	uint32_t version;		// struct version number


	// High voltage from uart
	struct CNTCTCALHV calhv[NUMHV]; 

 };

/* *************************************************************************/
void bpsensor_idx_v_struct_hardcode_params(struct CONTACTORLC* p);
/* @brief	: Init struct from hard-coded parameters (rather than database params in highflash)
 * @return	: 0
 * *************************************************************************/
 
#endif

