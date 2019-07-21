/******************************************************************************
* File Name          : contactor_hv.c
* Date First Issued  : 07/04/2019
* Description        : hv uart conversion and recalibration
*******************************************************************************/

#include "contactor_hv.h"
#include "SerialTaskReceive.h"
#include "hexbin.h"


/* *************************************************************************
 * void contactor_hv_uartline(struct CONTACTORFUNCTION* pcf);
 * @brief	: Get & convert ascii line to binary readings
 * @return	: readings stored in contactor function struct
 * *************************************************************************/
/*
Expect 13 hex chars: AAaaBBbbCCcc<CR> and,
place binary values in uint16_t array.
*/
/* From ContactorTask.h for convenience 
struct CNCNTHV
{
	struct CNTCTCALSI ical; // Scale integer calibration values
   float fhv;     // Calibrated: Float
	uint32_t ihv;  // Calibrated: Scaled
	uint16_t hv;   // Raw reading as received from uart
};
*/
void contactor_hv_uartline(struct CONTACTORFUNCTION* pcf)
{
	int i,j;
	uint8_t* pline;	// Pointer to line buffer
	do
	{
		/* Get pointer of next completed line. */
		pline = (uint8_t*)xSerialTaskReceiveGetline(pcf->prbcb3);
		if (pline != NULL)
		{ // Here, a line is ready.
			if (*(pline+12) != 0x0D) return; // Not correct line

			j = 0;
			for (i = 0; i < NUMHV; i++)
			{ 
				/* Table lookup ASCII to binary: 4 asci -> uint16_t */
				pcf->hv[i].hv  = \
                (hxbn[*(pline+0)] <<  0) | \
                (hxbn[*(pline+1)] <<  4) | \
				    (hxbn[*(pline+2)] <<  8) | \
                (hxbn[*(pline+3)] << 12);
				j += 4;
			}
		}
	} while (pline != NULL); // Catchup jic we got behind
	return;
}
/* *************************************************************************
 * void contactor_hv_calibrate(struct CONTACTORFUNCTION* pcf);
 * @brief	: Apply calibration to raw readings
 * @return	: readings stored in contactor function struct
 * *************************************************************************/
void contactor_hv_calibrate(struct CONTACTORFUNCTION* pcf)
{
	uint64_t u64;
	int i;
	for (i = 0; i < NUMHV; i++)
	{
		/* Volts = Volts per ADC tick * Raw ADC ticks (16b) from uart */
		u64 = pcf->hv[i].hvcal * pcf->hv[i].hv;
		pcf->hv[i].hvc = (u64 >> 32);
	}
	return;
}
