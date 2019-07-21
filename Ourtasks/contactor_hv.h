/******************************************************************************
* File Name          : contactor_hv.h
* Date First Issued  : 07/04/2019
* Description        : hv uart conversion and recalibration
*******************************************************************************/

#ifndef __CONTACTORHV
#define __CONTACTORHV

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "ContactorTask.h"
#include "CanTask.h"

/* *************************************************************************/
void contactor_hv_uartline(struct CONTACTORFUNCTION* pcf);
/* @brief	: Get & convert ascii line to binary readings
 * @return	: readings stored in contactor function struct
 * *************************************************************************/
void contactor_hv_calibrate(struct CONTACTORFUNCTION* pcf);
/* @brief	: Apply calibration to raw readings
 * @return	: readings stored in contactor function struct
 * *************************************************************************/

#endif
