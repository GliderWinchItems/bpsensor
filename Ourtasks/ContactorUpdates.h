/******************************************************************************
* File Name          : ContactorUpdates.h
* Date First Issued  : 07/02/2019
* Description        : Update outputs in Contactor function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#ifndef __CONTACTORUPDATES
#define __CONTACTORUPDATES

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "adc_idx_v_struct.h"
#include "ContactorTask.h"


/* *************************************************************************/
void ContactorUpdates(struct CONTACTORFUNCTION* pcf);
/* @brief	: Update outputs based on bits set
 * *************************************************************************/
#endif

