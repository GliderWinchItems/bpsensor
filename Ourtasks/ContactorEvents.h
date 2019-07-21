/******************************************************************************
* File Name          : ContactorEvents.h
* Date First Issued  : 07/01/2019
* Description        : Events in Contactor function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#ifndef __CONTACTOREVENTS
#define __CONTACTOREVENTS

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "adc_idx_v_struct.h"

void ContactorEvents_00(struct CONTACTORFUNCTION* pcf);
void ContactorEvents_01(struct CONTACTORFUNCTION* pcf);
void ContactorEvents_02(struct CONTACTORFUNCTION* pcf);
void ContactorEvents_03(struct CONTACTORFUNCTION* pcf);
void ContactorEvents_04(struct CONTACTORFUNCTION* pcf);
void ContactorEvents_05(struct CONTACTORFUNCTION* pcf);
void ContactorEvents_06(struct CONTACTORFUNCTION* pcf);
void ContactorEvents_07(struct CONTACTORFUNCTION* pcf);
void ContactorEvents_08(struct CONTACTORFUNCTION* pcf);

#endif

