/******************************************************************************
* File Name          : ADCTask.h
* Date First Issued  : 02/01/2019
* Description        : ADC w DMA using FreeRTOS/ST HAL
*******************************************************************************/

#ifndef __ADCTASK
#define __ADCTASK

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"

/* WARNING: if not 16 adcfastsums needs to be changed. */
#define ADCSEQNUM 16  // Number of ADC scans in 1/2 of the DMA buffer

/* *************************************************************************/
osThreadId xADCTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: ADCTaskHandle
 * *************************************************************************/

extern osThreadId ADCTaskHandle;

#endif

