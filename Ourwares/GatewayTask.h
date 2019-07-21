/******************************************************************************
* File Name          : GatewayTask.h
* Date First Issued  : 02/25/2019
* Description        : PC<->gateway using usart2, notified by MailboxTask
*******************************************************************************/

#ifndef __GATEWAYTASK
#define __GATEWAYTASK

#include "stm32f1xx_hal_def.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "malloc.h"
#include "common_can.h"



/* *************************************************************************/
osThreadId xGatewayTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: GatewayHandle
 * *************************************************************************/
void StartGatewayTask(void const * argument);
/*	@brief	: Task startup
 * *************************************************************************/
osThreadId xGatewayTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: GatewayHandle
 * *************************************************************************/

/* A notification copies the internal notification word to this. */
extern uint32_t GatewayTask_noteval;    // Receives notification word upon an API notify

#endif

