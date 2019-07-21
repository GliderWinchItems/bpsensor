/******************************************************************************
* File Name          : gateway_CANtoPC.h
* Date First Issued  : 01/19/2019
* Board              : FreeRTOS/STM32CubeMX
* Description        : gateway (binary) to PC (ascii/hex)
*******************************************************************************/

#ifndef __GATEWAY_CANTOPC
#define __GATEWAY_CANTOPC

#include <stdint.h>
#include "getserialbuf.h"
#include "common_can.h"

/* **************************************************************************************/
void gateway_CANtoPC(struct SERIALSENDTASKBCB** ppbcb, struct CANRCVBUF* pcan);
/* @brief	: Convert CAN msg into ascii/hex in a buffer for SerialTaskSend
 * @param	: pycb = pointer to pointer to buffer control block w buffer and uart handle
 * @param	: pcan = CAN msg
 * @return	: 
 * ************************************************************************************** */

#endif
