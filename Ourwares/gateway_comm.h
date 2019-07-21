/******************************************************************************
* File Name          : gateway_comm.h
* Date First Issued  : 01/19/2019
* Board              : 
* Description        : PC<->gateway 
*******************************************************************************/

#ifndef __GATEWAY_COMM
#define __GATEWAY_COMM

#include "common_misc.h"
#include "common_can.h"
#include "SerialTaskSend.h"
#include "PC_gateway_comm.h"

/* ***************************************************************************************/ 
uint16_t gateway_comm_CANtoPC(struct SERIALSENDTASKBCB** ppbcb, struct CANRCVBUF* pcan);
/* @brief	: Convert CAN msg to ASCII/HEX and send via serial port
 * @param	: ppbcb = Pointer to pointer to buffer control block and byte buffer
 * @param	: pcan = CAN message
 * @return	: OK
 * ************************************************************************************** */

#endif

