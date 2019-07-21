/******************************************************************************
* File Name          : gateway_comm.c
* Date First Issued  : 01/19/2019
* Board              : 
* Description        : PC<->gateway 
*******************************************************************************/

#include "gateway_comm.h"
#include "USB_PC_gateway.h"

static struct PCTOGATEWAY pctogateway; 	// CAN->PC
uint8_t seqct = 0;

/* ************************************************************************************** 
 * uint16_t gateway_comm_CANtoPC(struct SERIALSENDTASKBCB** ppbcb, struct CANRCVBUF* pcan);
 * @brief	: Convert CAN msg to ASCII/HEX and send via serial port
 * @param	: ppbcb = Pointer to pointer to buffer control block and byte buffer
 * @param	: pcan = CAN message
 * @return	: OK
 * ************************************************************************************** */
uint16_t gateway_comm_CANtoPC(struct SERIALSENDTASKBCB** ppbcb, struct CANRCVBUF* pcan)
{
	pctogateway.mode_link = MODE_LINK;
	pctogateway.cmprs.seq = seqct++;
	USB_toPC_msg_mode(ppbcb, &pctogateway, pcan); 	// Send to serial port
	return 0;
}

