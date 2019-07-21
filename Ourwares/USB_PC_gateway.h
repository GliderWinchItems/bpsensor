/******************************************************************************
* File Name          : USB_PC_gateway.h
* Date First Issued  : 10/16/2013
* Board              : Discovery F4
* Description        : PC<->gateway 
*******************************************************************************/

#ifndef __USB_PC_GATEWAY
#define __USB_PC_GATEWAY

#include "common_misc.h"
#include "common_can.h"
#include "SerialTaskReceive.h"
#include "yprintf.h"
/* **************************************************************************************/
int USB_toPC_msg_mode(struct SERIALSENDTASKBCB** ppbcb, struct PCTOGATEWAY* ptr, struct CANRCVBUF* pcan);
/* @brief	: Send msg to PC in selected mode
 * @brief	: ppbcb = pointer to pointer to buffer control block
 * @param	: pcan = pointer to struct with CAN msg (stm32 register format)
 * @param	: ptr = Pointer to msg buffer
 *              : ptr->mode_link = selection of PC<->gateway mode and format (binary, ascii,...)
 *              : ptr->mode_send = CAN msg (from calling routine) is binary or ascii
 *              : ptr->c[] = binary msg, possibly compressed
 *              : ptr->asc[] = asc line
 *              : ptr->ct; ptr->ctasc; counts for above, repsectively.
 *              : ptr->seq; sequence number to be sent., (if applicable to mode)
 * @return	: postive = number of bytes written
 *              : negative = error
 *              : -1 = The bozo that called this routine gave us booogus ptr->mode_link|send!
 * ************************************************************************************** */
int USB_toPC_msgASCII(struct SERIALSENDTASKBCB** ppbcb, struct PCTOGATECOMPRESSED* p);
/* @brief	: Send msg to PC after converting binary msg to ASCII/HEX 
 * @param	: pbufy = Pointer buffer control block w buffer and uart handle
 * @param	: p = Pointer to struct with bytes to send to PC
 * @return	: count of bytes written
 * ************************************************************************************** */
int USB_toPC_msgBIN(struct SERIALSENDTASKBCB** ppbcb, struct PCTOGATECOMPRESSED* p);
/* @brief	: Send msg to PC in the binary format
 * @brief	: pbufy = Pointer buffer control block w buffer and uart handle
 * @param	: p = Pointer to struct with bytes to send to PC
 * @return	: count of bytes written
 * ************************************************************************************** */
int USB_toPC_msg_asciican(struct SERIALSENDTASKBCB** ppbcb, char* pin, struct PCTOGATEWAY* ptr);
/* @brief	: I have a CAN msg in ASCII/HEX (no seq, no chksum).  Send in selected mode.
 * *param	: pbufy = pointer to buffer control block w buffer for uart
 * *param	: ptr = pointer to stuff used in conversions
 * @return	: postive = number of bytes written
 *              : negative = error
 *              : -1 = The bozo that called this routine gave us booogus ptr->mode_link!
 * @NOTE	: Be sure ptr->mode_link is set!
 * ************************************************************************************** */


#endif 

