/******************************************************************************
* File Name          : PC_gateway_comm.h
* Date First Issued  : 10/04/2013
* Board              : Not specific to PC or stm32
* Description        : PC<->gateway 
*******************************************************************************/

#ifndef __PC_GATEWAY_COMM
#define __PC_GATEWAY_COMM

#include "common_can.h"
#include "SerialTaskSend.h"

/* **************************************************************************************/
int PC_msg_get(struct PCTOGATEWAY* ptr, u8 c);
/* @brief	: Build message from incoming BINARY bytes.  Binary data including seq number goes
 *              : int ptr.cmprs struct.  ASCII data goes into ptr->asc[]
 * @param	: ptr = Pointer to msg buffer (see common_can.h)
 *		:  ptr->cmprs.cm[] = binary bytes (including seq number, but not checksum)
 *		:  ptr->cmprs.ct = count of binary bytes (including seq number, but not checksum)
 *		:  ptr->asc[] = ascii/hex bytes as received, including '\n' plus a '\0'
 *		:  ptr->ctasc = count of ascii chars = (2* binary ct + 1);
 * @param	: c = byte to add msg being built
 * @return	:  1 = completed; ptr->cmprs.ct holds byte count of binary data
 *              :  0 = msg not ready; 
 *              : -1 = completed, but bad checksum
 *  		: -2 = completed, but too few bytes to be a valid CAN msg
 *  		: -3 = completed, but too many bytes to be a valid CAN msg
 * ************************************************************************************** */
int PC_msg_getASCII(struct PCTOGATEWAY* ptr, u8 c);
/* @brief	: Build message from incoming ASCII/HEX bytes.  Binary data including seq number goes
 *              : int ptr.cmprs struct.  ASCII data goes into ptr->asc[]
 * @param	: ptr = Pointer to msg buffer (see common_can.h)
 *		:  ptr->cmprs.cm[] = binary bytes (including seq number, but not checksum)
 *		:  ptr->cmprs.ct = count of binary bytes (including seq number, but not checksum)
 *		:  ptr->asc[] = ascii/hex bytes as received, including '\n' plus a '\0'
 *		:  ptr->ctasc = count of ascii chars = (2* binary ct + 1);
 * @param	: c = byte to add msg being built
 * @return	:  1 = completed; ptr->ct holds byte count
 *              :  0 = line not complete; 
 *              : -1 = completed, but bad checksum
 *  		: -2 = completed, but too few bytes to be a valid CAN msg
 *		: -3 = not even number of hex incoming bytes (less newline)
 *		: -4 = char count exceeds the max size for a max size CAN msg
 * ************************************************************************************** */
int PC_msg_asctobin(struct PCTOGATEWAY* p, char* pin);
/* @brief	: Initialize struct for building a gateway message from the PC
 * @param	: Pointer to gateway message wrapper (see common_can.h)
 * ************************************************************************************** */
void PC_msg_initg(struct PCTOGATEWAY* p);
/* @brief	: Initialize struct for building a gateway message from the PC
 * @param	: Pointer to gatewau message wrapper (see common_can.h)
 * ************************************************************************************** */
int PC_msg_prep(u8* pout, int outsize, u8* pin, int ct);
/* @brief	: Convert input bytes into output with byte stuffing, checksum and framing
 * @param	: pout = pointer to bytes with stuffing and chksum added 
 * @param	: outsize = size of output buffer (to prevent overflow if too small)
 * @param	: pin = Pointer to bytes to send
 * @param	: ct = byte count to send (does not include frame bytes, chksum, or stuffing bytes)
 * @return	: number of bytes in prepped message
 * ************************************************************************************** */
int PC_msg_prepASCII(struct SERIALSENDTASKBCB** ppbcb, struct PCTOGATECOMPRESSED* p);
/* @brief	: Convert input binary bytes to ascii/hex output lines
 * @param	: ppbcb = pointer to pointer to control block w buffer
 * @param	: p = Pointer to struct with "stuff" used in conversions
 * @return	: number of bytes in message queued
 * ************************************************************************************** */
u8 CANgenchksum(u8* p, int ct);
/* @brief	: Generate a one byte checksum
 * @param	: p = pointer to array to be checksummed
 * @param	: ct = number of bytes to be checksumned
 * @return	: Checksum
 * **************************************************************************************/
int CANcompress(struct PCTOGATECOMPRESSED* pout, struct CANRCVBUF* pin);
/* @brief	: Convert a "standard" format CAN msg into a compressed format byte array with byte ct.
 * @param	: pout = pointer to output w compressed msg in a byte array
 * @param	: pin = pointer to input with a "standard" format CAN msg
 * @return	: 0
 *		: pout->ct = total number of bytes in compressed msg
 *		: pout->c[] = msg bytes
 * Note: see '../svn_discoveryf4/docs/trunk/Userdocs/gateway_format.txt'

 * ************************************************************************************** */
int CANcompress_G(struct PCTOGATECOMPRESSED* pout, struct CANRCVBUF* pcan);
/* @brief	: Convert a "standard" format CAN msg into Gonzaga format byte array with byte ct.
 * @param	: pout = pointer to output w compressed msg in a byte array
 * @param	: pcan = pointer to input with a "standard" format CAN msg
 * @return	:  0 = OK; 
 *		: -1 = bogus dlc count with 29b id msg
 *		: -2 = bogus dlc count with 11b id msg
 *		: pout->ct = total number of bytes in compressed msg
 *		: pout->c[] = msg bytes
 * Note: see '../svn_discoveryf4/docs/trunk/Userdocs/gateway_format.txt'
 * Note: 'cm[0] holds the sequence number
 * ************************************************************************************** */
int CANuncompress(struct CANRCVBUF* pout,  struct PCTOGATECOMPRESSED* pin);
/* @brief	: Given a binary msg (w/o byte stuffing/framing), convert to "standard" CAN register format
 * @param	: pout = pointer to output w uncompressed msg in register format
 * @param	: pin = pointer to input with compressed (binary) msg
 * @return	:  0 = OK, and--
 *        	:    pin->seq = 1st byte of received (binary) frame
 *        	:    pout = struct filled with id, dlc, payload
 *		: negative = Some error--
 *		: -1 = Too few bytes to a valid 29b compressed msg
 *		: -2 = dlc: payload ct too large (> 8) in a 29 bit id msg
 *		: -3 = dlc doesn't match byte count in a 29 bit id msg
 *		: -4 = Too few bytes to a valid 11b compressed msg
 *		: -5 = dlc: payload ct too large (> 8) in a 11 bit id msg
 *		: -6 = dlc doesn't match byte count in a 11 bit id msg
 * Note: see '../svn_discoveryf4/docs/trunk/Userdocs/gateway_format.txt'
 * Note: 'cm[0] holds the sequence number
 * ************************************************************************************** */
void CANcopyuncompressed(struct CANRCVBUF* pout,  struct PCTOGATEWAY* pin);
/* @brief	: Copy incoming byte message (uncompressed) to CAN struct msg
 * @param	: pout = pointer to output 
 * @param	: pin = pointer to input
 * @return	: none
 * ************************************************************************************** */
int CANuncompress_G(struct CANRCVBUF* pout,  struct PCTOGATECOMPRESSED* pin);
/* @brief	: Given a binary msg (w/o byte stuffing/framing), convert to "standard" CAN register format
 * @param	: pout = pointer to output w uncompressed msg in register format
 * @param	: pin = pointer to input with compressed (binary) msg
 * @return	:  0 = OK, and--
 *        	:    pin->seq = 1st byte of received (binary) frame
 *        	:    pout = struct filled with id, dlc, payload
 *		: negative = Some error--
 *		: -1 = Too few bytes to a valid 29b compressed msg
 *		: -2 = dlc: payload ct too large (> 8) in a 29 bit id msg
 *		: -3 = dlc doesn't match byte count in a 29 bit id msg
 * Note: see '../svn_discoveryf4/docs/trunk/Userdocs/gateway_format.txt'
 * Note: 'cm[0] holds the sequence number
 * ************************************************************************************** */
void CANcopycompressed(struct CANRCVBUF* pout,  struct PCTOGATEWAY* pin);
/* @brief	: Copy incoming byte message (compressed) to CAN struct msg
 * @param	: pout = pointer to output 
 * @param	: pin = pointer to input
 * @return	: none
 * ************************************************************************************** */
int CAN_id_valid(u32 id);
/* @brief	: Check if CAN id (32b) holds a valid CAN bus id
 * @param	: id = CAN id
 * @return	:  1 = 29 bit is OK
 *		:  0 = 11 bit is OK
 *		: -1 = faux CAN msg id bit 0 on
 *              : -2 = faux CAN msg id: 11 bit id, but stray bits in remaining 18 bits
 * ************************************************************************************** */
u8 CANgenchkcompress(u8* p, int ct);
/* @brief	: Generate a one byte checksum
 * @param	: p = pointer to array to be checksummed
 * @param	: ct = number of bytes to be checksumned
 * @return	: Checksum
 * ************************************************************************************** */
int CANpctocan(struct CANRCVBUF* pout, struct PCTOGATEWAY* pin);
/* @brief	: Move incoming binary bytes into standard msg struct
 * @param	: pin = binary message after converting & validating incoming hex
 * @param	: pout = msg struct used by CAN routine
 * @return	: sequence number 
 * ************************************************************************************** */
int CAN_id_valid(u32 id);
/* @brief	: Check if CAN id (32b) holds a valid CAN bus id
 * @param	: id = CAN id
 * @return	:  0 = good
 *		: -1 = faux CAN msg id bit 0 on
 *              : -2 = faux CAN msg id: 11 bit id, but stray bits in remaining 18 bits
 * ************************************************************************************** */

#endif 

