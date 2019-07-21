/******************************************************************************
* File Name          : contactor_msgs.c
* Date First Issued  : 07/05/2019
* Description        : Setup and send non-command function CAN msgs
*******************************************************************************/

#include "contactor_msgs.h"
#include "MailboxTask.h"

static void hvpayload(struct CONTACTORFUNCTION* pcf, uint8_t idx1,uint8_t idx2,uint8_t idx3);
static void load4(uint8_t *po, uint32_t n);

/* *************************************************************************
 * void contactor_msg1(struct CONTACTORFUNCTION* pcf, uint8_t w);
 *	@brief	: Setup and send responses: battery string voltage & battery current
 * @param	: pcf = Pointer to working struct for Contactor function
 * @param	: w = switch for CID_HB1 (0) or CID_MSG1 CAN ids (1)
 * *************************************************************************/
void contactor_msg1(struct CONTACTORFUNCTION* pcf, uint8_t w)
{
	// For loading float into payload
	union UIF
	{
		uint32_t ui;
		float    f;
	}tmp;

	double dtmp;
	uint8_t idx2;

	/* Use heartbeat or polled msg CAN id */
	if (w == 0) 
		idx2 = CID_MSG1;
	else
		idx2 = CID_HB1;

	// Load Battery string voltage (IDXHV1) as float into payload
	hvpayload(pcf, IDXHV1, idx2, 0);

	// Load Battery current reading as a float into payload
	dtmp  = pcf->padc->chan[ADC1IDX_CURRENTTOTAL].ival; // Convert int to double
	dtmp *= pcf->padc->chan[ADC1IDX_CURRENTTOTAL].dscale;
	tmp.f = dtmp; // Convert to float
	load4(&pcf->canmsg[idx2].can.cd.uc[4],tmp.ui); // Load float

	pcf->canmsg[idx2].can.dlc = 8;

	// Queue CAN msg
	xQueueSendToBack(CanTxQHandle,&pcf->canmsg[idx2],portMAX_DELAY);
	return;

}
/* *************************************************************************
 * void contactor_msg2(struct CONTACTORFUNCTION* pcf, uint8_t w);
 *	@brief	: Setup and send responses: voltages: DMOC+, DMOC-
 * @param	: pcf = Pointer to working struct for Contactor function
 * @param	: w = switch for CID_HB1 (0) or CID_MSG1 CAN ids (1)
 * *************************************************************************/
void contactor_msg2(struct CONTACTORFUNCTION* pcf, uint8_t w)
{
	uint8_t idx2;

	if (w == 0) 
		idx2 = CID_MSG2;
	else
		idx2 = CID_HB2;

	// Load high voltage 2 as a float into payload
	hvpayload(pcf, IDXHV2, idx2, 0);

	// Load high voltage 3 as a float into payload
	hvpayload(pcf, IDXHV3, idx2, 4);

	pcf->canmsg[idx2].can.dlc = 8;

	// Queue CAN msg
	xQueueSendToBack(CanTxQHandle,&pcf->canmsg[idx2],portMAX_DELAY);
	return;
}
/* *************************************************************************
 * static void hvpayload(struct CONTACTORFUNCTION* pcf, uint8_t idx1,uint8_t idx2,uint8_t idx3);
 *	@brief	: Setup and send responses: voltages: DMOC+, DMOC-
 * @param	: pcf = Pointer to working struct for Contactor function
 * @param	: idx1 = index into hv array (0-(NUMHV-1))
 * @param	: idx2 = index into ncan msg (0-(NUMCANMSGS-1))
 * @param	: idx3 = index into payload byte array
 * *************************************************************************/
static void hvpayload(struct CONTACTORFUNCTION* pcf, uint8_t idx1,uint8_t idx2,uint8_t idx3)
{
	union UIF
	{
		uint32_t ui;
		float    f;
	}tmp;
	double dtmp;

	// Load high voltage 3 as a float into payload
	dtmp  = pcf->hv[idx1].hv;  // Convert uint16_t to float
	dtmp *= pcf->hv[idx2].dscale;
	tmp.f = dtmp; // Convert double to float
	load4(&pcf->canmsg[idx2].can.cd.uc[idx3],tmp.ui);
	return;
}
/* *************************************************************************
 * void contactor_msg_ka(struct CONTACTORFUNCTION* pcf);
 *	@brief	: Setup and send Keep-alive response
 * @param	: pcf = Pointer to working struct for Contactor function
 * *************************************************************************/
void contactor_msg_ka(struct CONTACTORFUNCTION* pcf)
{
	/* Return command byte */
	pcf->canmsg[CID_KA_R].can.cd.uc[0]  = pcf->pmbx_cid_keepalive_i->ncan.can.cd.uc[0];

	/* Fault code */
	pcf->canmsg[CID_KA_R].can.cd.uc[1] = pcf->faultcode;

	/* substate connect codes */
	pcf->canmsg[CID_KA_R].can.cd.uc[1] = (pcf->substateC << 0);
	pcf->canmsg[CID_KA_R].can.cd.uc[1] = (pcf->substateC << 4);

	pcf->canmsg[CID_KA_R].can.dlc = 8; // Number of payload bytes

	// Queue CAN msg
	xQueueSendToBack(CanTxQHandle,&pcf->canmsg[CID_KA_R],portMAX_DELAY);
	return;
}
/* *************************************************************************
 * static void load4(uint8_t *po, uint32_t n);
 *	@brief	: Copy uint32_t into byte array (not aligned)
 * *************************************************************************/
static void load4(uint8_t *po, uint32_t n)
{
	*(po + 0) = (n >>  0);
	*(po + 1) = (n >>  8);
	*(po + 2) = (n >> 16);
	*(po + 3) = (n >> 24);
	return;
}
