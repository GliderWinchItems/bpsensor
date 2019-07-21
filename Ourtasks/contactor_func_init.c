/******************************************************************************
* File Name          : contactor_func_init.c
* Date First Issued  : 07/14/2019
* Description        : hv uart conversion and recalibration
*******************************************************************************/

#include "contactor_func_init.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "MailboxTask.h"
#include "CanTask.h"
#include "can_iface.h"
#include "can_iface.h"
#include "CanTask.h"
#include "ContactorTask.h"
#include "main.h"
#include "stm32f1xx_hal_tim.h"

/* From 'main.c' */
extern struct CAN_CTLBLOCK* pctl0;	// Pointer to CAN1 control block
extern TIM_HandleTypeDef htim4;



/* *************************************************************************
 * void contactor_func_init_init(struct CONTACTORFUNCTION* p, struct ADCFUNCTION* padc);
 *	@brief	: Initialize working struct for ContactorTask
 * @param	: p    = pointer to ContactorTask
 * @param	: padc = pointer to ADC working struct
 * *************************************************************************/
void contactor_func_init_init(struct CONTACTORFUNCTION* p, struct ADCFUNCTION* padc)
{
	int i;

	/* Pointer to ADC working parameters. */
	p->padc = padc;

	/* For each HV */
	for (i = 0; i < NUMHV; i++)
	{
		// Pointer to filter parameters
		p->hv[i].iir.pprm = &p->lc.calhv[i].iir;

		// Calibration volts per adc tick
		p->hv[i].dscale = (p->lc.calhv[i].dvcal / p->lc.calhv[i].adchv);
	}

	/* Battery low voltage as scaled uint32_t. */
	double dtmp = p->lc.fbattlow / p->hv[IDXHV1].dscale; 
	p->ibattlow = (dtmp * (1 << ADCSCALEbits));

	/* Prep-charge end volts threshold */
	p->iprechgendv = (p->lc.ddiffb4 * (1 << ADCSCALEbits));

	/* Convert ms to timer ticks. */
p->ka_k        = pdMS_TO_TICKS(p->lc.ka_t);        // Command/Keep-alive CAN msg timeout duration.
p->prechgmin_k = pdMS_TO_TICKS(p->lc.prechgmin_t); // Minimum pre-charge duration
p->prechgmax_k = pdMS_TO_TICKS(p->lc.prechgmax_t); // Maximum allowed for voltage to reach threshold
p->close1_k    = pdMS_TO_TICKS(p->lc.close1_t);    // contactor #1 coil energize-closure (timeout delay ticks)
p->close2_k    = pdMS_TO_TICKS(p->lc.close2_t);    // contactor #2 coil energize-closure (timeout delay ticks)
p->open1_k     = pdMS_TO_TICKS(p->lc.open1_t);     // contactor #1 coil de-energize-open (timeout delay ticks)
p->open2_k     = pdMS_TO_TICKS(p->lc.open2_t);     // contactor #2 coil de-energize-open (timeout delay ticks)
p->keepalive_k = pdMS_TO_TICKS(p->lc.keepalive_t); // keep-alive timeout (timeout delay ticks)
p->hbct1_k     = pdMS_TO_TICKS(p->lc.hbct1_t);     // Heartbeat ct: ticks between sending msgs hv1:cur1
p->hbct2_k     = pdMS_TO_TICKS(p->lc.hbct2_t);     // Heartbeat ct: ticks between sending msgs hv2:cur2

	/* Add CAN Mailboxes                         CAN           CAN ID              Notify bit   Paytype */
	p->pmbx_cid_cmd_i       =  MailboxTask_add(pctl0,p->lc.cid_cmd_i,      NULL,CNCTBIT06,0,36);
	p->pmbx_cid_keepalive_i =  MailboxTask_add(pctl0,p->lc.cid_keepalive_i,NULL,CNCTBIT07,0,23);
	p->pmbx_cid_gps_sync    =  MailboxTask_add(pctl0,p->lc.cid_gps_sync,   NULL,CNCTBIT08,0,23);

	/* PWM working struct for switching PWM values */
	p->sConfigOCn.OCMode = TIM_OCMODE_PWM1;
	p->sConfigOCn.Pulse = 0;	// New PWM value inserted here during execution
	p->sConfigOCn.OCPolarity = TIM_OCPOLARITY_HIGH;
	p->sConfigOCn.OCFastMode = TIM_OCFAST_DISABLE;

	// Convert PWM as percent to timer count
   p->ipwmpct1 = p->lc.fpwmpct1 * 0.01 * (htim4.Init.Period + 1) - 1;
   p->ipwmpct2 = p->lc.fpwmpct2 * 0.01 * (htim4.Init.Period + 1) - 1;

	/* Pre-load fixed data in CAN msgs */
	for (i = 0; i < NUMCANMSGS; i++)
	{
		p->canmsg[i].pctl = pctl0;   // Control block for CAN module (CAN 1)
		p->canmsg[i].maxretryct = 8; //
		p->canmsg[i].bits = 0;       //
		p->canmsg[i].can.dlc = 8;    // Default payload size (modified when loaded and sent)
	}

	// Pre-load CAN ids
	p->canmsg[CID_KA_R ].can.id  = p->lc.cid_keepalive_r;
	p->canmsg[CID_MSG1 ].can.id  = p->lc.cid_msg1;
	p->canmsg[CID_MSG2 ].can.id  = p->lc.cid_msg2;
	p->canmsg[CID_CMD_R].can.id  = p->lc.cid_cmd_r;
	p->canmsg[CID_HB1  ].can.id  = p->lc.cid_hb1;
	p->canmsg[CID_HB2  ].can.id  = p->lc.cid_hb2;

	p->canmsg[0].can.dlc = 4; // Default payload length
	
	return;
}


