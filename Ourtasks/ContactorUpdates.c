/******************************************************************************
* File Name          : ContactorUpdates.c
* Date First Issued  : 07/02/2019
* Description        : Update outputs in Contactor function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "ADCTask.h"
#include "adctask.h"

#include "ContactorTask.h"
#include "contactor_idx_v_struct.h"
#include "CanTask.h"
#include "contactor_msgs.h"

#include "morse.h"

/* From 'main.c' */
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim4;

/* *************************************************************************
 * void ContactorUpdates(struct CONTACTORFUNCTION* pcf);
 * @brief	: Update outputs based on bits set
 * *************************************************************************/
void ContactorUpdates(struct CONTACTORFUNCTION* pcf)
{
	/* Queue KA response CAN msg. */
	if ((pcf->outstat & CNCTOUT05KA) != 0)
	{ // Here, someone flagged to have this msg sent
		pcf->outstat &= ~CNCTOUT05KA; // Reset request
		
		/* Load payload */
		pcf->canmsg[0].can.cd.uc[0] = pcf->state;
		// TODO set bits 7 & 6
		xQueueSendToBack(CanTxQHandle,&pcf->canmsg[0],portMAX_DELAY);
	}

	/* Contactor #1 on/off energization */
	if (((pcf->outstat & CNCTOUT00K1) ^ (pcf->outstat_prev & CNCTOUT00K1)) != 0)
	{ // Change was signalled
		if ((pcf->outstat & CNCTOUT00K1) != 0)
		{ // Off->On (100%)
			pcf->sConfigOCn.Pulse = htim4.Init.Period+2; // Max+1 PWM period
			HAL_TIM_PWM_ConfigChannel(&htim4, &pcf->sConfigOCn, TIM_CHANNEL_3);
			pcf->outstat_prev |= (pcf->outstat & CNCTOUT00K1);
		}
		else
		{ // On->Off
			pcf->sConfigOCn.Pulse = 0; // Period = 0
			HAL_TIM_PWM_ConfigChannel(&htim4, &pcf->sConfigOCn, TIM_CHANNEL_3);
			pcf->outstat_prev &= ~(pcf->outstat & CNCTOUT00K1);		
		}
	}
	/* Contactor #1 pwm energization */
	if (((pcf->outstat & CNCTOUT06KAw) ^ (pcf->outstat_prev & CNCTOUT06KAw)) != 0)
	{ // Change was signalled
		if ((pcf->outstat & CNCTOUT06KAw) != 0)
		{ // No pwm->Yes pwm @ x%
			pcf->sConfigOCn.Pulse = pcf->ipwmpct1;
			HAL_TIM_PWM_ConfigChannel(&htim4, &pcf->sConfigOCn, TIM_CHANNEL_3);
			pcf->outstat_prev |= (pcf->outstat & CNCTOUT06KAw);
		}
		else
		{ // On->Off
			pcf->sConfigOCn.Pulse = 0;
			HAL_TIM_PWM_ConfigChannel(&htim4, &pcf->sConfigOCn, TIM_CHANNEL_3);
			pcf->outstat_prev &= ~(pcf->outstat & CNCTOUT06KAw);		
		}
	}

	/* Contactor #2 on/off energization */
	if (((pcf->outstat & CNCTOUT01K2) ^ (pcf->outstat_prev & CNCTOUT01K2)) != 0)
	{ // Change was signalled
		if ((pcf->outstat & CNCTOUT01K2) != 0)
		{ // Off->On (100%)
			pcf->sConfigOCn.Pulse = htim4.Init.Period+2; // Max+1 PWM period
			HAL_TIM_PWM_ConfigChannel(&htim4, &pcf->sConfigOCn, TIM_CHANNEL_3);
			pcf->outstat_prev |= (pcf->outstat & CNCTOUT01K2);
		}
		else
		{ // On->Off
			pcf->sConfigOCn.Pulse = 0; // Period = 0
			HAL_TIM_PWM_ConfigChannel(&htim4, &pcf->sConfigOCn, TIM_CHANNEL_4);
			pcf->outstat_prev &= ~(pcf->outstat & CNCTOUT01K2);		
		}
	}
	/* Contactor #2 pwm energization */
	if (((pcf->outstat & CNCTOUT07KAw) ^ (pcf->outstat_prev & CNCTOUT07KAw)) != 0)
	{ // Change was signalled
		if ((pcf->outstat & CNCTOUT07KAw) != 0)
		{ // No pwm->Yes pwm @ x%
			pcf->sConfigOCn.Pulse = pcf->ipwmpct1;
			HAL_TIM_PWM_ConfigChannel(&htim4, &pcf->sConfigOCn, TIM_CHANNEL_4);
			pcf->outstat_prev |= (pcf->outstat & CNCTOUT07KAw);
		}
		else
		{ // On->Off
			pcf->sConfigOCn.Pulse = 0;
			HAL_TIM_PWM_ConfigChannel(&htim4, &pcf->sConfigOCn, TIM_CHANNEL_4);
			pcf->outstat_prev &= ~(pcf->outstat & CNCTOUT07KAw);		
		}
	}

	/* Queue keep-alive status CAN msg */
	if ((pcf->outstat & CNCTOUT05KA) != 0)
	{
		pcf->outstat &= !CNCTOUT05KA;	
		contactor_msg_ka(pcf);
	}
}

