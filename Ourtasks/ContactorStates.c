/******************************************************************************
* File Name          : ContactorStates.c
* Date First Issued  : 07/01/2019
* Description        : States in Contactor function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "ADCTask.h"
#include "adctask.h"

#include "ContactorStates.h"
#include "ContactorTask.h"
#include "contactor_idx_v_struct.h"
#include "morse.h"

void static transition_disconnected(struct CONTACTORFUNCTION* pcf);
static void transition_connecting(struct CONTACTORFUNCTION* pcf);
static void transition_disconnecting(struct CONTACTORFUNCTION* pcf);
static void new_state(struct CONTACTORFUNCTION* pcf, uint32_t newstate);
static void open_contactors(struct CONTACTORFUNCTION* pcf);

/* TIM4 CH3, CH4 drive Conatactor #1, #2 coils. */
extern TIM_HandleTypeDef htim4; // Needs this for autoreload period


/*
		case DISCONNECTED:
			ContactorStates_disconnected(pcf);
			break;
		case CONNECTING:
			ContactorStates_connecting(pcf);
			break;
		case CONNECTED:
			ContactorStates_connected(pcf);
			break;
		case FAULTING:
			ContactorStates_faulting(pcf);
			break;
		case FAULTED:
			ContactorStates_faulted(pcf);
			break;
		case RESETTING:
			ContactorStates_resetting(pcf);
			break;
		case DISCONNECTING:
			ContactorStates_disconnecting(pcf);
			break;
*/
/* *************************************************************************
 * @brief	: 
 * *************************************************************************/
/* ==== xDISCONNECTED ======================================= */
void static transition_disconnected(struct CONTACTORFUNCTION* pcf)
{ // Intialize disconnected state
	new_state(pcf,DISCONNECTED);
	return;
}
/* ==== DISCONNECTED ======================================== */
void ContactorStates_disconnected(struct CONTACTORFUNCTION* pcf)
{
	uint32_t tmp;

	/* Check for battery string below threshold. */
	if (pcf->hv[IDXHV1].hvc < pcf->ibattlow)
	{ // Here, battery voltage is too low (or missing!)
		transition_faulting(pcf,BATTERYLOW);
		return;
	}
	/* Check if aux contacts match, if aux contacts present. */
	if ((pcf->lc.hwconfig & AUX1PRESENT) != 0)
	{ // Aux contacts are present
		tmp = HAL_GPIO_ReadPin(AUX1_GPIO_REG,AUX1_GPIO_IN);// read i/o pin
		if ((pcf->lc.hwconfig & AUX1SENSE) != 0)
		{ // Reverse sense of bit
			tmp ^= 0x1;
		}
		if (tmp != GPIO_PIN_RESET)
		{ // Transition to fault state; set fault code
			transition_faulting(pcf,CONTACTOR1_OFF_AUX1_ON);
			return;			
		}
	}
	/* Check if aux contacts match, if aux contacts present. */
	if ((pcf->lc.hwconfig & AUX2PRESENT) != 0)
	{ // Aux contacts are present
		tmp = HAL_GPIO_ReadPin(AUX2_GPIO_REG,AUX2_GPIO_IN);// read i/o pin
		if ((pcf->lc.hwconfig & AUX2SENSE) != 0)
		{ // Reverse sense of bit
			tmp ^= 0x1;
		}
		if (tmp != GPIO_PIN_RESET)
		{ // Transition to fault state; set fault code
			transition_faulting(pcf,CONTACTOR2_OFF_AUX2_ON); 
			return;			
		}
	}
	/* Command/Keep-alive CAN msg received. */
	if ((pcf->evstat & CMDCONNECT) != 0)
	{ // Here, request to connect
		transition_connecting(pcf);
		return;
	}

	/* JIC.  Be sure Updates have both coils de-energized. */
	pcf->outstat &= ~(CNCTOUT00K1 | CNCTOUT01K2 | CNCTOUT06KAw | CNCTOUT07KAw);
}
/* *************************************************************************
 * void ContactorStates_connecting(struct CONTACTORFUNCTION* pcf);
 * @brief	: CONNECTING state
 * *************************************************************************/
/* ===== xCONNECTING ==================================================== */
static void transition_connecting(struct CONTACTORFUNCTION* pcf)
{ // Intialize disconnected state

	/* Reset sub-states for connecting */
	pcf->substateC = CONNECT_C1;

	/* Set one-shot timer for contactor #1 closure delay */
	xTimerChangePeriod(pcf->swtimer2,pcf->close1_k, 2); 

	/* Energize coil #1 */
	pcf->outstat |= CNCTOUT00K1; // Energize coil during update section

	/* Update main state */
	new_state(pcf,CONNECTING);
	return;
}
/* ====== CONNECTING ==================================================== */
void ContactorStates_connecting(struct CONTACTORFUNCTION* pcf)
{
	uint32_t tmp;

	switch(pcf->substateC)
	{
	case CONNECT_C1:  // Contactor #1 closure delay
		if ((pcf->evstat & CNCTEVTIMER2) == 0) break;
		/* Timer timed out, so contactor #1 should be closed. */

		/* Check if aux contacts match, if aux contacts present. */
		if ((pcf->lc.hwconfig & AUX1PRESENT) != 0)
		{ // Aux contacts are present
			tmp = HAL_GPIO_ReadPin(AUX1_GPIO_REG,AUX1_GPIO_IN);// read i/o pin
			if ((pcf->lc.hwconfig & AUX1SENSE) == 1)
			{ // Reverse sense of bit
				tmp ^= 0x1;
			}
			if (tmp != GPIO_PIN_SET)
			{ // Transition to fault state; set fault code
				/* Aux contact says it did not close. */
				transition_faulting(pcf,CONTACTOR1_ON_AUX1_OFF);
				return;			
			}
		}

		/* For two contactor config, we can check if it looks closed. */
		if ((pcf->lc.hwconfig & ONECONTACTOR) != 0)
		{ // Here, two contactor config, so voltage should jump up
			if ((pcf->hv[IDXHV1].hvc - pcf->hv[IDXHV2].hvc) < pcf->ihv1mhv2max) // 
			{
				transition_faulting(pcf,CONTACTOR1_DOES_NOT_APPEAR_CLOSED); 
				return;
			}
		}	

		/* Here, looks good, so start a minimum pre-charge delay. */

		/* If this contactor is to be PWM'ed drop down from 100%. */
		if ((pcf->lc.hwconfig & PWMCONTACTOR1) != 0)
		{ // TIM4 CH3 Lower PWM from 100%
			pcf->outstat |= CNCTOUT06KAw; // Switch pwm during update section
		}

		/* Set one-shot timer for a minimum pre-charge duration. */
		xTimerChangePeriod(pcf->swtimer2, pcf->prechgmin_k, 2); 

		pcf->substateC = CONNECT_C2;
		break;
/* ...................................................................... */
	case CONNECT_C2:  // Minimum pre-charge duration delay
		if ((pcf->evstat & CNCTEVTIMER2) != 0)
		{ // Timer2 timed out before cutoff voltage reached
			pcf->evstat &= ~CNCTEVTIMER2;	// Clear timeout bit 
			transition_faulting(pcf,PRECHGVOLT_NOTREACHED);
			return;
		}
		/* Check if voltage has reached cutoff. */
		if ((pcf->evstat & CNCTEVHV) != 0)
		{ // Here, new readings available
			pcf->evstat &= ~CNCTEVHV; // Clear new reading bit

			if ((pcf->lc.hwconfig & ONECONTACTOR) == 0)
			{ // Here, one contactor
				if ((pcf->hv[IDXHV1].hvc - pcf->hv[IDXHV2].hvc) < pcf->iprechgendv)
				{ // Here, end of pre-charge.  Energize contactor 2
					pcf->outstat |= CNCTOUT07KAw; // Energize #2 during update section

					/* Set one-shot timer for contactor (relay) 2 closure duration. */
					xTimerChangePeriod(pcf->swtimer2,pcf->close2_k, 2); 

					pcf->substateC = CONNECT_C3; // Next substate
					return;			
				}
			}
			else
			{ // Here, two contactors
				if (pcf->hv[IDXHV3].hvc < pcf->iprechgendv)
				{ // Here, end of pre-charge. Energize contactor 2
					pcf->outstat |= CNCTOUT07KAw; // Energize #2 during update section

					/* Set one-shot timer for contactor 2 closure duration. */
					xTimerChangePeriod(pcf->swtimer2,pcf->close2_k, 2); 

					pcf->substateC = CONNECT_C3; // Next substate
					return;
				}
			}
		}
		// Here, event was not relevant
		break;
/* ...................................................................... */
	case CONNECT_C3:  // Contactor #2 close

		if ((pcf->evstat & CNCTEVTIMER2) != 0)
		{ // Timer2 timed out: Contactor #2 should be closed
			if (((pcf->hv[IDXHV1].hvc - pcf->hv[IDXHV2].hvc) > pcf->ihv1mhv2max))
			{ // Here, something not right with contactor closing
				transition_faulting(pcf,CONTACTOR2_CLOSED_VOLTSTOOBIG);
			}
					
			/* If this contactor is to be PWM'ed drop down from 100%. */
			if ((pcf->lc.hwconfig & PWMCONTACTOR1) != 0)
			{ // TIM4 CH3 Lower PWM from 100%
				pcf->outstat |= CNCTOUT07KAw; // Switch to lower pwm in update section
			}
			new_state(pcf,CONNECTED);
		}
		/* event not relevant. Continue waiting for timer2 */
		break;
	}
}
		
/* ======= CONNECTED ==================================================== */
void ContactorStates_connected(struct CONTACTORFUNCTION* pcf)
{
	/* Terminate CONNECTED if commands are disconnect or reset. */
	if ( ((pcf->evstat & CNCTEVCMDCN) == 0) | ((pcf->evstat & CMDRESET) != 0) ) 
	{ // 
		transition_disconnecting(pcf);
	}
	/* Continue connection. */
	return;
}
/* ===== xDISCONNECTING ================================================= */
static void transition_disconnecting(struct CONTACTORFUNCTION* pcf)
{
		open_contactors(pcf);
		new_state(pcf,DISCONNECTING);	
		return;	
}
/* ===== DISCONNECTING ================================================== */
void ContactorStates_disconnecting(struct CONTACTORFUNCTION* pcf)
{
	if ((pcf->evstat & CNCTEVTIMER2) != 0)
	{
			pcf->state = DISCONNECTED;
	}		
	/* Here, still waiting for TIMER2 to time out. */
	return;
}
/* ===== xFAULTING ====================================================== */
void transition_faulting(struct CONTACTORFUNCTION* pcf, uint8_t fc)
{
		open_contactors(pcf);     // Be sure to open contactors, set timer2
		pcf->faultcode = fc;	     // Set fault code
		new_state(pcf,FAULTING);	
		return;	
}
/* ===== FAULTING ======================================================= */
void ContactorStates_faulting(struct CONTACTORFUNCTION* pcf)
{
	if ((pcf->evstat & CNCTEVTIMER2) != 0)
	{
			new_state(pcf,FAULTED);
	}		
	/* Here, still waiting for TIMER2 to time out. */
	return;
}
/* ===== FAULTED ======================================================= */
void ContactorStates_faulted(struct CONTACTORFUNCTION* pcf)
{
	if ((pcf->evstat & CMDRESET) != 0)
	{ // Command to RESET
		pcf->faultcode = 0; // Clear fault code.
		new_state(pcf,DISCONNECTED);
	}
	/* Stuck in this state until Command to Reset */
	return;
}
/* ===== RESET ========================================================= */
void ContactorStates_reset(struct CONTACTORFUNCTION* pcf)
{
	if ((pcf->evstat & CMDRESET) != 0)
	{ // Command to RESET
		transition_disconnecting(pcf);
	}
	return;
}
/* *************************************************************************
 * static void open_contactors(struct CONTACTORFUNCTION* pcf);
 * @brief	: De-energize contactors and set time delay for opening
 * @param	: pcf = pointer to struct with "everything" for this function
 * *************************************************************************/
static void open_contactors(struct CONTACTORFUNCTION* pcf)
{
	/* Set one-shot timer for contactors opening duration. */
	if (pcf->open2_k > pcf->open1_k)
	{
		xTimerChangePeriod(pcf->swtimer2,pcf->open2_k, 2);
	} 
	else
	{
		xTimerChangePeriod(pcf->swtimer2,pcf->open1_k, 2); 
	}
	/* De-engerize both contactors and pwm'ing if on */
	pcf->outstat      &= ~(CNCTOUT00K1 | CNCTOUT01K2 | CNCTOUT06KAw | CNCTOUT07KAw);
	pcf->outstat_prev |=  (CNCTOUT00K1 | CNCTOUT01K2 | CNCTOUT06KAw | CNCTOUT07KAw);
	return;
}
/* *************************************************************************
 * static void new_state(struct CONTACTORFUNCTION* pcf, uint32_t newstate);
 * @brief	: When there is a state change, do common things
 * @param	: pcf = pointer to struct with "everything" for this function
 * @param	: newstate = code number for the new state
 * *************************************************************************/
static void new_state(struct CONTACTORFUNCTION* pcf, uint32_t newstate)
{
	pcf->outstat |= CNCTOUT05KA;	// Queue keep-alive status CAN msg
	pcf->state = newstate;
	return;
}

