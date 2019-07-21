/******************************************************************************
* File Name          : ContactorTask.c
* Date First Issued  : 06/18/2019
* Description        : Contactor function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "ADCTask.h"
#include "adctask.h"
#include "morse.h"
#include "SerialTaskReceive.h"
#include "ContactorTask.h"
#include "contactor_idx_v_struct.h"
#include "ContactorEvents.h"
#include "ContactorStates.h"
#include "ContactorUpdates.h"
#include "contactor_func_init.h"

/* From 'main.c' */
extern UART_HandleTypeDef huart3;

#define SWTIM1 500

osThreadId ContactorTaskHandle;

struct CONTACTORFUNCTION contactorfunction;

/* *************************************************************************
 * void swtim1_callback(TimerHandle_t tm);
 * @brief	: Software timer 1 timeout callback
 * *************************************************************************/
static void swtim1_callback(TimerHandle_t tm)
{
	xTaskNotify(ContactorTaskHandle, CNCTBIT04, eSetBits);
	return;
}
/* *************************************************************************
 * void swtim2_callback(TimerHandle_t tm);
 * @brief	: Software timer 2 timeout callback
 * *************************************************************************/
static void swtim2_callback(TimerHandle_t tm)
{
	xTaskNotify(ContactorTaskHandle, CNCTBIT05, eSetBits);
	return;
}
/* *************************************************************************
 * void swtim3_callback(TimerHandle_t tm);
 * @brief	: Software timer 3 timeout callback
 * *************************************************************************/
static void swtim3_callback(TimerHandle_t tm)
{
	xTaskNotify(ContactorTaskHandle, CNCTBIT03, eSetBits);
	return;
}
/* *************************************************************************
 * osThreadId xContactorTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: ContactorTaskHandle
 * *************************************************************************/
osThreadId xContactorTaskCreate(uint32_t taskpriority)
{
 	osThreadDef(ContactorTask, StartContactorTask, osPriorityNormal, 0, 192);
	ContactorTaskHandle = osThreadCreate(osThread(ContactorTask), NULL);
	vTaskPrioritySet( ContactorTaskHandle, taskpriority );
	return ContactorTaskHandle;
}
/* *************************************************************************
 * void StartContactorTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartContactorTask(void const * argument)
{
	/* Convenience pointer. */
	struct CONTACTORFUNCTION* pcf = &contactorfunction;

	/* A notification copies the internal notification word to this. */
	uint32_t noteval = 0;    // Receives notification word upon an API notify

	/* notification bits processed after a 'Wait. */
	uint32_t noteused = 0;

	/* Setup serial receive for uart (HV sensing) */
	/* Get buffer control block for incoming uart lines. */
	// 8 line buffers of 16 chars, no dma buff, char-by-char line mode
	pcf->prbcb3  = xSerialTaskRxAdduart(&huart3,0,CNCTBIT01,&noteval,8,16,0,0);
	if (pcf->prbcb3 == NULL) morse_trap(47);

	/* Init struct with working params */
	contactor_idx_v_struct_hardcode_params(&contactorfunction.lc);

	/* Initialize working struc for ContactorTask. */
	extern struct ADCFUNCTION adc1;
	contactor_func_init_init(pcf, &adc1);
      
	/* Create timer for keep-alive.  Auto-reload/periodic */
	pcf->swtimer1 = xTimerCreate("swtim1",pcf->ka_k,pdTRUE,\
		(void *) 0, swtim1_callback);
	if (pcf->swtimer1 == NULL) {morse_trap(41);}
	/* Create timer for other delays. One-shot */
	pcf->swtimer2 = xTimerCreate("swtim2",10,pdFALSE,\
		(void *) 0, &swtim2_callback);
	if (pcf->swtimer2 == NULL) {morse_trap(42);}

	/* Create timer uart RX keep-alive. One-shot */
	pcf->swtimer3 = xTimerCreate("swtim3",10,pdFALSE,\
		(void *) 0, &swtim3_callback);
	if (pcf->swtimer3 == NULL) {morse_trap(43);}

	/* Start command/keep-alive timer */
	BaseType_t bret = xTimerReset(pcf->swtimer1, 10);
	if (bret != pdPASS) {morse_trap(44);}

//while(1==1) osDelay(10); // Some debugging & testing

  /* Infinite loop */
  for(;;)
  {
		/* Wait for notifications */
		xTaskNotifyWait(noteused, 0, &noteval, portMAX_DELAY);
		noteused = 0;	// Accumulate bits in 'noteval' processed.
  /* ========= Events =============================== */
// NOTE: this could be made into a loop that shifts 'noteval' bits
// and calls from table of addresses.  This would have an advantage
// if the high rate bits are shifted out first since a test for
// no bits left could end the testing early.
		// Check notification and deal with it if set.
		if ((noteval & CNCTBIT00) != 0)
		{ // ADC readings ready
			noteused |= CNCTBIT00; // We handled the bit
			ContactorEvents_00(pcf);
		}
		else if ((noteval & CNCTBIT01) != 0)
		{ // uart RX line ready
			noteused |= CNCTBIT01; // We handled the bit
			ContactorEvents_01(pcf);
		}
		else if ((noteval & CNCTBIT02) != 0)
		{ // (spare)
			noteused |= CNCTBIT02; // We handled the bit
		}
		else if ((noteval & CNCTBIT03) != 0)
		{ // TIMER3: uart RX keep alive timed out
			noteused |= CNCTBIT03; // We handled the bit
			ContactorEvents_03(pcf);			
		}
		else if ((noteval & CNCTBIT04) != 0)
		{ // TIMER1: Command Keep Alive duration tick
			noteused |= CNCTBIT04; // We handled the bit
			ContactorEvents_04(pcf);
		}
		else if ((noteval & CNCTBIT05) != 0)
		{ // TIMER2: Multiple use Delay timed out
			noteused |= CNCTBIT05; // We handled the bit
		}
		else if ((noteval & CNCTBIT06) != 0) 
		{ // CAN: cid_cmd_i 
			noteused |= CNCTBIT06; // We handled the bit
			ContactorEvents_06(pcf);
		}
		else if ((noteval & CNCTBIT07) != 0) 
		{ // CAN: cid_keepalive_i 
			noteused |= CNCTBIT07; // We handled the bit
			ContactorEvents_07(pcf);
		}
		else if ((noteval & CNCTBIT08) != 0) 
		{ // CAN: cid_gps_sync 
			noteused |= CNCTBIT08; // We handled the bit
			ContactorEvents_08(pcf);
		}
  /* ========= States =============================== */
		if (pcf->evstat == CNCTEVTIMER1)	// Show that TIMER1 timed out
		{
			transition_faulting(pcf,KEEP_ALIVE_TIMER_TIMEOUT);
		}

		switch (pcf->state)
		{
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
			ContactorStates_reset(pcf);
			break;
		case DISCONNECTING:
			ContactorStates_disconnecting(pcf);
			break;
		}
  /* ========= Update outputs ======================= */
		ContactorUpdates(pcf);
  }
}

