/**
  ******************************************************************************
  * @file           : cdc_txbuff.h
  * @brief          : Buffering for HAL CDC 2017 12 11
  ******************************************************************************
Updates:

2018 12 30 Multiple Tasks can call, plus timer polling (allows unmodified HAL code)
   
*/
#ifndef CDC_TXBUFF_H
#define CDC_TXBUFF_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"

/* Pointers for each local buffer */
struct CDCBUFFPTR
{
   uint8_t* begin;
   uint8_t* end;
   uint8_t* work;
};

/* Queue of Buffer Control Blocks */
struct CDCTXTASKBCB
{
	uint8_t	*pbuf;             // Pointer to byte buffer to be sent
	uint32_t size;              // Number of bytes to be sent
};

/** ****************************************************************************/
  struct CDCBUFFPTR* cdc_txbuff_init(uint16_t numbuff, uint16_t size);
 /* @brief	: Setup buffer pair for CDC TX
  * @param	: numbuff = number of buffers (of size 'size')
  * @param	: size = number of bytes in each buffer
  * @return	: NULL = calloc failed; not NULL = pointer to 1st struct with buff ptrs
  ******************************************************************************/
 osMessageQId xCdcTxTaskSendCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: Handle to queue
 * *****************************************************************************/
  uint32_t cdc_txbuff_poll(void);
 /* @brief	: Start USBD sending if it is not busy
  * @return	: 0 = busy; 1 = new buffer started sending
  ******************************************************************************/
void StartSerialTaskSend(void);
/*	@brief	: Task startup
 * *****************************************************************************/

extern osMessageQId CdcTxTaskSendQHandle;

/* Macro to simplify Tasks loading queue */
#define mCdcTxQueueBuf(cdc) xQueueSendToBack(CdcTxTaskSendQHandle,cdc,5000);
#define mSerialTaskSendWait( noteval, bit){while((noteval & bit) == 0){xTaskNotifyWait(bit, 0, &noteval, 5000);}}

#ifdef THE_MACROS_ABOVE_ARE_THE_FOLLOWING_CODE_CONSOLIDATED
/* *************************************************************
 * void vSerialTaskSendWait(uint32_t* pnoteval, uint32_t bit);
 * @param	: pnoteval = pointer to task notification returned
 * @param	: bit = bit within notification for buffer
 ***************************************************************/
void vSerialTaskSendWait(uint32_t* pnoteval, uint32_t bit){
  while((*pnoteval&bit) == 0){
    xTaskNotifyWait(bit, 0, pnoteval, 3000);
  }
}
/* *************************************************************
 * void vSerialTaskSendQueueBuf(struct SERIALSENDTASKBCB* pbcb);
 * @param	: pbcb = pointer Buffer Control Block
 ***************************************************************/
void vSerialTaskSendQueueBuf(struct SERIALSENDTASKBCB* pbcb){
	uint32_t qret;
	do
	{
		qret = xQueueSendToBack(SerialTaskSendQHandle,pbcb,portMAX_DELAY);
	} while (qret == errQUEUE_FULL);
}
#endif

#endif
