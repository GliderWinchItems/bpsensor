/**
  ******************************************************************************
  * @file           : cdc_txbuff.c
  * @brief          : Buffering for HAL CDC 2017 12 11
  ******************************************************************************
Updates:
2018 12 30 Multiple Tasks can call, plus timer polling (allows unmodified HAL code)
   
Strategy:
A circular array of local buffers are created during initialization.  Tasks that
have data to send via cdc tx place a block holding a pointer to the data plus the
number of bytes on a queue.

The serial task send routine waits for blocks to be added tot he queue.  When it receives
the notification it checks for NULL pointer and zero size and calls the 'add' routine
that copies the data into the local buffer(s).  It starts the cdc tx sending if it
is not busy.  

When the timer, or modified HAL program, executes a callback the callback routine 
checks if more buffers are to be sent and initiates the sending of the next buffer.

When data is added to the local buffer and the cdc tx is busy sending the previous
local buffer, the new data is appended to the local buffer.  This allows the lower
level cdc routines to send longer runs of data.  Otherwise, it would be likely that
sending short strings would result in the usb causing 1 ms delays between each
string, thus limiting the throughput.

==> NOTE: STM32CubeMX places MX_USB_DEVICE_Init(); in "StartDefaultTask".  A time
delay is needed for the PC to recognize our usb device, e.g. 'osDelay(1000)'.

*/

#include <malloc.h>
#include "cdc_txbuff.h"
#include "usbd_cdc_if.h"

#define CDCTIMEDURATION 5	// Timer time period (ms)

/* Prototypes */
static uint32_t cdc_txbuff_add(struct CDCTXTASKBCB* p);
void cdc_txbuff_callback(void const * argument);

/* Pointers for each buffer */
static struct CDCBUFFPTR* pbuff_begin;
static struct CDCBUFFPTR* pbuff_end;
static struct CDCBUFFPTR* pbuff_m;	// Pointer to buffer that 'main' is adding to
static struct CDCBUFFPTR* pbuff_i;	// Pointer to buffer that 'interrupt' (or poll) taking from

/* Task */
#define SSPRIORITY 1	// Priority for this task (0 = Normal, -3 = Idle)

static uint32_t CdcTxTaskSendBuffer[ 64 ];
static osStaticThreadDef_t CdcTxTaskSendControlBlock;
osThreadId CdcTxTaskSendHandle = NULL;
void StartCdcTxTaskSend(void const * argument);
osTimerId CdcTxTimerHandle;

/* Queue */
#define CDCTXQUEUESIZE 8	// Total size of bcb's tasks can queue up

osMessageQId CdcTxTaskSendQHandle;
static uint8_t CdcTxTaskSendQBuffer[ CDCTXQUEUESIZE * sizeof( struct CDCTXTASKBCB ) ];
static osStaticMessageQDef_t CdcTxTaskSendQCB;

/* *************************************************************************
 * osMessageQId xCdcTxTaskSendCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: Handle to queue
 * *************************************************************************/
osThreadId xCdcTxTaskSendCreate(uint32_t taskpriority)
{
	/* definition and creation of task: CdcTxTaskSend */
	osThreadStaticDef(CdcTxTaskSend, StartCdcTxTaskSend, osPriorityNormal, 0, 64, CdcTxTaskSendBuffer, &CdcTxTaskSendControlBlock);
   CdcTxTaskSendHandle = osThreadCreate(osThread(CdcTxTaskSend), NULL);
	vTaskPrioritySet( CdcTxTaskSendHandle, taskpriority );
	if (CdcTxTaskSendHandle == NULL) return NULL;

	/* FreeRTOS queue for task with data to send. */
  osMessageQStaticDef(CdcTxSendQ, CDCTXQUEUESIZE, struct CDCTXTASKBCB, CdcTxTaskSendQBuffer, &CdcTxTaskSendQCB);
  CdcTxTaskSendQHandle = osMessageCreate(osMessageQ(CdcTxSendQ), NULL);

  /* definition and creation of CdcTxTimer */
  osTimerDef(CdcTxTim, cdc_txbuff_callback);
  CdcTxTimerHandle = osTimerCreate(osTimer(CdcTxTim), osTimerPeriodic, NULL);

	/* Start timer callback polling */
	osTimerStart (CdcTxTimerHandle, CDCTIMEDURATION);	

	return CdcTxTaskSendQHandle;
}
/* *****************************************************************************
  Advance buffer pointer
********************************************************************************/
static struct CDCBUFFPTR* step_ptr(struct CDCBUFFPTR* pb)
{
		pb++;	//Step to next struct of pointers for a buffer
		if (pb >= pbuff_end)
			pb = pbuff_begin; // Wrap around
		return pb;
}
/* *****************************************************************************
   Get buffer space and init pointers
********************************************************************************/
static uint8_t* pbuff_init(struct CDCBUFFPTR* pb, uint16_t size)
{
  pb->begin = calloc(size, sizeof(uint8_t));
  pb->work  = pb->begin;
  pb->end   = pb->begin + size;
  return pb->begin;
}
/** ****************************************************************************
  * struct CDCBUFFPTR* cdc_txbuff_init(uint16_t numbuff, uint16_t size);
  * @brief	: Setup buffer pair for CDC TX
  * @param	: numbuff = number of buffers (of size 'size')
  * @param	: size = number of bytes in each buffer
  * @return	: NULL = calloc failed; not NULL = pointer to 1st struct with buff ptrs
  ******************************************************************************
  */
struct CDCBUFFPTR* cdc_txbuff_init(uint16_t numbuff, uint16_t size)
{
	/* Minimum of 2 buffers required */
	if (numbuff < 2)
	{ // Here, we force 2, but we *could* just bomb, but what if the hapless Programmer
		// doesn't check the return!
		numbuff = 2; 
	}
	/* Miminum of one char for the buffer (is this a duh?) */
	if (size == 0)
	{
		size = 1;
	}

	struct CDCBUFFPTR* pb = calloc(numbuff, sizeof(struct CDCBUFFPTR));
	if (pb == NULL)  return NULL;
	pbuff_begin = pb;
	pbuff_end = pbuff_begin;
	pbuff_end += numbuff;

	/* Get memory for buffers, and init pointers for each */
	while (pb != pbuff_end)
	{
	   if (pbuff_init(pb++, size) == NULL) return NULL;
   }

	/* Init pointers for adding/taking */
	pbuff_m = pbuff_begin;	// Adding
	pbuff_i = pbuff_begin;	// Taking
	pbuff_i += numbuff;		//  (initially one behind Adding)

   return pbuff_m;	// Probably not used except for debugging
}
/** ****************************************************************************
  * uint32_t cdc_txbuff_poll(void);
  * @brief	: Start USBD sending if it is not busy
  * @return	: 0 = busy; 1 = new buffer started sending; 2 = no new data to send
  *****************************************************************************
  */
static uint32_t poll(void);
static volatile uint32_t cdcaddbusy = 0;	// 0 = cdc_txbuff_add routine not active; 1 = in process

uint32_t cdc_txbuff_poll(void)
{
	cdcaddbusy = 1;	// Show callback that adding data is active
	uint32_t ret = poll();
	cdcaddbusy = 0;	// Done adding	
	return ret;
}

uint32_t cdcct5;

/* static version called by callback */
static uint32_t poll(void)
{
	struct CDCBUFFPTR* pbuff_tmp;

	/* Return if caught up with buffer ptrs AND no new data */
	pbuff_tmp = step_ptr(pbuff_i);	// Advance last sent ptr JIC
	if ((pbuff_m == pbuff_tmp)	&& ((pbuff_m->work - pbuff_m->begin) == 0)) return 2;

	/* Start next buffer sending if USBD is ready */
	if ((CDC_Transmit_FS( (uint8_t*)pbuff_tmp->begin, (pbuff_tmp->work - pbuff_tmp->begin))) == USBD_OK)
	{ // Here, not busy, AND sending of next-buffer-to-send has been started
cdcct5 +=1;	// DEBUG: Count number of buffer "sends"
		pbuff_i = pbuff_tmp;	// Save buffer pointer being sent
		pbuff_i->work = pbuff_i->begin;  // Reset char pointer for buffer just sent
		if (pbuff_m == pbuff_i)
		{
			pbuff_m = step_ptr(pbuff_m);
		}
		return 1;
	}
	return 0;	
}
/* *************************************************************************
 * void StartSerialTaskSend(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartCdcTxTaskSend(void const * argument)
{
	BaseType_t Qret;	// queue receive return
	struct CDCTXTASKBCB   ssb; // Copied item from queue

  /* Infinite RTOS Task loop */
  for(;;)
  {
		do
		{
		/* Wait indefinitely for someone to load something into the queue */
		/* Skip over empty returns, and NULL pointers that would cause trouble */
			Qret = xQueueReceive(CdcTxTaskSendQHandle,&ssb,portMAX_DELAY);
			if (Qret == pdPASS) // Break loop if not empty
				break;
		} while ( (ssb.pbuf == NULL) || (ssb.size == 0));
	   cdc_txbuff_add(&ssb);  // Add data to local cdc tx buffer
  }
  return;
}
		
/** ****************************************************************************
  * static uint32_t cdc_txbuff_add(struct CDCTXTASKBCB* p);
  * @brief	: Add chars to buffer and start USBD TX if not busy
  * @param	: p = pointer data/size to be added
  * @return	: number chars added to buffer;
  ******************************************************************************
  */
uint32_t cdcct3;
uint32_t cdcct4;
uint32_t cdcT0;
uint32_t cdcMax;

static uint32_t cdc_txbuff_add(struct CDCTXTASKBCB* p)
{
	cdcaddbusy = 1;	// Show callback that adding data is active
	uint16_t csize = 0;	// Count of chars added

	/* Add data to current buffer */
 	while (p->size > 0)
   {
		// Copy into current "add-to" buffer
		*pbuff_m->work++ = *p->pbuf++;
		p->size -= 1;	// Count down chars copied

		if (pbuff_m->work == pbuff_m->end) // End of local buffer?
		{ // Here, yes
			pbuff_m = step_ptr(pbuff_m);	// Advance ptr to next buff
			pbuff_m->work = pbuff_m->begin; // Start at beginning 
			if (pbuff_m == pbuff_i)
			{ // Here, we point to output buffer; about to OVERRUN
cdcct4 += 1;	// DEBUG: Count number of instances
#include "DTW_counter.h"
cdcT0 = DTWTIME;	// DEBUG: Time wasted in loop
				while(poll() == 0)	// Loop until buffer is free   
				{
cdcct3+=1;	// DEBUG: count loops
					
				}
// DEBUG: time wasted in loop
cdcT0 = DTWTIME - cdcT0;
cdcMax += cdcT0;
			}
		}
		csize += 1;
	}

	/* All chars in the input string were added to the buffer. */
	cdc_txbuff_poll();	// Start sending if not already sending   

	cdcaddbusy = 0;	// Done adding	
	return csize;
}
/** ############################################################################
  * void cdc_txbuff_callback(void const * argument);
  * @brief	: Send next buffer, if available NOTE: routine is under CDC interrupt
  ##############################################################################
  */
/*
 For modified HAL:
	This routine is entered when the USB is complete and the busy has been set 
   to zero. 
	Entry is from 'usbd_cdc.c'  See routine beginning at line 664.

 For unmodified HAL, use timer polling:
	Entry is from software timer callback (in 'main')
*/
int cdcct1 = 0;
int cdcct2 = 0;

void cdc_txbuff_callback(void const * argument)
{
	if (cdcaddbusy != 0)
	{ // Here, 'cdc_txbuff_add' is busy adding chars to a buffer
cdcct2 += 1;

		return;
	}
cdcct1 += 1;
	poll();	// Send next buffer, if available
	return;
}



