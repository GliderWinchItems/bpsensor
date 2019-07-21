/******************************************************************************
* File Name          : MailboxTask.c
* Date First Issued  : 02/20/2019
* Description        : Incoming CAN msgs to Mailbox
*******************************************************************************/

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_can.h"
#include "CanTask.h"
#include "MailboxTask.h"
#include "morse.h"
#include "DTW_counter.h"
#include "payload_extract.h"
#include "GatewayTask.h"

extern osThreadId GatewayTaskHandle;

/* One struct for each CAN module, e.g. CAN 1, 2, 3, ... */
struct MAILBOXCANNUM mbxcannum[STM32MAXCANNUM] = {0};

osThreadId MailboxTaskHandle; // This wonderful task handle

void StartMailboxTask(void const * argument);
static struct MAILBOXCAN* loadmbx(struct MAILBOXCANNUM* pmbxnum, struct CANRCVBUFN* pncan);

/* *************************************************************************
 * struct MAILBOXCANNUM* MailboxTask_add_CANlist(struct CAN_CTLBLOCK* pctl, uint16_t arraysize);
 *	@brief	: Add CAN module mailbox list
 * @param	: pctl = Pointer to CAN control block
 * @param	: arraysize = max number of mailboxes in sorted list
 * @return	: Pointer which probably will not be used; NULL = failed (more important)
 * NOTE: This is normally called in 'main' before the FreeRTOS scheduler starts.
 * *************************************************************************/
struct MAILBOXCANNUM* MailboxTask_add_CANlist(struct CAN_CTLBLOCK* pctl, uint16_t arraysize)
{
	struct MAILBOXCAN** ppmbxarray; // Pointer to array of pointers to mailboxes

	if (pctl == NULL) morse_trap(21); // Oops

	if (arraysize == 0) morse_trap(22); // Oops

taskENTER_CRITICAL();

	/* Save max number of mailboxes for this CAN module */
	mbxcannum[pctl->canidx].arraysizemax = arraysize;

	/* This needed to find the CAN module in 'StartMailboxTask' */
	mbxcannum[pctl->canidx].pctl = pctl;

	/* Get memory for the array of mailbox pointers */
	ppmbxarray = (struct MAILBOXCAN**)calloc(arraysize, sizeof(struct MAILBOXCAN*));
	if (ppmbxarray == NULL) {taskEXIT_CRITICAL(); morse_trap(23);}

	/* xMailboxTaskCreate needs to be called before this 'add to list' */
	if (MailboxTaskHandle == NULL) {taskEXIT_CRITICAL();morse_trap(24);}

	/* Get a circular buffer 'take' pointer for this CAN module. */
	// The first three notification bits are reserved for CAN modules 
	mbxcannum[pctl->canidx].ptake = can_iface_mbx_init(pctl, MailboxTaskHandle, (1 << pctl->canidx) );

	/* Save pointer to array of pointers to mailboxes. */
	mbxcannum[pctl->canidx].pmbxarray = ppmbxarray;

	/* Save number of mailbox pointers */
	mbxcannum[pctl->canidx].arraysizemax = arraysize; // Max

	/* Start with no mailboxes created. */
	mbxcannum[pctl->canidx].arraysizecur = 0;

	/* What is important to return a non-NULL pointer to show success. */
taskEXIT_CRITICAL();
	return &mbxcannum[pctl->canidx];
}
/* *************************************************************************
 *  struct CANNOTIFYLIST* MailboxTask_disable_notifications(struct MAILBOXCAN* pmbx);
 *  struct CANNOTIFYLIST* MailboxTask_enable_notifications (struct MAILBOXCAN* pmbx);
 *	@brief	: Disable, enable mailbox notifications
 * @param	: pmbx = pointer to mailbox
 * @return	: Pointer to notification block, for calling task; NULL = task not found
 * *************************************************************************/
static struct CANNOTIFYLIST* noteskip(struct MAILBOXCAN* pmbx, uint8_t skip)
{
	osThreadId tskhandle = xTaskGetCurrentTaskHandle();
	struct CANNOTIFYLIST* pnotetmp;	
	struct CANNOTIFYLIST* pnotex;	

	// Traverse linked list to find task
	pnotetmp = pmbx->pnote;	// Ptr to head of list
	if (pnotetmp == NULL) return NULL; // No notifications setup!
	do 
	{
		if (tskhandle == pnotetmp->tskhandle)
		{ // Notification for "this" task found
			pnotetmp->skip = skip; // Update 'skip' flag
			return pnotetmp; // Ptr to notification struct
		}
			pnotex   = pnotetmp;
			pnotetmp = pnotetmp->pnext;
	} while (pnotetmp != pnotex);
	return NULL; // Here, the current running task not found
}
struct CANNOTIFYLIST* MailboxTask_disable_notifications(struct MAILBOXCAN* pmbx)
{
	return noteskip(pmbx, 1);
}
struct CANNOTIFYLIST* MailboxTask_enable_notifications(struct MAILBOXCAN* pmbx)
{
	return noteskip(pmbx, 0);
}

/* *************************************************************************
 * struct MAILBOXCAN* MailboxTask_add(struct CAN_CTLBLOCK* pctl,\
		 uint32_t canid,\
       osThreadId tskhandle,\
		 uint32_t notebit,\
		 uint8_t noteskip,\
		 uint8_t paytype);
 *	@brief	: Add a mailbox, given CAN control block ptr, and other stuff
 * @param	: pctl = Pointer to CAN control block, i.e. CAN module/CAN bus, for mailbox
 * @param	: canid = CAN ID
 * @param	: tskhandle = Task handle; NULL for use current task; 
 * @param	: notebit = notification bit; NULL = no notification
 * @paran	: noteskip = notify = 0; skip notification = 1;
 * @param	: paytype = payload type code (see 'PAYLOAD_TYPE_INSERT.sql' in 'GliderWinchCommons/embed/svn_common/db')
 * @return	: Pointer to mailbox; NULL = failed
 * *************************************************************************/
struct MAILBOXCAN* MailboxTask_add(struct CAN_CTLBLOCK* pctl,\
		 uint32_t canid,\
       osThreadId tskhandle,\
		 uint32_t notebit,\
		 uint8_t noteskip,\
		 uint8_t paytype)
{
	int j;
	struct MAILBOXCAN* pmbx;
	struct CANNOTIFYLIST* pnotex;
	struct CANNOTIFYLIST* pnotetmp;
	struct MAILBOXCAN** ppmbx;

	/* Check that the bozo programmer got the prior initializations done correctly. */
	if (canid == 0)    return NULL;
	if (pctl  == NULL) return NULL;
	if (pctl->canidx >= STM32MAXCANNUM) return NULL;
	if (mbxcannum[pctl->canidx].pctl == NULL) return NULL;

	if (tskhandle == NULL)
		tskhandle = xTaskGetCurrentTaskHandle();

	/* Pointer to beginning of array of mailbox pointers. */
	ppmbx = mbxcannum[pctl->canidx].pmbxarray;

taskENTER_CRITICAL();

	/* We are working with the array of pointers to mailboxes. */
	// Check if this 'canid' has a mailbox
	for (j = 0; j < mbxcannum[pctl->canidx].arraysizecur; j++)
	{
		pmbx = *(ppmbx+j);  // Get pointer to a mailbox from array of pointers
		if (pmbx == NULL) morse_trap(20); // jic|debug
		if (pmbx->ncan.can.id == canid)
		{ // Here, CAN id already has a mailbox, so a notification must be wanted by this task
			if (notebit != 0)
			{ // Here add a notification to the existing mailbox

				/* Get a notification block. */
				pnotex = (struct CANNOTIFYLIST*)calloc(1, sizeof(struct CANNOTIFYLIST));
				if (pnotex == NULL){ taskEXIT_CRITICAL();return NULL;}

				/* Check if this mailbox has any notifications */
				if (pmbx->pnote == NULL)
				{ // This is the first notification for this mailbox.
					pmbx->pnote       = pnotex;   // Mailbox points to first notification
					pnotex->pnext     = pnotex;	// Last on list points to self
					pnotex->tskhandle = tskhandle;
 					pnotex->notebit   = notebit;  // Notification bit to use
					pnotex->skip      = noteskip; // Skip notification flag
					/* Here, there is no need to sort array on CANID for a binary lookup
						since a new mailbox was not added. */
					taskEXIT_CRITICAL();
					return pmbx;
				}
				else
				{ // Here, one of more notifications.  Add to list.
					/* Seach end of list */
					pnotetmp = pmbx->pnote;
					while (pnotetmp != pnotetmp->pnext) pnotetmp = pnotetmp->pnext;

					/* Add to list and initialize. */
					pnotetmp->pnext   = pnotex; // End block now points to new block
					pnotex->pnext     = pnotex; // New block points to self
					pnotex->tskhandle = tskhandle;
 					pnotex->notebit   = notebit;  // Notification bit to use
					pnotex->skip      = noteskip; // Skip notification flag
					/* Here, there is no need to sort array on CANID for a binary lookup
						since a new mailbox was not added. */
					taskEXIT_CRITICAL();
					return pmbx;
				}
			}
			/* Here, no notification bit, but CAN id already has a mailbox!
            Either the canid is wrong, or this call was not necessary. */
			taskEXIT_CRITICAL();
			return NULL;
		}
	}

	/* Here, a mailbox for 'canid' was not found in the list.  
      Or, this is the first mailbox created.                  

      Create a mailbox for this canid                         */

	// Point to next available location in array of mailbox pointers.
   //                 pointer to beginning   + number of entries
	ppmbx = mbxcannum[pctl->canidx].pmbxarray + mbxcannum[pctl->canidx].arraysizecur;

	/* Create one mailbox */
	pmbx = (struct MAILBOXCAN*)calloc(1, sizeof(struct MAILBOXCAN));
	if (pmbx == NULL){ taskEXIT_CRITICAL();return NULL;}

	pmbx->ctr   = 0;       // Redundant (calloc set it zero)
	pmbx->pnote = NULL;    // Redundant (calloc set it zero)
	pmbx->ncan. can.id = canid;   // Save CAN id
	pmbx->ncan.toa     = DTWTIME; // Set current time for initial time-of-arrival

	if (notebit != 0)
	{ // Here, a notification is requested.  Add first instance of notification  
		pnotex = (struct CANNOTIFYLIST*)calloc(1, sizeof(struct CANNOTIFYLIST));
		if (pnotex == NULL){ taskEXIT_CRITICAL();return NULL;}

		pmbx->pnote       = pnotex; // Mailbox points to first notification
		pnotex->pnext     = pnotex;	// Last on list points to self
		pnotex->tskhandle = tskhandle; // Task to notify
		pnotex->notebit   = notebit;  // Notification bit to use
		pnotex->skip      = noteskip; // Skip notification flag
	}

// TODO: Sort pointers for new Mailbox if later binary lookup on CAN ID.

taskEXIT_CRITICAL();
	return pmbx;
}

/* *************************************************************************
 * osThreadId xMailboxTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: QueueHandle_t = queue handle
 * *************************************************************************/
osThreadId xMailboxTaskCreate(uint32_t taskpriority)
{
 /* definition and creation of CanTask */
  osThreadDef(MailboxTask, StartMailboxTask, osPriorityNormal, 0, 96);

  MailboxTaskHandle = osThreadCreate(osThread(MailboxTask), NULL);

	vTaskPrioritySet( MailboxTaskHandle, taskpriority );
	return MailboxTaskHandle;
}
/* *************************************************************************
 * void StartMailboxTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartMailboxTask(void const * argument)
{
	struct MAILBOXCANNUM* pmbxnum;
	struct CANRCVBUFN* pncan;
	struct CANTAKEPTR* ptake[STM32MAXCANNUM];
	int i;
//	int8_t flag;

//while(1==1) osDelay(10); // Debug: make task do nothing

	/* Get circular buffer pointers for each CAN module in list. */	
	for (i = 0; i < STM32MAXCANNUM; i++)
	{
		if (mbxcannum[i].pmbxarray != NULL)
		{ // Here, array of pointers was initialized
			ptake[i] = can_iface_mbx_init(mbxcannum[i].pctl, NULL, (1 << i));
			if (ptake[i] == NULL) morse_trap(22);
		}
	}

	/* A notification copies the internal notification word to this. */
	uint32_t noteval = 0;    // Receives notification word upon an API notify

	/* notification bits processed after a 'Wait. */
	uint32_t noteused = 0;

  /* Infinite MailboxTask loop */
  for(;;)
  {
		/* Wait for a CAN module to load its circular buffer. */
		/* The notification bit identifies the CAN module. */
		xTaskNotifyWait(noteused, 0, &noteval, portMAX_DELAY);
		noteused = 0;	// Accumulate bits in 'noteval' processed.

		/* Step through possible notification bits */
		for (i = 0; i < STM32MAXCANNUM; i++)
		{
//			flag = 0;
			if ((noteval & (1 << i)) != 0)
			{	
				noteused |= (1 << i);
				pmbxnum = &mbxcannum[i]; // Pt to CAN module mailbox control block
if (pmbxnum == NULL) morse_trap(77); // Debug trap
				do
				{
					/* Get a pointer to the circular buffer w CAN msgs. */
					pncan = can_iface_get_CANmsg(pmbxnum->ptake);

					if (pncan != NULL)
					{ // Here, CAN msg is available
//						flag = 1;
						loadmbx(pmbxnum, pncan); // Load mailbox. if CANID is in list
					}
				} while (pncan != NULL);

  #ifdef GATEWAYTASKINCLUDED
				/* Notify GatewayTask that one or more CAN msgs in circular buffer. */
				if ( (GatewayTaskHandle != NULL) && ((noteval & (1 << i)) != 0) && (flag != 0) )
				{
					xTaskNotify(GatewayTaskHandle, (1 << i), eSetBits);
				}
  #endif
			}
		}
  }
}
/* *************************************************************************
 * static struct MAILBOXCAN* lookup(struct MAILBOXCANNUM* pmbxnum, struct CANRCVBUFN* pncan);
 *	@brief	: (Bonehead) Lookup CAN ID by a straight pass down the array of mailbox pointers
 * @param	: pmbxnum = pointer to mailbox control block
 * @param	: pncan = pointer to CAN msg in can_face.c circular buffer
 * *************************************************************************/
static struct MAILBOXCAN* lookup(struct MAILBOXCANNUM* pmbxnum, struct CANRCVBUFN* pncan)
{
	struct MAILBOXCAN** ppmbx;
	struct MAILBOXCAN*   pmbx;
	int i;

	ppmbx = pmbxnum->pmbxarray;
	for (i = 0; i < pmbxnum->arraysizecur; i++)
	{
		pmbx = *(ppmbx + i); // Point to mailbox[i]
		if (pmbx->ncan.can.id == pncan->can.id)
		{ // Here, found!
			return pmbx;
		}
	}
	return NULL;
}

/* ************************************************************************* 
 * static struct MAILBOXCAN loadmbx(struct MAILBOXCANNUM* pmbxnum, struct CANRCVBUFN* pncan);
 *	@brief	: Lookup CAN ID and load mailbox with extract payload reading(s)
 * @param	: pmbxnum = pointer to mailbox control block
 * @param	: pncan = pointer to CAN msg in can_face.c circular buffer
 * *************************************************************************/
static struct MAILBOXCAN* loadmbx(struct MAILBOXCANNUM* pmbxnum, struct CANRCVBUFN* pncan)
{
	struct CANNOTIFYLIST* pnotetmp;	
	struct CANNOTIFYLIST* pnotex;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Check if received CAN id is in the mailbox CAN id list. */
	// 'lookup' is a straight loop; use 'lookupq' for binary search (when implemented)
	struct MAILBOXCAN* pmbx = lookup(pmbxnum, pncan);
	if (pmbx == NULL) return NULL; // Return: CAN id not in mailbox list

	/* Here, this CAN msg has a mailbox. */
	// Copy CAN msg into mailbox, and extract payload
	payload_extract(pmbx, pncan);

	/* Execute notifications */
	pnotetmp = pmbx->pnote; // Get ptr to head of linked list
	if (pnotetmp == NULL) return pmbx; // CANID found, but no notifications
	
	// Traverse linked list making notifications
	do 
	{
		/* Make a notification if "not skip" and 'taskhandle and 'notebit' were setup */
		if ((pnotetmp->skip == 0) && (pnotetmp->tskhandle != NULL) && (pnotetmp->notebit != 0))
		{
			xTaskNotifyFromISR(pnotetmp->tskhandle,\
				pnotetmp->notebit, eSetBits,\
				&xHigherPriorityTaskWoken );	
		}

		/* Step to next item in list. */
			pnotex   = pnotetmp;
			pnotetmp = pnotetmp->pnext;

	} while (pnotetmp != pnotex);

	return pmbx;
}



