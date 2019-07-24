/******************************************************************************
* File Name          : SensorTask.c
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
#include "SensorTask.h"
#include "sensor_idx_v_struct.h"

/* From 'main.c' */
extern UART_HandleTypeDef huart3;

static char* hex(char *p, u8 c);

osThreadId SensorTaskHandle;

struct SENSORFUNCTION sensorfunction;

/* *************************************************************************
 * osThreadId xSensorTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: SensorTaskHandle
 * *************************************************************************/
osThreadId xSensorTaskCreate(uint32_t taskpriority)
{
 	osThreadDef(SensorTask, StartSensorTask, osPriorityNormal, 0, 192);
	SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);
	vTaskPrioritySet( SensorTaskHandle, taskpriority );
	return SensorTaskHandle;
}
/* *************************************************************************
 * void StartSensorTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartSensorTask(void const * argument)
{
	/* A notification copies the internal notification word to this. */
	uint32_t noteval = 0;    // Receives notification word upon an API notify

	/* notification bits processed after a 'Wait. */
	uint32_t noteused = 0;

	/* Serial send buffer for sending readings. */
	struct SERIALSENDTASKBCB* pbuf3 = getserialbuf(&huart3,48);
	if (pbuf3 == NULL) morse_trap(31);


	/* Init struct with working params */
	sensor_idx_v_struct_hardcode_params(&sensorfunction.lc);

	/* Initialize working struc for SensorTask. */
	extern struct ADCFUNCTION adc1;
	sensor_func_init_init(pcf, &adc1);

	char c[32];
      
  /* Infinite loop */
  for(;;)
  {
		/* Wait for notifications */
		xTaskNotifyWait(noteused, 0, &noteval, portMAX_DELAY);
		noteused = 0;	// Accumulate bits in 'noteval' processed.
		if ((noteval & CNCTBIT00) != 0)
		{ // ADC readings ready
			noteused |= CNCTBIT00;  // We handled the bit
			adctoserial(&c[0]);     // Construct the line
			yputs(&c[0]); // Queue for sending
		}
		if ((noteused & ~CNCTBIT00) != 0) // Debugging jic
				morse_trap(32);
	}
}
/* *************************************************************************
 * static void adctoserial(void);
 *	@brief	: Build serial line 
 * *************************************************************************/
/*
NOTE: This is where COBS coding would be done, but we start with ascii-hex
*/

/* bin to ascii lookup table */
static const char h[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
static char* hex(char *p, u8 c)	// Convert 'c' to hex, placing in output *p.
{
		*p++ = h[((c >> 4) & 0x0f)];	// Hi order nibble
		*p++ = h[(c & 0x0f)];		// Lo order nibble
		return p;			// Return new output pointer position

static void adctoserial(char* p)
{
	char cout[32];
	char* p = &cout[0];
	
	struct ADCCHANNEL* padc = &adc1;
	
	p = hex(p, padc1->chan[ADC1IDX_HIGHVOLT1].ival >> 0);
	p = hex(p, padc1->chan[ADC1IDX_HIGHVOLT1].ival >> 8);

	p = hex(p, padc1->chan[ADC1IDX_HIGHVOLT2].ival >> 0);
	p = hex(p, padc1->chan[ADC1IDX_HIGHVOLT2].ival >> 8);

	p = hex(p, padc1->chan[ADC1IDX_HIGHVOLT3].ival >> 0);
	p = hex(p, padc1->chan[ADC1IDX_HIGHVOLT3].ival >> 8);

	*p++ = '\n'; *p = 0;
	return;
}

