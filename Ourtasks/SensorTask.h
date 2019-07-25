/******************************************************************************
* File Name          : SensorTask.h
* Date First Issued  : 06/25/2019
* Description        : Sensor function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#ifndef __CONTACTORTASK
#define __CONTACTORTASK

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "adc_idx_v_struct.h"
#include "bpsensor_idx_v_struct.h"

/* 
=========================================      
CAN msgs: 
suffix: "_i" = incoming msg; "_r" response msg

Received CAN msgs directed into contactor function:
 (1) contactor command (also keep-alive) "cid_keepalive_i"
     payload[0]
       bit 7 - connect request
       bit 6 - reset critical error
 (2) poll (time sync) "cid_gps_sync"
 (3) function command (diagnostic poll) "cid_cmd_i"
    
Sent by contactor function:
 (1) contactor command "cid_keepalive_r" (response to "cid_keepalive_i")
     payload[0]
       bit 7 - faulted (code in payload[2])
       bit 6 - warning: minimum pre-chg immediate connect.
              (warning bit only resets with power cycle)
		 bit[0]-[3]: Current main state code

     payload[2] = critical error state error code
         0 = No fault
         1 = battery string voltage (hv1) too low
         2 = contactor 1 de-energized, aux1 closed
         3 = contactor 2 de-energized, aux2 closed
         4 = contactor #1 energized, after closure delay aux1 open
         5 = contactor #2 energized, after closure delay aux2 open
         6 = contactor #1 does not appear closed
         7 = Timeout before pre-charge voltage reached cutoff
         8 = Sensor #1 closed but voltage across it too big
         9 = Sensor #2 closed but voltage across it too big

		payload[3]
         bit[0]-[3] - current substate CONNECTING code
         bit[4]-[7] - current substate (spare) code

 poll  (response to "cid_gps_sync") & heartbeat
 (2)  "cid_msg1" hv #1 : current #1  battery string voltage:current
 (3)	"cid_msg2" hv #2 : hv #3       DMOC+:DMOC- voltages

 function command "cid_cmd_r"(response to "cid_cmd_i")
 (4)  conditional on payload[0], for example(!)--
      - ADC ct for calibration purposes hv1
      - ADC ct for calibration purposes hv2
      - ADC ct for calibration purposes hv3
      - ADC ct for calibration purposes current1
      - ADC ct for calibration purposes current2
      - Duration: (Energize coil 1 - aux 1)
      - Duration: (Energize coil 2 - aux 2)
      - Duration: (Drop coil 1 - aux 1)
      - Duration: (Drop coil 2 - aux 2)
      - volts: 12v CAN supply
      - volts: 5v regulated supply
      ... (many and sundry)

 heartbeat (sent in absence of keep-alive msgs)
 (5)  "cid_hb1" Same as (2) above
 (6)  "cid_hb2" Same as (3) above

=========================================    
NOTES:
1. The command/keep-alive msgs are sent
   - As a response to every command/keep-alive
   - Immediately when the program state changes
   - Every keep-alive timer timeout when incoming keep-alive
     msgs are not being receive, i.e. becomes a status heartbeat.

2. hv3 cannot measure negative voltages which might occur during
   regeneration with contactor #2 closed.  Likewise, hv2 can 
   exceed hv1 so that the difference becomes negative.  In both
   case the negative values would be small.
*/


/* Task notification bit assignments. */
#define CNCTBIT00	(1 << 0)  // ADCTask has new readings


/* *************************************************************************/
osThreadId xSensorTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: SensorTaskHandle
 * *************************************************************************/
void StartSensorTask(void const * argument);
/*	@brief	: Task startup
 * *************************************************************************/

extern osThreadId SensorTaskHandle;

#endif

