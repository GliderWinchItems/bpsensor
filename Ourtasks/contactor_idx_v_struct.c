/******************************************************************************
* File Name          : contactor_idx_v_struct.c
* Date First Issued  : 06/26/2019
* Board              :
* Description        : Load parameter struct 
*******************************************************************************/

#include "contactor_idx_v_struct.h"
#include "SerialTaskReceive.h"


/* *************************************************************************
 * void contactor_idx_v_struct_hardcode_params(struct struct CONTACTORLC* p);
 * @brief	: Init struct from hard-coded parameters (rather than database params in highflash)
 * @return	: 0
 * *************************************************************************/
void contactor_idx_v_struct_hardcode_params(struct CONTACTORLC* p)
{ /*
Copied for convenience--

struct CONTACTORLC
 {
	uint32_t size;			// Number of items in struct
 	uint32_t crc;			// crc-32 placed by loader
	uint32_t version;		// struct version number
	float fdiffb4;       // hv1-hv2 voltage difference before closing (volts)
	float fdiffafter;    // allowable hv1-hv2 voltage difference after closure (volts)
	uint32_t prechgmax_t;// allowable delay for diffafter to reach closure point (timeout delay ms)
	uint32_t close1_t;   // contactor #1 coil energize-closure (timeout delay ms)
	uint32_t close2_t;   // contactor #2 coil energize-closure (timeout delay ms)
	uint32_t open1_t;    // contactor #1 coil de-energize-open (timeout delay ms)
	uint32_t open2_t;    // contactor #2 coil de-energize-open (timeout delay ms)
	uint32_t auxoc1_t;   // aux open/close diff from contactor coil #1 (duration ms)
	uint32_t auxoc2_t;   // aux open/close diff from contactor coil #2 (duration ms)
	uint32_t hv2stable_t;// hv 2 reading stable after closer (duration ms)
	uint32_t keepalive_t;// keep-alive timeout (timeout delay ms)
	uint32_t hbct1_t;		// Heartbeat ct: ticks between sending msgs hv1:cur1
	uint32_t hbct2_t;		// Heartbeat ct: ticks between sending msgs hv2:cur2
	// Calibrations (offset, scale)
	struct CNTCTCALF fcalhv1;  // Battery_minus-to-contactor #1 Battery_plus
	struct CNTCTCALF fcalhv2;  // Battery_minus-to-contactor #1 DMOC_plus
	struct CNTCTCALF fcalhv3;  // Battery_minus-to-contactor #2 DMOC_minus

   // Send CAN ids
	uint32_t cid_hb1;    // CANID-Heartbeat msg volt1:cur1 (volts:amps)
	uint32_t cid_hb2;    // CANID-Heartbeat msg volt2:cur2 (volts:amps)
   uint32_t cid_msg1;   // CANID-contactor poll response msg: volt1:cur1 (volts:amps)
   uint32_t cid_msg1;   // CANID-contactor poll response msg: volt2:cur2 (volts:amps)
	uint32_t cid_cmd_r;  // CANID_CMD_CNTCTR1R
	uint32_t cid_keepalive_r; // CANID-keepalive response (status)

   // Receive CAN ids List of CAN ID's for setting up hw filter
	uint32_t cid_cmd_i;       // CANID_CMD: incoming command
	uint32_t cid_keepalive_i;// CANID-keepalive connect command
	uint32_t cid_gps_sync;    // CANID-GPS time sync msg (poll msg)
	uint32_t code_CAN_filt[CANFILTMAX-3];// Spare
 };

*/
	p->size       = 30;
	p->crc        = 0;
   p->version    = 1;

	/* Bits that define the hw features. */
	p->hwconfig   = 0;  // None of the additional hw features!



	p->ka_t       = 1500; // Command/Keep-alive CAN msg timeout duration.
	p->ddiffb4    = 15.0; // hv1-hv2 voltage difference before closing (volts)
	p->fdiffafter = 2.0;  // allowable hv1-hv2 voltage difference after closure (volts)
	p->prechgmax_t= 6500; // allowable delay for diffafter to reach closure point (timeout delay ms)
	p->close1_t   = 25;   // contactor #1 coil energize-closure (timeout delay ms)
	p->close2_t   = 25;   // contactor #2 coil energize-closure (timeout delay ms)
	p->open1_t    = 15;   // contactor #1 coil de-energize-open (timeout delay ms)
	p->open2_t    = 15;   // contactor #2 coil de-energize-open (timeout delay ms)
	p->hv2stable_t=  5;   // hv 2 reading stable after closure (duration ms)
	p->keepalive_t= 750;  // keep-alive timeout (timeout delay ms)
	p->hbct1_t    = 1000; // Heartbeat ct: ticks between sending msgs hv1:cur1
	p->hbct2_t    = 1000; // Heartbeat ct: ticks between sending msgs hv2:cur2

	// Battery_minus-to-contactor #1
	p->calhv[IDXHV1].iir.k     = 3;
	p->calhv[IDXHV1].iir.scale = 2;
 	p->calhv[IDXHV1].adchv  = 420.0; // Applied voltage
	p->calhv[IDXHV1].offset = 53761; // ADC reading (received from uart)
	p->calhv[IDXHV1].dvcal  = p->calhv[IDXHV1].adchv / p->calhv[IDXHV1].offset; // volts/per ADCtick

	// Battery_minus-to-contactor #1 DMOC_plus
	p->calhv[IDXHV2].iir.k     = 3;
	p->calhv[IDXHV2].iir.scale = 2;
 	p->calhv[IDXHV2].adchv  = 420.0; // Applied voltage
	p->calhv[IDXHV2].offset = 53761; // ADC reading (received from uart)
	p->calhv[IDXHV2].dvcal  = p->calhv[IDXHV2].adchv / p->calhv[IDXHV2].offset; // volts/per ADCtick

	// Battery_minus-to-contactor #1 DMOC_minus
	p->calhv[IDXHV3].iir.k     = 3;
	p->calhv[IDXHV3].iir.scale = 2;
 	p->calhv[IDXHV3].adchv  = 420.0; // Applied voltage
	p->calhv[IDXHV3].offset = 53761; // ADC reading (received from uart)
	p->calhv[IDXHV3].dvcal  = p->calhv[IDXHV3].adchv / p->calhv[IDXHV3].offset; // volts/per ADCtick

   //                 CANID_HEX      CANID_NAME       CAN_MSG_FMT     DESCRIPTION
	p->cid_hb1        = 0xFF800000; // CANID_HB_CNTCTR1V  : FF_FF : Contactor1: Heartbeat: High voltage1:Current sensor1
	p->cid_hb2        = 0xFF000000; // CANID_HB_CNTCTR1A  : FF_FF : Contactor1: Heartbeat: High voltage2:Current sensor2
   p->cid_msg1       = 0x50400000; // CANID_MSG_CNTCTR1V : FF_FF : Contactor1: poll response: High voltage1:Current sensor1
   p->cid_msg2       = 0x50600000; // CANID_MSG_CNTCTR1A : FF_FF : Contactor1: poll response: battery gnd to: DMOC+, DMOC-
	p->cid_cmd_r      = 0xE3600000; // CANID_CMD_CNTCTR1R : U8_VAR: Contactor1: R: Command response
	p->cid_keepalive_r= 0xE3640000; // CANID_CMD_CNTCTRKAR: U8_U8 : Contactor1: R KeepAlive response

	// List of CAN ID's for setting up hw filter for incoming msgs
	p->cid_cmd_i        = 0xE360000C; // CANID_CMD_CNTCTR1I: U8_VAR: Contactor1: I: Command CANID incoming
	p->cid_keepalive_i  = 0xE3620000; // CANID_CMD_CNTCTRKAR:U8',    Contactor1: I KeepAlive and connect command
	p->cid_gps_sync     = 0x00400000; // CANID_HB_TIMESYNC:  U8 : GPS_1: U8 GPS time sync distribution msg-GPS time sync msg
	p->code_CAN_filt[3] = 0xFFFFFFFC; // CANID_DUMMY: UNDEF: Dummy ID: Lowest priority possible (Not Used)
	p->code_CAN_filt[4] = 0xFFFFFFFC; // CANID_DUMMY: UNDEF: Dummy ID: Lowest priority possible (Not Used)
	p->code_CAN_filt[5] = 0xFFFFFFFC; // CANID_DUMMY: UNDEF: Dummy ID: Lowest priority possible (Not Used)
	p->code_CAN_filt[6] = 0xFFFFFFFC; // CANID_DUMMY: UNDEF: Dummy ID: Lowest priority possible (Not Used)
	p->code_CAN_filt[7] = 0xFFFFFFFC; // CANID_DUMMY: UNDEF: Dummy ID: Lowest priority possible (Not Used)

	return;
}
