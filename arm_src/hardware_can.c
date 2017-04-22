#include "chip.h"
#include <string.h>
#include <stdint.h>
#include "ccand_11xx.h"

#define CAN_BUF_SIZE 8

static CCAN_MSG_OBJ_T msg_obj;
static RINGBUFF_T rx_buffer;
static CCAN_MSG_OBJ_T _rx_buffer[CAN_BUF_SIZE];

static RINGBUFF_T tx_buffer;
static CCAN_MSG_OBJ_T _tx_buffer[CAN_BUF_SIZE];
static CCAN_MSG_OBJ_T tmp_msg_obj; // temporarily store data/CAN ID to insert into tx_buf

static bool can_error_flag;
static uint32_t can_error_info;

static volatile bool busy = false;

// integrates with and based on ccand_11x.h error types
// explanations of page 289 of lpc11cx4 user manual
typedef enum HARDWARE_CAN_ERROR {
    HARDWARE_NO_CAN_ERROR,
    HARDWARE_NO_RX_CAN_MESSAGE,
    HARDWARE_CAN_ERROR,
} HARDWARE_CAN_ERROR_T;

void Hardware_prepare_tx_msg_obj(uint32_t msg_id, uint8_t* data, uint8_t data_len, CCAN_MSG_OBJ_T* msgobj);
void Hardware_do_transmit(CCAN_MSG_OBJ_T *msg_obj);

void Hardware_CAN_Init(uint32_t baud_rate);
HARDWARE_CAN_ERROR_T Hardware_CAN_Receive(CCAN_MSG_OBJ_T* user_buffer);
HARDWARE_CAN_ERROR_T Hardware_CAN_Transmit(uint32_t msg_id, uint8_t* data, uint8_t data_len);
HARDWARE_CAN_ERROR_T Hardware_CAN_TransmitMsgObj(CCAN_MSG_OBJ_T *msg_obj);
void Hardware_CAN_Pass(void);


/*************************************************
 *                  HELPERS
 * ************************************************/

// TODO EXPLAIN WHAT THIS DOES AND SIMPLIFY
void Hardware_Baudrate_Calculate(uint32_t baud_rate, uint32_t *can_api_timing_cfg) {
	uint32_t pClk, div, quanta, segs, seg1, seg2, clk_per_bit, can_sjw;
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_CAN);
	pClk = Chip_Clock_GetMainClockRate();

	clk_per_bit = pClk / baud_rate;

	for (div = 0; div <= 15; div++) {
		for (quanta = 1; quanta <= 32; quanta++) {
			for (segs = 3; segs <= 17; segs++) {
				if (clk_per_bit == (segs * quanta * (div + 1))) {
					segs -= 3;
					seg1 = segs / 2;
					seg2 = segs - seg1;
					can_sjw = seg1 > 3 ? 3 : seg1;
					can_api_timing_cfg[0] = div;
					can_api_timing_cfg[1] =
						((quanta - 1) & 0x3F) | (can_sjw & 0x03) << 6 | (seg1 & 0x0F) << 8 | (seg2 & 0x07) << 12;
					return;
				}
			}
		}
	}
}

/*************************************************
 *                  CALLBACKS
 * ************************************************/

/*	CAN receive callback */
void Hardware_CAN_rx(uint8_t msg_obj_num) {
	/* Determine which CAN message has been received */
	msg_obj.msgobj = msg_obj_num;
	/* Now load up the msg_obj structure with the CAN message */
	LPC_CCAN_API->can_receive(&msg_obj);
	RingBuffer_Insert(&rx_buffer, &msg_obj);
}

/*	CAN transmit callback */
void Hardware_CAN_tx(uint8_t msg_obj_num) {
	UNUSED(msg_obj_num);
  busy = false;
}

void Hardware_CAN_Pass(void) {
  if (!busy && !RingBuffer_IsEmpty(&tx_buffer)){
      // Receive is called every iteration, so piggyback on it for queue cleanup
      RingBuffer_Pop(&tx_buffer, &tmp_msg_obj);
      Hardware_do_transmit(&tmp_msg_obj);
  }
}

/*	CAN error callback */
void Hardware_CAN_error(uint32_t error_info) {
	can_error_info = error_info;
	can_error_flag = true;
}

/**
 * @brief	CCAN Interrupt Handler
 * @return	Nothing
 * @note	The CCAN interrupt handler must be provided by the user application.
 *	It's function is to call the isr() API located in the ROM
 */
void Hardware_CAN_IRQHandler(void) {
	LPC_CCAN_API->isr();
}

void Hardware_CAN_Init(uint32_t baud_rate) {

	RingBuffer_Init(&rx_buffer, _rx_buffer, sizeof(CCAN_MSG_OBJ_T), CAN_BUF_SIZE);
	RingBuffer_Flush(&rx_buffer);

	RingBuffer_Init(&tx_buffer, _tx_buffer, sizeof(CCAN_MSG_OBJ_T), CAN_BUF_SIZE);
	RingBuffer_Flush(&tx_buffer);

	uint32_t CanApiClkInitTable[2];
	CCAN_CALLBACKS_T callbacks = {
		Hardware_CAN_rx,
		Hardware_CAN_tx,
		Hardware_CAN_error,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
	};
	Hardware_Baudrate_Calculate(baud_rate, CanApiClkInitTable);

	LPC_CCAN_API->init_can(&CanApiClkInitTable[0], TRUE);
	/* Configure the CAN callback functions */
	LPC_CCAN_API->config_calb(&callbacks);
	/* Enable the CAN Interrupt */
	NVIC_EnableIRQ(CAN_IRQn);

	/* Configure message object 1 to only ID 0x600 */
	msg_obj.msgobj = 1;
	msg_obj.mode_id = 0xFFF;
    // ANDs the mask with the input ID and checks if == to mode_id
	msg_obj.mask = 0x000; 
	LPC_CCAN_API->config_rxmsgobj(&msg_obj);

	can_error_flag = false;
	can_error_info = 0;
}

HARDWARE_CAN_ERROR_T Hardware_CAN_Receive(CCAN_MSG_OBJ_T* user_buffer) {
	if (can_error_flag) {
		can_error_flag = false;
		return HARDWARE_CAN_ERROR;
	} else if (!RingBuffer_IsEmpty(&rx_buffer)) {
    RingBuffer_Pop(&rx_buffer, user_buffer);
    return HARDWARE_NO_CAN_ERROR;
  }
  return HARDWARE_NO_RX_CAN_MESSAGE;
}

void Hardware_prepare_tx_msg_obj(uint32_t msg_id, uint8_t* data, uint8_t data_len, CCAN_MSG_OBJ_T* m_obj) {
    m_obj->msgobj = 2;
    m_obj->mode_id = msg_id;
    m_obj->dlc = data_len;
    uint8_t i;
    for (i = 0; i < m_obj->dlc; i++) {	
        m_obj->data[i] = data[i];
    }
}

void Hardware_do_transmit(CCAN_MSG_OBJ_T *msg_obj) {
    busy = true;
    LPC_CCAN_API->can_transmit(msg_obj);
}

HARDWARE_CAN_ERROR_T Hardware_CAN_Transmit(uint32_t msg_id, uint8_t* data, uint8_t data_len) {
  Hardware_prepare_tx_msg_obj(msg_id, data, data_len, &tmp_msg_obj);
  return Hardware_CAN_TransmitMsgObj(&tmp_msg_obj);
}

HARDWARE_CAN_ERROR_T Hardware_CAN_TransmitMsgObj(CCAN_MSG_OBJ_T *msg_obj) {
	if (can_error_flag) {
		can_error_flag = false;
		return HARDWARE_CAN_ERROR;
	} else {
    bool has_pending_messages = (busy || !RingBuffer_IsEmpty(&tx_buffer));
    if (has_pending_messages) {
      // Message already pending, so add this to FIFO queue
      RingBuffer_Insert(&tx_buffer, msg_obj);
    } else {
      // No messages ahead of us so just go ahead and transmit now
      Hardware_do_transmit(msg_obj);
    }
    return HARDWARE_NO_CAN_ERROR;
	}
}
