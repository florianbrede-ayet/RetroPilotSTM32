#include "can_handler.h"
#include "logger.h"
#include "critical.h"
#include <string.h>

#define CANMANAGER_DEBUG


typedef struct {
  volatile uint32_t w_ptr;
  volatile uint32_t r_ptr;
  uint32_t fifo_size;
  CAN_STD_Msg *elems;
} can_ring;

#define can_buffer(x, size) \
  CAN_STD_Msg elems_##x[size]; \
  can_ring can_##x = { .w_ptr = 0, .r_ptr = 0, .fifo_size = size, .elems = (CAN_STD_Msg *)&elems_##x };

can_buffer(rx1_q, 0x100)
can_buffer(rx2_q, 0x100)
can_buffer(tx1_q, 0x100)
can_buffer(tx2_q, 0x100)

can_ring *can_rx_queues[] = {&can_rx1_q, &can_rx2_q};
can_ring *can_tx_queues[] = {&can_tx1_q, &can_tx2_q};

// global CAN stats
volatile uint32_t can_rx_cnt = 0;
volatile uint32_t can_tx_cnt = 0;
volatile uint32_t can_err_cnt = 0;
volatile uint32_t can_overflow_cnt = 0;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

CAN_HandleTypeDef *can_handles[] = {&hcan1, &hcan2};

CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint32_t              TxMailbox;


// ********************* interrupt safe queue *********************

bool can_pop(can_ring *q, CAN_STD_Msg *elem) {    
  bool ret = 0;

  ENTER_CRITICAL();
  if (q->w_ptr != q->r_ptr) {
    *elem = q->elems[q->r_ptr];
    if ((q->r_ptr + 1U) == q->fifo_size) {
      q->r_ptr = 0;
    } else {
      q->r_ptr += 1U;
    }
    ret = 1;
  }
  EXIT_CRITICAL();

  return ret;
}



bool can_push(can_ring *q, CAN_STD_Msg *elem) {
  bool ret = false;
  uint32_t next_w_ptr;

  ENTER_CRITICAL();
  if ((q->w_ptr + 1U) == q->fifo_size) {
    next_w_ptr = 0;
  } else {
    next_w_ptr = q->w_ptr + 1U;
  }
  if (next_w_ptr != q->r_ptr) {
    q->elems[q->w_ptr] = *elem;
    q->w_ptr = next_w_ptr;
    ret = true;
  }
  EXIT_CRITICAL();
  if (!ret) {
    can_overflow_cnt++;
    #ifdef CANMANAGER_DEBUG
      logger("can_push failed!\n");
    #endif
  }
  return ret;
}

uint32_t can_slots_empty(can_ring *q) {
  uint32_t ret = 0;

  ENTER_CRITICAL();
  if (q->w_ptr >= q->r_ptr) {
    ret = q->fifo_size - 1U - q->w_ptr + q->r_ptr;
  } else {
    ret = q->r_ptr - q->w_ptr ;
  }
  EXIT_CRITICAL();

  return ret;
}


bool can_peek(can_ring *q) {    
  uint32_t num_empty = can_slots_empty(q);
  return (num_empty<q->fifo_size-1);
}

void can_clear(can_ring *q) {
  ENTER_CRITICAL();
  q->w_ptr = 0;
  q->r_ptr = 0;
  EXIT_CRITICAL();
}


void can_init_all(void) {
  for (uint8_t i=0U; i < 2; i++) {
    can_clear(can_rx_queues[i]);
    can_clear(can_tx_queues[i]);
  }
}

bool can_receive_message(int bus_number, CAN_STD_Msg *msg) {
    return can_pop(can_rx_queues[bus_number], msg);
}


void can_send_internal() {
    for (int bus_number=0; bus_number<2; bus_number++) {

        while(can_peek(can_tx_queues[bus_number]) && HAL_CAN_GetTxMailboxesFreeLevel(can_handles[bus_number])>0) {
            CAN_STD_Msg msg;
            if (can_pop(can_tx_queues[bus_number], &msg)) {
                TxHeader.StdId=msg.id;                
                if (HAL_CAN_AddTxMessage(can_handles[bus_number], &TxHeader, msg.buf, &TxMailbox) != HAL_OK)
                {
                    can_err_cnt++;
                    break;
                }
                else {
                    can_tx_cnt++;
                }
            }
            else break;
        }
    }
}

bool can_send_message(int bus_number, uint32_t txId, uint8_t txBuf[8]) {
    CAN_STD_Msg msg;
    msg.id=txId;
    memcpy(&msg.buf, txBuf, 8*sizeof(uint8_t));

    bool ret = can_push(can_tx_queues[bus_number], &msg);
    can_send_internal();
    return ret;
}






void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    logger("HAL_CAN_TxMailbox0CompleteCallback(%d)\n", hcan==&hcan1 ? 1 : 2);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
    logger("HAL_CAN_TxMailbox1CompleteCallback(%d)\n", hcan==&hcan1 ? 1 : 2);   
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
    logger("HAL_CAN_TxMailbox2CompleteCallback(%d)\n", hcan==&hcan1 ? 1 : 2);   
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan) {
    logger("HAL_CAN_TxMailbox0AbortCallback(%d)\n", hcan==&hcan1 ? 1 : 2);   
    can_err_cnt++;
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan) {
    logger("HAL_CAN_TxMailbox1AbortCallback(%d)\n", hcan==&hcan1 ? 1 : 2);   
    can_err_cnt++;
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan) {
    logger("HAL_CAN_TxMailbox2AbortCallback(%d)\n", hcan==&hcan1 ? 1 : 2);   
    can_err_cnt++;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    //logger("HAL_CAN_RxFifo0MsgPendingCallback(%d)\n", hcan==&hcan1 ? 1 : 2);   
    CAN_STD_Msg canMsg;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, canMsg.buf) != HAL_OK)
    {
        can_err_cnt++;
    }
    else {
        canMsg.id=RxHeader.StdId;
        can_push(can_rx_queues[hcan==&hcan1 ? 0 : 1], &canMsg);
    }
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {
    logger("HAL_CAN_RxFifo0FullCallback(%d)\n", hcan==&hcan1 ? 1 : 2);   
    can_err_cnt++;
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    //logger("HAL_CAN_RxFifo1MsgPendingCallback(%d)\n", hcan==&hcan1 ? 1 : 2);   
    CAN_STD_Msg canMsg;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, canMsg.buf) != HAL_OK)
    {
        can_err_cnt++;
    }
    else {
        canMsg.id=RxHeader.StdId;
        can_push(can_rx_queues[hcan==&hcan1 ? 0 : 1], &canMsg);
        can_rx_cnt++;
    }
}

void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan) {
    logger("HAL_CAN_RxFifo1FullCallback(%d)\n", hcan==&hcan1 ? 1 : 2);   
    can_err_cnt++;
}

void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan) {
    logger("HAL_CAN_SleepCallback(%d)\n", hcan==&hcan1 ? 1 : 2);   
}

void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan) {
    logger("HAL_CAN_WakeUpFromRxMsgCallback(%d)\n", hcan==&hcan1 ? 1 : 2);   
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    logger("HAL_CAN_ErrorCallback(%d)\n", hcan==&hcan1 ? 1 : 2);   
    can_err_cnt++;
}

bool check_can_errors() {
  ENTER_CRITICAL();
  uint32_t irx=can_rx_cnt;
  uint32_t itx=can_tx_cnt;
  uint32_t ierr=can_err_cnt;
  uint32_t ioflw=can_overflow_cnt;
  can_err_cnt=0; can_tx_cnt=0; can_rx_cnt=0; can_overflow_cnt=0;
  EXIT_CRITICAL();

  bool has_err=false;
  if (ierr>0 && ierr>=(irx+itx)*2/100)
    has_err=true;

  if (ioflw>0 && ioflw>=itx/100)
    has_err=true;


  return has_err;

}