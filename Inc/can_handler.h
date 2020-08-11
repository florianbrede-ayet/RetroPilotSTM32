#ifndef _CANHANDLER_H_
#define _CANHANDLER_H_

#include "can.h"
#include "core.h"


typedef struct
{
    uint32_t id;
    uint8_t buf[8];
} CAN_STD_Msg;

void can_init_all(void);
bool can_receive_message(int bus_number, CAN_STD_Msg *msg);
bool can_send_message(int bus_number, uint32_t txId, uint8_t txBuf[8]);
void can_send_internal();

bool check_can_errors();


void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);


#endif