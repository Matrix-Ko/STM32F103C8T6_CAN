

#include "can.h"
#include "stm32f1xx_hal_can.h"

//定义数据
extern uint8_t rcvdFlag;
extern CANTxMsg_t TxMsg;
extern CANRxMsg_t RxMsg;


//声明
void CAN1_Filter_Init(void);
void CAN1_Receive_Msg(CANRxMsg_t *msg);
uint8_t CAN1_Send_Msg(CANTxMsg_t *msg,uint16_t mailbox_id,uint8_t *sendbuff);
