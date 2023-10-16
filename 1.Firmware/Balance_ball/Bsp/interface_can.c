//
// Created by 10798 on 2023/7/23.
//

#include "interface_can.h"
#include "ctrl_types.h"


CAN_TxHeaderTypeDef txMessage;  //鍙戦?佺粨鏋勪綋
CAN_RxHeaderTypeDef rxMessage;  //鎺ュ彈缁撴瀯浣?


//物理层代码
void can_send_message(uint16_t id, uint8_t *buf,uint8_t len)
{
    uint32_t tx_mail = CAN_TX_MAILBOX0;
//    uint32_t tx_mail;
    txMessage.StdId = id;
    txMessage.DLC = len;
    txMessage.IDE = CAN_ID_STD;
    txMessage.RTR = CAN_RTR_DATA;
    txMessage.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan, &txMessage, buf, &tx_mail) != HAL_OK)
    {
        /* Transmission request Error */
        Error_Handler();
    }
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
}

void can_send_cmd(uint16_t id)
{
    uint32_t tx_mail = CAN_TX_MAILBOX0;
//    uint32_t tx_mail;
    txMessage.StdId = id;
    txMessage.DLC = 4;
    txMessage.IDE = CAN_ID_STD;
    txMessage.RTR = CAN_RTR_REMOTE;
    txMessage.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan, &txMessage, NULL, &tx_mail) != HAL_OK)
    {
        /* Transmission request Error */
        Error_Handler();
    }
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
}

//驱动层代码

void Drive_Init(uint32_t CAN_ID)
{
    can_send_cmd((CAN_ID<<5) + 0x005); //电机进入闭环模式
}

void Driver_control(uint32_t CAN_ID, float target)
{
    driver_data _data;
    uint8_t send_ch[4];
    _data.data_f=target;
    for(uint8_t i=0;i<4;i++)
    {
        send_ch[i]=_data.data_ch[i];
    }
    can_send_message((CAN_ID<<5) + 0x00d, send_ch,4);
}

void Drive_Clear_Error(uint32_t CAN_ID)
{
    can_send_cmd((CAN_ID<<5) + 0x018); //清除错误
}


void DriverCmdSend(ctrl_rc_t* _rc, motorSpeed_t * _motor){
    //再次检测是否解锁
    //这里因为可能没收到ibus信息，故不能检测到no就return
    if(_rc->armed == RC_ARMED_YES){
        //输出三个转轴的转速
        Driver_control(M0,_motor->m1);
        Driver_control(M1,_motor->m2);
        Driver_control(M2,_motor->m3);
    } else {
        //输出三个转轴的转速
        Driver_control(M0,0.0f);
        Driver_control(M1,0.0f);
        Driver_control(M2,0.0f);

    }


}
