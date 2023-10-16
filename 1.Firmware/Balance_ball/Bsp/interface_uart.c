//
// Created by 10798 on 2023/1/4.
//

#include <string.h>
#include <utils/ctrl_math.h>
#include "interface_uart.h"



//--------------------------------------------------------------------------------
//DMA读出来的
uint8_t wifi_rx_buffer[WIFI_BUFFER_LENGTH];

void WIFIRead(uint8_t* _rx_buf, ctrl_rc_t* _rc)
{
    memcpy(&_rc->armed, &_rx_buf[0], sizeof(uint16_t));
    memcpy(&_rc->mode, &_rx_buf[2], sizeof(uint16_t));
    memcpy(&_rc->roll, &_rx_buf[4], sizeof(float));
    memcpy(&_rc->pitch, &_rx_buf[8], sizeof(float));
    memcpy(&_rc->yaw, &_rx_buf[12], sizeof(float));
    _rc->thrust = 0.0f;

}