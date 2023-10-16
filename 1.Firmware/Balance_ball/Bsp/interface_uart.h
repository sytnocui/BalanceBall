//
// Created by 10798 on 2023/1/4.
//

#ifndef ATK_F405_FW_IBUS_H
#define ATK_F405_FW_IBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"
#include "ctrl_types.h"

#define WIFI_BUFFER_LENGTH     16
extern uint8_t wifi_rx_buffer[WIFI_BUFFER_LENGTH];

void WIFIRead(uint8_t* _rx_buffer, ctrl_rc_t* _rc);


#ifdef __cplusplus
}
#endif

#endif //ATK_F405_FW_IBUS_H
