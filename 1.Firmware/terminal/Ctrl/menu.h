//
// Created by 10798 on 2023/8/22.
//

#ifndef TERMINAL_MENU_H
#define TERMINAL_MENU_H

#include <stdint-gcc.h>

void BTN_SINGLE_Click_Handler(void* btn);
void BTN_LONG_Press_Start_Handler(void* btn);
uint8_t read_button_GPIO(uint8_t button_id);

#endif //TERMINAL_MENU_H
