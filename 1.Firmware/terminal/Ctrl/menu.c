//
// Created by 10798 on 2023/8/22.
//

#include <main.h>
#include <common_inc.h>
#include "menu.h"
#include "multi_button.h"
#include "ctrl_types.h"

//单击按键回调函数
void BTN_SINGLE_Click_Handler(void* btn)
{
    Button *temp_button = (Button *)btn;
    switch(temp_button->button_id)
    {
        case 1:
            HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
            //单击按键锁定
            if(terminal_rc.armed == RC_ARMED_YES){
                terminal_rc.armed = RC_ARMED_NO;
            }
            break;
        case 2:
            HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
            //短按按键再stabilized 和 position之间切换，也会把forward切回来
            if(terminal_rc.mode == RC_MODE_STABILIZED){
                terminal_rc.mode = RC_MODE_POSITION;
            }
            else if(terminal_rc.mode == RC_MODE_POSITION){
                terminal_rc.mode = RC_MODE_STABILIZED;
            }
            else if(terminal_rc.mode == RC_MODE_FORWARD){
                terminal_rc.mode = RC_MODE_STABILIZED;
            }
            break;
        default:
            break;
    }
}

void BTN_LONG_Press_Start_Handler(void* btn)
{
    Button *temp_button = (Button *)btn;
    switch(temp_button->button_id)
    {
        case 1:
            HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
            //长按按键解锁
            if(terminal_rc.armed == RC_ARMED_NO){
                terminal_rc.armed = RC_ARMED_YES;
            } else{
                terminal_rc.armed = RC_ARMED_NO;
            }
            break;
        case 2:
            HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
            //长按按键把所有模式都切到FORWARD模式
            if(terminal_rc.mode == RC_MODE_STABILIZED){
                terminal_rc.mode = RC_MODE_FORWARD;
            }
            else if(terminal_rc.mode == RC_MODE_POSITION){
                terminal_rc.mode = RC_MODE_FORWARD;
            }
            break;
        default:
            break;
    }
}



//按键函数定义
uint8_t read_button_GPIO(uint8_t button_id)
{
    switch(button_id)
    {
        case 1:
            return HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
            break;
        case 2:
            return HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin);
            break;
        default:
            return 0;
            break;
    }
}