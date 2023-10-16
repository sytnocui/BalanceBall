#include <attitude.h>
#include "common_inc.h"
#include "usart.h"

#include "mpu6050.h"
#include "ctrl_types.h"
#include <oled.h>
#include <stm32f1xx_hal_tim.h>
#include <menu.h>
#include <multi_button.h>
#include <tim.h>
#include "retarget.h"

ctrl_rc_t terminal_rc;
struct Button button1;
struct Button button2;

uint16_t mytime = 0;

#define WIFI_BUFFER_LENGTH     16
uint8_t wifi_tx_buffer[WIFI_BUFFER_LENGTH];

/* Thread Definitions -----------------------------------------------------*/

/* Timer Callbacks -------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    //tim3定时器中断
    if (htim->Instance == TIM3) {
        //10ms
        //读取角速度和加速度
        mpu6050AccAndGyroRead(&acc, &gyro);
        mpu6050AccTransformUnit(&acc,&acc_f,&acc_drift);//转换单位
        mpu6050GyroTransformUnit(&gyro,&gyro_f,&gyro_drift);//转换单位

        //姿态解算
        //这里传入的参数坐标系有一个转换关系，源数据是东北天坐标系，抄的这个算法是北西天坐标系
        MahonyAHRSupdateIMU(q, gyro_f.x, gyro_f.y, gyro_f.z, acc_f.x, acc_f.y, acc_f.z);
        // 四元数反解欧拉角
        AttitudeQuaternionToEulerAngle(q,&state_attitude);
        // 转换单位
        AttitudeRadianToAngle(&state_attitude,&state_attitude_angle);

        //注意这里现在是  弧度制
        terminal_rc.roll = state_attitude.roll;
        terminal_rc.pitch = state_attitude.pitch;
        terminal_rc.yaw = state_attitude.yaw;

        mytime++;
//        printf("%.3f,%.3f,%.3f,%.3f\r\n",state_attitude_angle.roll,state_attitude_angle.pitch,state_attitude_angle.yaw,(float)mytime);


    } else if (htim->Instance == TIM4) {
        //5ms
        //按键扫描
        button_ticks();
    }  else if (htim->Instance == TIM2) {
        //50ms
        //wifi发送数据
        memcpy(&wifi_tx_buffer[0], &terminal_rc.armed, sizeof(uint16_t));
        memcpy(&wifi_tx_buffer[2], &terminal_rc.mode, sizeof(uint16_t));
        memcpy(&wifi_tx_buffer[4], &terminal_rc.roll, sizeof(float));
        memcpy(&wifi_tx_buffer[8], &terminal_rc.pitch, sizeof(float));
        memcpy(&wifi_tx_buffer[12], &terminal_rc.yaw, sizeof(float));
        HAL_UART_Transmit(&WIFI_UART, wifi_tx_buffer,sizeof(wifi_tx_buffer),0xffff);
    }


}

//软件定时器虽好，可不要贪杯欧

/* Default Entry -------------------------------------------------------*/
void Main(void) {
    RetargetInit(&WIFI_UART);

    oled_init();

    //init mpu6050
    mpu6050Init();
    HAL_Delay(1000);
    Gyro_And_Acc_Calibrate(&gyro_drift, &acc_drift);

    ////==========================按键相关===========================================
    // 初始化按键（硬件相关）
    button_init(&button1, read_button_GPIO, 0,1);
    button_init(&button2, read_button_GPIO, 0,2);
    // 按键回调函数（软件相关）
    // arm按钮
    button_attach(&button1, SINGLE_CLICK,     BTN_SINGLE_Click_Handler);
    button_attach(&button1, LONG_PRESS_START,     BTN_LONG_Press_Start_Handler);
    // mode按钮
    button_attach(&button2, SINGLE_CLICK,     BTN_SINGLE_Click_Handler);
    button_attach(&button2, LONG_PRESS_START,     BTN_LONG_Press_Start_Handler);
    //打开btn
    button_start(&button1);
    button_start(&button2);

    //初始化遥控状态
    terminal_rc.roll = 0.0f;
    terminal_rc.pitch = 0.0f;
    terminal_rc.yaw = 0.0f;
    terminal_rc.armed = RC_ARMED_NO;
    terminal_rc.mode = RC_MODE_STABILIZED;

    //启动定时器
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim4);

    while (1){
        ////=====================================================================
        //oled显图
        oled_show_string( 0, 2, "ROLL:");
        oled_show_string( 0, 4, "PITCH:");
        oled_show_string( 0, 6, "YAW:");

        oled_show_float( 48, 0, (float) mytime,4,0);
        oled_show_float( 60, 2, state_attitude_angle.roll,3,2);
        oled_show_float( 60, 4, state_attitude_angle.pitch,3,2);
        oled_show_float( 60, 6, state_attitude_angle.yaw,3,2);

        if(terminal_rc.armed == RC_ARMED_NO)
            oled_show_string( 0, 0, "DIS");
        else if(terminal_rc.armed == RC_ARMED_YES)
            oled_show_string( 0, 0, "ARM");

        if(terminal_rc.mode == RC_MODE_STABILIZED)
            oled_show_string( 100, 0, "STB");
        else if(terminal_rc.mode == RC_MODE_POSITION)
            oled_show_string( 100, 0, "POS");
        else if(terminal_rc.mode == RC_MODE_FORWARD)
            oled_show_string( 100, 0, "FWD");

        HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);

        HAL_Delay(10);
    }
}
