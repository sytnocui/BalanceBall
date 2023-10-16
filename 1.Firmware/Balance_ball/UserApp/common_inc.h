#ifndef COMMON_INC_H
#define COMMON_INC_H

#define CONFIG_FW_VERSION 1.0

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"

#define DEBUG_UART		   (huart1)
#define WIFI_UART		   (huart2)



void Main(void);

#ifdef __cplusplus
}

/*---------------------------- C++ Scope ---------------------------*/



#endif
#endif
