#ifndef COMMON_INC_H
#define COMMON_INC_H

#define CONFIG_FW_VERSION 1.0

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif


#include <ctrl_types.h>
#include "main.h"


#define WIFI_UART		   (huart2)

extern ctrl_rc_t terminal_rc;

void Main(void);

#ifdef __cplusplus
}

/*---------------------------- C++ Scope ---------------------------*/



#endif
#endif
