#ifndef INCLUDE_H_
#define INCLUDE_H_

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/rom.h"
#include "driverlib/fpu.h"

#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "UART.h"
#include "GPIO.h"

#include "led_task.h"
#include "switch_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#endif
