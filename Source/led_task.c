#include <stdbool.h>
#include <stdint.h>
#include "UART.h"
#include "led_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


#define SwitchDelay_ms	25

//*****************************************************************************
//
// This task toggles the LED at a selected frequency.
//
//*****************************************************************************
static void LEDTask(void *pvParameters)
{
    portTickType ui32WakeTime;
	LED_Data_t Led_Message;
	uint32_t T, T_ON, T_OFF;	// milisecond

	// Default
	Led_Message.LED_Code = GREEN;
	T_ON = 100;
	T_OFF = 0;
	
    // Get the current tick count.
    ui32WakeTime = xTaskGetTickCount();

    // Loop forever.
    while(1)
    {
        // Read the next message, if available on queue.
        if(xQueueReceive(RGBLED_Queue, &Led_Message, 0) == pdPASS)
        {
			switch (Led_Message.LED_Code)
			{
				case RED:		break;
				case BLUE:		break;
				case GREEN:		break;
				case YELLOW:	break;
				case SKYBLUE:	break;
				case PINK:		break;
				case WHITE:		break;
				default:
					Led_Message.LED_Code = DARK; // turn off all LED			
			}
			
			if(Led_Message.freq <= 0.0f)
			{
				T_ON = 1000;
				T_OFF = 0;
			}
			else
			{
				T = (uint32_t)(1000.0f/Led_Message.freq);
				T_ON = T * Led_Message.duty / 100;
				T_OFF = T - T_ON;
			}
        }
		
		if(T_ON > 0)
		{
			// Turn on the LED.
			Led_ON(Led_Message.LED_Code);

			// Wait for the required amount of time.
			vTaskDelayUntil(&ui32WakeTime, T_ON / portTICK_RATE_MS);
		}

		if(T_OFF > 0)
		{
			// Turn off the LED.
			Led_OFF(ALL);

			// Wait for the required amount of time.
			vTaskDelayUntil(&ui32WakeTime, T_OFF / portTICK_RATE_MS);
		}
    }
}

//*****************************************************************************
//
// This task reads the buttons' state and passes this information to Main_Task.
//
//*****************************************************************************
static void SwitchTask(void *pvParameters)
{
    portTickType ui16LastTime;
    uint8_t ui8CurButtonState, ui8PrevButtonState;
    uint8_t ui8Message;

    ui8CurButtonState = ui8PrevButtonState = 0;

    // Get the current tick count.
    ui16LastTime = xTaskGetTickCount();

    // Loop forever.
    while(1)
    {
		

/*
        // Poll the debounced state of the buttons.
        ui8CurButtonState = ButtonsPoll(0, 0);

        // Check if previous debounced state is equal to the current state.
        if(ui8CurButtonState != ui8PrevButtonState)
        {
            ui8PrevButtonState = ui8CurButtonState;

            // Check to make sure the change in state is due to button press
            // and not due to button release.
            if((ui8CurButtonState & ALL_BUTTONS) != 0)
            {
                if((ui8CurButtonState & ALL_BUTTONS) == LEFT_BUTTON)
                {
                    ui8Message = LEFT_BUTTON;

                    // Guard UART from concurrent access.
                    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
                    UART_OutString("Left Button is pressed.\n");
                    xSemaphoreGive(g_pUARTSemaphore);
                }
                else if((ui8CurButtonState & ALL_BUTTONS) == RIGHT_BUTTON)
                {
                    ui8Message = RIGHT_BUTTON;

                    // Guard UART from concurrent access.
                    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
                    UART_OutString("Right Button is pressed.\n");
                    xSemaphoreGive(g_pUARTSemaphore);
                }

                // Pass the value of the button pressed to LEDTask.
                if(xQueueSend(g_pLEDQueue, &ui8Message, portMAX_DELAY) !=
                   pdPASS)
                {
                    // Error. The queue should never be full. If so print the
                    // error message on UART and wait for ever.
                    UART_OutString("\nQueue full. This should never happen.\n");
                    while(1)
                    {
                    }
                }
            }
        }
*/
        // Wait for the required amount of time to check back.
        vTaskDelayUntil(&ui16LastTime, SwitchDelay_ms / portTICK_RATE_MS);
    }
}


//*****************************************************************************
//
// Initializes the LED task and Switch task.
//
//*****************************************************************************
uint32_t LED_SwitchTaskInit(void)
{
    // Initialize the GPIO port F for RGB LEDs and two switches
	PortF_Init();
	
	
    // Create the LED task.
    if(xTaskCreate(LEDTask, (const portCHAR *)"LED", LEDTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_LED_TASK, NULL) != pdTRUE)
    {
        return(0);
    }

    // Create the switch task.
    if(xTaskCreate(SwitchTask, (const portCHAR *)"Switch",
                   SWITCHTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_SWITCH_TASK, NULL) != pdTRUE)
    {
        return(0);
    }	
	
    // Success.
    return(1);
}

