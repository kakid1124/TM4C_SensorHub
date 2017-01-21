#include "include.h"


#define SwitchDelay_ms	5
#define ReleaseTime_ms	100
#define PressTime_ms	10
#define HoldPressTime_ms	3000

#define ON	0		// muc logic LOW (tuong ung voi 0V dien ap)
#define OFF	1		// muc logic HIGH (+3.3V)

enum Switch_States {BothSW_OFF = 0,
					LeftSW_ON,
					LeftSW_ON3s,
					LeftSW_OFF,
					RightSW_ON,
					RightSW_ON3s,
					RightSW_OFF,
					BothSW_ON,
					BothSW_ON3s	};

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
	T_OFF = 100;
	
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
//    uint8_t ui8CurButtonState, ui8PrevButtonState;
//    uint8_t ui8Message;

//    ui8CurButtonState = ui8PrevButtonState = 0;

	uint8_t Keys_Changed;
	uint8_t Keys_State = BothSW_OFF, Raw_State;
	uint8_t Debounced_LeftSW_Press = OFF;
	uint8_t Debounced_RightSW_Press = OFF;
	uint8_t StableTime1, StableTime2;
	int16_t HoldTime1 = 1000, HoldTime2 = 1000;
	
    // Get the current tick count.
    ui16LastTime = xTaskGetTickCount();

    // Loop forever.
    while(1)
    {
		Keys_Changed = 0;
		
	// Poll the debounced state of the buttons.
	// Left Switch
		if((GPIO_PORTF_DATA_R & SW1) == ON)
			Raw_State = ON;
		else
			Raw_State = OFF;
		
		if(Raw_State == Debounced_LeftSW_Press)
		{
		// Set the timer which allows a change from current state.
			if(Debounced_LeftSW_Press == ON)
			{
				HoldTime1 = (--HoldTime1 > 0) ? HoldTime1 : 0;
				StableTime1 = ReleaseTime_ms/SwitchDelay_ms;
			}
			else
				StableTime1 = PressTime_ms/SwitchDelay_ms;
		}
		else
		{
		// Key has changed - wait for new state to become stable.
			if(--StableTime1 == 0)
			{
			// Timer expired - accept the change.
				Debounced_LeftSW_Press = Raw_State;
				Keys_Changed = 1;
				
			// And reset the timer.
				HoldTime1 = HoldPressTime_ms/SwitchDelay_ms;
				
				if(Debounced_LeftSW_Press == ON)
					StableTime1 = ReleaseTime_ms/SwitchDelay_ms;
				else
					StableTime1 = PressTime_ms/SwitchDelay_ms;
			}
		}
	
	// Right Switch
		if((GPIO_PORTF_DATA_R & SW2) == ON)
			Raw_State = ON;
		else
			Raw_State = OFF;
		
		if(Raw_State == Debounced_RightSW_Press)
		{
		// Set the timer which allows a change from current state.
			if(Debounced_RightSW_Press == ON)
			{
				HoldTime2 = (--HoldTime2 > 0) ? HoldTime2 : 0;
				StableTime2 = ReleaseTime_ms/SwitchDelay_ms;
			}
			else
				StableTime2 = PressTime_ms/SwitchDelay_ms;
		}
		else
		{
		// Key has changed - wait for new state to become stable.
			if(--StableTime2 == 0)
			{
			// Timer expired - accept the change.
				Debounced_RightSW_Press = Raw_State;
				Keys_Changed = 1;
				
			// And reset the timer.
				HoldTime2 = HoldPressTime_ms/SwitchDelay_ms;
				
				if(Debounced_RightSW_Press == ON)
					StableTime2 = ReleaseTime_ms/SwitchDelay_ms;
				else
					StableTime2 = PressTime_ms/SwitchDelay_ms;
			}
		}
		

		if((Keys_Changed) || (HoldTime1 == 0) || (HoldTime2 == 0))
		{
			if((Debounced_LeftSW_Press == ON)&&(Debounced_RightSW_Press == OFF))
				Keys_State = LeftSW_ON;
			if(Debounced_LeftSW_Press == OFF)
				Keys_State = LeftSW_OFF;
			
			if((Debounced_LeftSW_Press == OFF)&&(Debounced_RightSW_Press == ON))
				Keys_State = RightSW_ON;
			if(Debounced_RightSW_Press == OFF)
				Keys_State = RightSW_OFF;
			
			if((Debounced_LeftSW_Press == ON)&&(Debounced_RightSW_Press == ON))
				Keys_State = BothSW_ON;
			
			if((HoldTime1 == 0)&&(Debounced_LeftSW_Press == ON)&&(Debounced_RightSW_Press == OFF))
				Keys_State = LeftSW_ON3s;
			if((HoldTime2 == 0)&&(Debounced_LeftSW_Press == OFF)&&(Debounced_RightSW_Press == ON))
				Keys_State = RightSW_ON3s;
			
			
			
			// Pass the value of the button pressed to Queue.
            if(xQueueSend(SwitchesState_Queue, &Keys_State, portMAX_DELAY) != pdPASS)
            {
                // Error. The queue should never be full. If so print the
                // error message on UART and wait for ever.
                UART_OutString("\nQueue full. This should never happen.\n");
                while(1){}
            }
		}

/*
        
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

