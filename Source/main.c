//*************************************************//
//**** Tiva C - FreeRTOS Project ****//
//
// Nguyen Huynh Da Khoa - Version 1.0 - 10.01.2017 //
//
// System clock: 8 ~ 40MHz
//
//*************************************************//

#include "include.h"

//**** Function Prototype ****//
#ifdef __cplusplus
extern "C" {
#endif

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

static void SystemExecution_Task(void *pvParameters);
static void MonitorAndCommunication_Task(void *pvParameters);
static void ProcessReceivedCommands_Task(void *pvParameters);
	
#ifdef __cplusplus
}
#endif


//*****************************************************************************
// This hook is called by FreeRTOS when an stack overflow error is detected.
void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    while(1){}
}


//**************************************//
//
//************ Main Program ************//	
//
//**************************************//

int main()
{
	#ifdef System_Clock_40MHz
		PLL_Init();			// System Clock: 40MHz
		UART_Init();		// UART: 115200 bps
		Timer0A_Delay_Init();
	#elif System_Clock_8MHz
		PLL_Init_8MHz(); 	// System Clock: 8MHz
		UART_Init_9600bps();// UART: 9600 bps
		Timer0A_Delay_Init_8MHz();
	#endif	

//	EnableInterrupts();
	ROM_FPUEnable();
	ROM_FPULazyStackingEnable();

	System_Status = Init;
	
    // Create mutex and semaphore
	GlobalVariable_Mutex = xSemaphoreCreateMutex();

	// Create a queue for sending messages to the LED task.
    RGBLED_Queue = xQueueCreate(LED_QUEUE_SIZE, LED_ITEM_SIZE);
	SwitchesState_Queue = xQueueCreate(SW_QUEUE_SIZE, SW_ITEM_SIZE);

	
	// Create the System Execution Task
//	xTaskCreate(SystemExecution_Task, (const portCHAR *)"SystemExecution_Task", SystemExecution_STACKSIZE, NULL,
//                   tskIDLE_PRIORITY + PRIORITY_SystemExecution_TASK, NULL);

    // Create the Monitor And Communication task.
    xTaskCreate(MonitorAndCommunication_Task, (const portCHAR *)"MonitorAndCommunication_Task",
                   MonitorAndCommunicationTask_STACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_MonitorAndCommunication_TASK, NULL);
	
	// Create the Process Received Commands task.
    xTaskCreate(ProcessReceivedCommands_Task, (const portCHAR *)"ProcessReceivedCommands_Task",
                   ProcessReceivedCommandsTask_STACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_ProcessReceivedCommands_TASK, NULL);

	// Create the MPU9150 task.
    if(MPU9150TaskInit() == pdFAIL)
    {
        while(1){}
    }
	
	System_Status = System_Ready;
	
    // Start the scheduler.  This should not return.
    vTaskStartScheduler();

    // In case the scheduler returns for some reason, print an error and loop
    // forever.
	while(1){}
}




//*****************************************************************************
//
//	System Execution Task: 
//		
//		
//
//*****************************************************************************
static void SystemExecution_Task(void *pvParameters)
{
	
	
	while(1)
	{
		switch(System_Ready){
			case ArmedFlight_Mode:
				
			
				break;
			
		}
		
		WaitForInterrupt();
	}
}

//*****************************************************************************
//
//		Monitor and Communication Task (Giam Sat va Truyen Thong)
//	1. Receive commands and global data to display
//		(send to UART, nRF or Wifi): 5~50Hz
//	2. Battery monitor
//	3. Thu thap du lieu tu glabal data va giam sat he thong
//		--> thay doi System_Status
//	4. LED Toggle
//
//*****************************************************************************
#define FreqMonitor_Hz		100
#define MonitorDelay_ms		(1000/FreqMonitor_Hz)
#define FreqDisplay_Hz		5
#define FrameDisplay		(FreqMonitor_Hz/FreqDisplay_Hz)
#define FreqMaxLED			100
#define DoPhanGiaiLED_ms	(1000/FreqMaxLED)

static void MonitorAndCommunication_Task(void *pvParameters)
{
    portTickType ui32WakeTime;
	static uint8_t tick_count = 0;
	
	enum Display_Data {None = 0, UART_Monitor, nRF_Device};
	
	uint8_t SW_State, DisplayToWhat = 0;
	LED_Data_t Led_Message = {GREEN, 50, 1}, Led_Message_Received;
	int32_t T, T_ON, t, t_on;
	
	Sensor_t Sensors;
	
	// Default when power on
	xQueueSend(RGBLED_Queue, &Led_Message, portMAX_DELAY);
	
    // Get the current tick count.
    ui32WakeTime = xTaskGetTickCount();

    // Loop forever.
    while(1)
    {
		tick_count = (tick_count + 1) % FreqMonitor_Hz;
		
		// 1. Display datas: 5Hz
		if(tick_count % FrameDisplay == 1)	// tick_count = 1..21..41..61..81
		{
			switch (DisplayToWhat){
				case UART_Monitor:
				
					UART_OutString((char *)"Eulers: ");
					UART_OutFloat(Sensors.MPU9150.Euler[0]*R2D);
					UART_OutString((char *)"  ");
					UART_OutFloat(Sensors.MPU9150.Euler[1]*R2D);
					UART_OutString((char *)"  ");
					UART_OutFloat(Sensors.MPU9150.Euler[2]*R2D);
					UART_OutString((char *)"\n\r");
					
					break;
				case nRF_Device:
				
					break;
			}
		}
		
		// 2. Batter Monitor: 1Hz
		if(tick_count == 99)
		{
			
		}
		
		// 3. Giam sat he thong: 100Hz
		{
			// Read the message from SWITCH queue.
			if(xQueueReceive(SwitchesState_Queue, &SW_State, 0) == pdPASS)
			{
				switch (SW_State){
					case LeftSW_ON:
						
						break;
					case LeftSW_ON3s:
						if((System_Status == System_Ready)&&(DisplayToWhat == None))
						{
							DisplayToWhat = UART_Monitor;
							
							Led_Message.LED_Code = BLUE;
							Led_Message.duty = 50;
							Led_Message.freq = 5;
							xQueueSend(RGBLED_Queue, &Led_Message, portMAX_DELAY);
						}
						break;
					case LeftSW_OFF:
					
						break;
					case RightSW_ON:
						
						break;
					case RightSW_ON3s:
						if(DisplayToWhat == UART_Monitor)
						{
							DisplayToWhat = None;
							
							Led_Message.LED_Code = RED;
							Led_Message.duty = 50;
							Led_Message.freq = 0;
							xQueueSend(RGBLED_Queue, &Led_Message, portMAX_DELAY);
						}
						break;
					case RightSW_OFF:
					
						break;
					case BothSW_ON:
					
						break;
					case BothSW_ON3s:
					
						break;
				}
				
				
				
			}
			
			// Get new data from global variables
			xSemaphoreTake(GlobalVariable_Mutex, portMAX_DELAY);
			Sensors.MPU9150 = MPU9150_FilteredData;
			xSemaphoreGive(GlobalVariable_Mutex);
			
		}
		
		// 4. LEDs Display:
		{
			// Read the next message, if available on queue.
			// => (LED_Code, T_ON, T_OFF)
			if(xQueueReceive(RGBLED_Queue, &Led_Message_Received, 0) == pdPASS)
			{	
				if(Led_Message_Received.freq <= 0.0f)
				{
					T = T_ON = 1;
				}
				else
				{
					T = (int32_t)(1000.0f / Led_Message_Received.freq) / DoPhanGiaiLED_ms;	//[10ms]
					if(T <= 1)
					{
						T = T_ON = 1;
					}
					else
					{
						T_ON = (T * Led_Message_Received.duty / 100);
						if((Led_Message_Received.duty > 0)&&(T_ON == 0))
							T_ON = 1;		//[10ms]
					}
				}
				
				t = T;
				t_on = T_ON;
			}
		
			switch (Led_Message_Received.LED_Code)
			{
				case RED:
				case BLUE:
				case GREEN:
				case YELLOW:
				case SKYBLUE:
				case PINK:
				case WHITE:
					if((T <= 0)||(T_ON <= 0))
					{
						// turn off all LED
						Led_OFF(ALL);
					}
					else if((T_ON > 0)&&(t_on > 0))
					{
						// Turn on the LED.
						Led_ON(Led_Message_Received.LED_Code);
						t--; t_on--;
					}
					else if((t > 0)&&(t_on == 0))
					{
						// turn off all LED
						Led_OFF(ALL);
						t--;
						
						if((t == 0)&&(T_ON > 0))
						{	// reset
							t = T;
							t_on = T_ON;
						}
					}
				break;
				default: // turn off all LED
					Led_OFF(ALL);
				break;
			}
		}
	
		// Wait for the required amount of time to check back.
		vTaskDelayUntil(&ui32WakeTime, MonitorDelay_ms / portTICK_RATE_MS);
    }
}

//*****************************************************************************
//
//		Process Received Commands Task
//	1. Read state from 2 switches on board (200Hz) => SwitchesState_Queue
//	2. Process command from Receiver (50Hz)
//	3. Receive and process command from UART or SPI (nRF24L01, Wifi,...)
//	
//*****************************************************************************
#define SwitchDelay_ms		5
#define Freq_ScanSwitches	(1000/SwitchDelay_ms)
#define ReleaseTime_ms		100
#define PressTime_ms		10
#define HoldPressTime_ms	3000

#define ON	0		// muc logic LOW (tuong ung voi 0V dien ap)
#define OFF	1		// muc logic HIGH (+3.3V)

static void ProcessReceivedCommands_Task(void *pvParameters)
{
	portTickType ui16LastTime;
	static uint8_t tick_count = 0;
	
	// Switches
	uint8_t LeftSW_Changed, RightSW_Changed;
	uint8_t Keys_State = BothSW_OFF, Raw_State;
	uint8_t Debounced_LeftSW_Press = OFF;
	uint8_t Debounced_RightSW_Press = OFF;
	uint8_t StableTime1, StableTime2;
	int16_t HoldTime1 = 1000, HoldTime2 = 1000;
	
	// Get the current tick count.
    ui16LastTime = xTaskGetTickCount();
	while(1)
	{
		tick_count = (tick_count + 1) % Freq_ScanSwitches; // tick_count = 0..199
		
		// Poll 2 Switches: 200Hz
		{	LeftSW_Changed = 0;
			RightSW_Changed = 0;
			
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
			else{
				// Key has changed - wait for new state to become stable.
				if(--StableTime1 == 0)
				{
					// Timer expired - accept the change.
					Debounced_LeftSW_Press = Raw_State;
					LeftSW_Changed = 1;
				
					// And reset the timer.
					HoldTime1 = HoldTime2 = HoldPressTime_ms/SwitchDelay_ms;
				
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
					RightSW_Changed = 1;
				
					// And reset the timer.
					HoldTime1 = HoldTime2 = HoldPressTime_ms/SwitchDelay_ms;
				
					if(Debounced_RightSW_Press == ON)
						StableTime2 = ReleaseTime_ms/SwitchDelay_ms;
					else
						StableTime2 = PressTime_ms/SwitchDelay_ms;
				}
			}
		
			// Deceide the buttons' state changed
			if((LeftSW_Changed) || (RightSW_Changed) || (HoldTime1 == 0) || (HoldTime2 == 0))
			{
				if((HoldTime1 == 0)&&(HoldTime2 == 0)&&(Keys_State == BothSW_ON3s))
				{}
				else if((HoldTime1 == 0)&&(Keys_State == LeftSW_ON3s))
				{}
				else if((HoldTime2 == 0)&&(Keys_State == RightSW_ON3s))
				{}
				else
				{
					if((LeftSW_Changed)&&(Debounced_LeftSW_Press == ON)&&(Debounced_RightSW_Press == OFF))
						Keys_State = LeftSW_ON;
					else if((LeftSW_Changed)&&(Debounced_LeftSW_Press == OFF))
						Keys_State = LeftSW_OFF;
					else if((RightSW_Changed)&&(Debounced_LeftSW_Press == OFF)&&(Debounced_RightSW_Press == ON))
						Keys_State = RightSW_ON;
					else if((RightSW_Changed)&&(Debounced_RightSW_Press == OFF))
						Keys_State = RightSW_OFF;
					else if((HoldTime1 == 0)&&(Debounced_LeftSW_Press == ON)&&(Debounced_RightSW_Press == OFF))
						Keys_State = LeftSW_ON3s;
					else if((HoldTime2 == 0)&&(Debounced_LeftSW_Press == OFF)&&(Debounced_RightSW_Press == ON))
						Keys_State = RightSW_ON3s;
					else if((HoldTime1 == 0)&&(HoldTime2 == 0)&&(Debounced_LeftSW_Press == ON)&&(Debounced_RightSW_Press == ON))
						Keys_State = BothSW_ON3s;
					else if(((LeftSW_Changed)||(RightSW_Changed))&&(Debounced_LeftSW_Press == ON)&&(Debounced_RightSW_Press == ON))
						Keys_State = BothSW_ON;
			
			
					// Pass the value of the button pressed to Queue.
					if(xQueueSend(SwitchesState_Queue, &Keys_State, portMAX_DELAY) != pdPASS)
					{
					// Error. The queue should never be full. If so print the
					// error message on UART and wait for ever.
					//UART_OutString("\nQueue full. This should never happen.\n");
						while(1){}
					}
				}
			}
		}
		
		// Received from Receiver: 50Hz
		if(tick_count%4 == 0){
			
			
		}
		
		// Received from UART:
		
		
		// Wait for the required amount of time to check back.
        vTaskDelayUntil(&ui16LastTime, SwitchDelay_ms / portTICK_RATE_MS);
	}
}


