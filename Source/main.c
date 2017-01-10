//*************************************************//
//**** Tiva C - FreeRTOS Project ****//
//
// Nguyen Huynh Da Khoa - Version 1.0 - 10.01.2017 //
//
// System clock: 8 ~ 40MHz
//
//*************************************************//

#include "include.h"

#define Highspeed_Clock		// System clock: 40MHz, baudrate: 115200bps

//**** Function Prototype ****//
#ifdef __cplusplus
extern "C" {
#endif

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

void System_Init(void);
	
#ifdef __cplusplus
}
#endif

//*****************************************************************************
//
// The mutex that protects concurrent access of UART from multiple tasks.
//
//*****************************************************************************
xSemaphoreHandle g_pUARTSemaphore;

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}


//**************************************//
//
//************ Main Program ************//	
//
//**************************************//

int main()
{
	System_Init();
	
	UART_OutString("\n\nWelcome to the FreeRTOS Demo...\n");
	
    //
    // Create a mutex to guard the UART.
    //
    g_pUARTSemaphore = xSemaphoreCreateMutex();

    //
    // Create the LED task.
    //
    if(LEDTaskInit() != 0)
    {

        while(1)
        {
        }
    }

    //
    // Create the switch task.
    //
    if(SwitchTaskInit() != 0)
    {

        while(1)
        {
        }
    }

    //
    // Start the scheduler.  This should not return.
    //
    vTaskStartScheduler();

    //
    // In case the scheduler returns for some reason, print an error and loop
    // forever.
    //
		
		while(1)
		{
		}
}


//*************************************************//
//**** Systems_Init ****//
//
//*************************************************//
void System_Init(void)
{
#ifdef Highspeed_Clock
	PLL_Init();			// System Clock: 40MHz
	UART_Init();		// UART: 115200 bps
#else	
	PLL_Init_8MHz(); 	// System Clock: 8MHz
	UART_Init_9600bps();// UART: 9600 bps
#endif	

//	PortF_Init();	
//	EnableInterrupts();
	
	FPUEnable();
	FPULazyStackingEnable();
	
}

