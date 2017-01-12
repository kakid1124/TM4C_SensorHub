
#include "tm4c123gh6pm.h"
#include "Timer0A.h"

void Timer0A_Delay_Init(void){
//  SYSCTL_RCGCTIMER_R |= 0x01;      // 0) activate timer0
//  TIMER0_CTL_R &= ~0x00000001;     // 1) disable timer0A during setup
//  TIMER0_CFG_R = 0x00000004;       // 2) configure for 16-bit timer mode
//  TIMER0_TAMR_R = 0x00000002;      // 3) configure for periodic mode, default down-count settings
//  TIMER0_TAILR_R = 0xFFFF;     		// 4) reload value
//  TIMER0_TAPR_R = 79;              // 5) 1us timer0A (clock 80MHz)
//  TIMER0_ICR_R = 0x00000001;       // 6) clear timer0A timeout flag
//  TIMER0_CTL_R |= 0x00000001;      // 10) enable timer0A

	SYSCTL_RCGCWTIMER_R |= 0x01;      // 0) activate wide timer 0
	WTIMER0_CTL_R &= ~0x00000001;     // 1) disable timer0A during setup
  WTIMER0_CFG_R = 0x00000004;       // 2) configure for 32-bit timer mode
  WTIMER0_TAMR_R = 0x00000002;      // 3) configure for periodic mode, default down-count settings
  WTIMER0_TAILR_R = 0xFFFFFFFF;     // 4) reload value
  WTIMER0_TAPR_R = 39;              // 5) 1us timer0A <== (clock 40MHz)/(39+1)
  WTIMER0_ICR_R = 0x00000001;       // 6) clear timer0A timeout flag
  WTIMER0_CTL_R |= 0x00000001;      // 10) enable timer0A
}

void Timer0A_Delay_Init_8MHz(void){
  SYSCTL_RCGCTIMER_R |= 0x01;      // 0) activate timer0
  TIMER0_CTL_R &= ~0x00000001;     // 1) disable timer0A during setup
  TIMER0_CFG_R = 0x00000004;       // 2) configure for 16-bit timer mode
  TIMER0_TAMR_R = 0x00000002;      // 3) configure for periodic mode, default down-count settings
  TIMER0_TAILR_R = 0xFFFF;     		// 4) reload value
  TIMER0_TAPR_R = 7;              // 5) 1us timer0A
  TIMER0_ICR_R = 0x00000001;       // 6) clear timer0A timeout flag
  TIMER0_CTL_R |= 0x00000001;      // 10) enable timer0A
}

void Delay_us(uint32_t us)
{
	volatile uint32_t elapsedTime;
	uint32_t startTime = WTIMER0_TAR_R;
	do{
		elapsedTime = (startTime - WTIMER0_TAR_R) & 0xFFFFFFFF;
	}
	while(elapsedTime <= us);
}

void Delay_ms(uint32_t milliseconds)
{
		uint32_t i;
		for(i=0; i<milliseconds; i++){
			Delay_us(1000);
		}
}
