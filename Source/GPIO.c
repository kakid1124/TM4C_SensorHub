// negative logic switches connected to PF0 and PF4 on the Launchpad
// red LED connected to PF1 on the Launchpad
// blue LED connected to PF2 on the Launchpad
// green LED connected to PF3 on the Launchpad
// NOTE: The NMI (non-maskable interrupt) is on PF0.  That means that
// the Alternate Function Select, Pull-Up Resistor, Pull-Down Resistor,
// and Digital Enable are all locked for PF0 until a value of 0x4C4F434B
// is written to the Port F GPIO Lock Register.  After Port F is
// unlocked, bit 0 of the Port F GPIO Commit Register must be set to
// allow access to PF0's control registers.  On the LM4F120, the other
// bits of the Port F GPIO Commit Register are hard-wired to 1, meaning
// that the rest of Port F can always be freely re-configured at any
// time.  Requiring this procedure makes it unlikely to accidentally
// re-configure the JTAG pins as GPIO, which can lock the debugger out
// of the processor and make it permanently unable to be debugged or
// re-programmed.


#include "tm4c123gh6pm.h"
#include "GPIO.h"

void PortF_Init(void){ volatile uint32_t delay;
  SYSCTL_RCGCGPIO_R |= 0x00000020;  // 1) activate clock for Port F
  delay = SYSCTL_RCGCGPIO_R;        // allow time for clock to start
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 in, PF3-1 out
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
  GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital I/O on PF4-0
}

void MotionInt_Init(void){
	// Configure GPIO for Motion Interrupt: PB2
	SYSCTL_RCGCGPIO_R |= 0x02;		// (a) activate port B
	while((SYSCTL_PRGPIO_R&0x0002) == 0){};
		
	GPIO_PORTB_DIR_R &= ~0x04;		// (c) PB2 is Input
	GPIO_PORTB_AFSEL_R &= ~0x04;  //     disable alt funct on PB2
	GPIO_PORTB_AMSEL_R &= ~0x04;  //     disable analog functionality on PB2
//	GPIO_PORTB_PUR_R |= 0x04;     //     enable weak pull-up on PB2
	GPIO_PORTB_DEN_R |= 0x04;			// (d) enable digital I/O on PB2
	GPIO_PORTB_IS_R &= ~0x04;			//		 PB2 is edge-sensitive
	GPIO_PORTB_IBE_R &= ~0x04;		//		 PB2 is not both edges
	GPIO_PORTB_IEV_R &= ~0x04;		//		 PB2 is falling edge event
	GPIO_PORTB_ICR_R = 0x04;			// (e) clear flag2
	GPIO_PORTB_IM_R |= 0x04;			// (f) arm interrupt on PB2
	
	NVIC_PRI0_R = (NVIC_PRI0_R & 0xFFFF00FF) | 0x0000A000; // (g) priority 5
	NVIC_EN0_R |= 2;							// (h) enable interrupt 1 in NVIC
}

bool SW1_Pushed(void){
	if((GPIO_PORTF_DATA_R&0x10) == 0)
		return true;
	else
		return false;
}

bool SW1_Release(void){
	if((GPIO_PORTF_DATA_R&0x10) != 0)
		return true;
	else
		return false;
}

uint32_t PortF_Input(void){     
  return (GPIO_PORTF_DATA_R&0x11);  // read PF4,PF0 inputs
}

void PortF_Output(uint32_t data){ // write PF3-PF1 outputs
  GPIO_PORTF_DATA_R = data;      
}

void Scan_switch(void){ uint32_t status;
  status = PortF_Input();
  switch(status){                    // switches are negative logic on PF0 and PF4
		case 0x01: PortF_Output(BLUE); break;   // SW1 pressed (PF4)
    case 0x10: PortF_Output(RED); break;    // SW2 pressed (PF0)
    case 0x00: PortF_Output(GREEN); break;  // both switches pressed
    case 0x11: PortF_Output(0); break;      // neither switch pressed    
  }
}

void Led_ON(uint8_t color){
	PortF_Output(color);
}

void Led_OFF(uint8_t color){
	switch(color){
		case RED: Red_LED = 0; break;
		case GREEN: Green_LED = 0; break;
		case BLUE: Blue_LED = 0; break;
		default: PortF_Output(0); break;
	}
}

void Led_Toggle(uint8_t color){
	switch(color){
		case RED: Red_LED ^= RED; break;
		case GREEN: Green_LED ^= GREEN; break;
		case BLUE: Blue_LED ^= BLUE; break;
	}
}
