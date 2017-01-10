
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_LOCK_KEY           0x4C4F434B  // Unlocks the GPIO_CR register
#define PF0       (*((volatile uint32_t *)0x40025004))
#define PF4       (*((volatile uint32_t *)0x40025040))
#define SWITCHES  (*((volatile uint32_t *)0x40025044))
#define SW1       0x10                      // on the left side of the Launchpad board
#define SW2       0x01                      // on the right side of the Launchpad board
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
#define Red_LED		(*((volatile uint32_t *)0x40025008))
#define Blue_LED	(*((volatile uint32_t *)0x40025010))
#define Green_LED	(*((volatile uint32_t *)0x40025020))
	

//Color 	LED(s) 	PortF
#define RED       0x02	// red      R--    0x02
#define BLUE      0x04	// blue     --B    0x04
#define GREEN     0x08	// green    -G-    0x08
#define YELLOW		0x0A	// yellow   RG-    0x0A
#define SKYBLUE		0x0C	// sky blue -GB    0x0C
#define WHITE			0x0E	// white    RGB    0x0E
#define PINK			0x06	// pink     R-B    0x06
#define ALL				0x00	// dark     ---    0


void PortF_Init(void);
void MotionInt_Init(void);
	
uint32_t PortF_Input(void);
void PortF_Output(uint32_t data);
void Scan_switch(void);
void Led_ON(uint8_t color);
void Led_OFF(uint8_t color);
void Led_Toggle(uint8_t color);

bool SW1_Pushed(void);
bool SW1_Release(void);

#ifdef __cplusplus
}
#endif
