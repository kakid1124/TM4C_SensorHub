#include "tm4c123gh6pm.h"
#include "I2C.h"
#include "Timer0A.h"

//****************************************//
// I2C1 Initialize (use for INA219 current sensors)
// SCL --> PA6
// SDA --> PA7
//****************************************//
void I2C1_Init(void){
  SYSCTL_RCGCI2C_R |= 0x0002;           // activate I2C1
  SYSCTL_RCGCGPIO_R |= 0x0001;          // activate port A
  while((SYSCTL_PRGPIO_R&0x0001) == 0){};// ready?

  GPIO_PORTA_AFSEL_R |= 0xC0;           // 3) enable alt funct on PA6,7
  GPIO_PORTA_ODR_R |= 0x80;             // 4) enable open drain on PA7 only
  GPIO_PORTA_DEN_R |= 0xC0;             // 5) enable digital I/O on PA6,7
                                        // 6) configure PA6,7 as I2C
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0x00FFFFFF)+0x33000000;
  GPIO_PORTA_AMSEL_R &= ~0xC0;          // 7) disable analog functionality on PA6,7
	
  I2C1_MCR_R = I2C_MCR_MFE;      				// 9) master function enable
  I2C1_MTPR_R = 9;              				// 8) configure for 400 kbps clock
  // 20*(TPR+1)*12.5ns = 10us, with TPR=39 --> 100kbps
	//  with TPR=9 --> 400kbps
}

//========================================//
//	I2C1 Functions:
//========================================//
uint32_t I2C1_Write_2Bytes(uint8_t device_address, uint8_t device_register, uint16_t device_data){
	uint8_t byte1 = (uint8_t)((device_data & 0xFF00) >> 8);
	uint8_t byte0 = (uint8_t)(device_data & 0x00FF);
	
	while(I2C1_MCS_R&I2C_MCS_BUSY){};				// wait for I2C ready
  I2C1_MSA_R = (device_address<<1)&0xFE;	// MSA[7:1] is slave address
  I2C1_MSA_R &= ~0x01;             				// MSA[0] is 0 for send
  I2C1_MDR_R = device_register;         	// register address
  I2C1_MCS_R = (0
                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                     //  & ~I2C_MCS_STOP    // no stop
                       | I2C_MCS_START    // generate start/restart
                       | I2C_MCS_RUN);    // master enable
	Delay_us(1);
		
  while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // check error bits
  if((I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
    I2C1_MCS_R = (0                // send stop if nonzero
                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // stop
                     //  & ~I2C_MCS_START   // no start/restart
                     //  & ~I2C_MCS_RUN   // master disable
                       );   
		Delay_us(1);
                                          // return error bits if nonzero
    return (I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
  }
  I2C1_MDR_R = byte1;         // prepare first byte
  I2C1_MCS_R = (0
                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                     //  & ~I2C_MCS_STOP    // no stop
                     //  & ~I2C_MCS_START   // no start/restart
                       | I2C_MCS_RUN);    // master enable
	Delay_us(1);
	
  while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // check error bits
  if((I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
    I2C1_MCS_R = (0                // send stop if nonzero
                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // stop
                     //  & ~I2C_MCS_START   // no start/restart
                     //  & ~I2C_MCS_RUN   // master disable
                        );
		Delay_us(1);
                                          // return error bits if nonzero
    return (I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
  }
  I2C1_MDR_R = byte0;         // prepare second byte
  I2C1_MCS_R = (0
                    //   & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // generate stop
                    //   & ~I2C_MCS_START   // no start/restart
                       | I2C_MCS_RUN);    // master enable
	Delay_us(1);
	
  while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // return error bits
  return (I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
}

uint16_t I2C1_Read_2Bytes(uint8_t device_address, uint8_t device_register){
	// Set Register Pointer
	while(I2C1_MCS_R&I2C_MCS_BUSY){};				// wait for I2C ready
  I2C1_MSA_R = (device_address<<1)&0xFE;	// MSA[7:1] is slave address
  I2C1_MSA_R &= ~0x01;             				// MSA[0] is 0 for send
  I2C1_MDR_R = device_register;         	// register address
  I2C1_MCS_R = (0
                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP    	// stop
                       | I2C_MCS_START    // generate start/restart
                       | I2C_MCS_RUN);    // master enable
	Delay_us(1);
		
  while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // check error bits
  if((I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
    I2C1_MCS_R = (0                // send stop if nonzero
                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // stop
                     //  & ~I2C_MCS_START   // no start/restart
                     //  & ~I2C_MCS_RUN   // master disable
                       );   
		Delay_us(1);
                                          // return error bits if nonzero
    return (I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
  }
	
	// Read data
	uint16_t receive = 0;

  while(I2C1_MCS_R&I2C_MCS_BUSY){};					// wait for I2C ready
		
  I2C1_MSA_R = (device_address<<1)&0xFE;    // MSA[7:1] is slave address
  I2C1_MSA_R |= 0x01;              					// MSA[0] is 1 for receive

  I2C1_MCS_R = (0
                  | I2C_MCS_ACK      // positive data ack
              //  & ~I2C_MCS_STOP    // no stop
                  | I2C_MCS_START    // generate start/restart
                  | I2C_MCS_RUN);    // master enable

	Delay_us(1);		// phai co delay moi lay du lieu duoc!
			
  while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for transmission done	
  receive = (uint16_t)((I2C1_MDR_R) << 8);       // MSB data sent first
			
  I2C1_MCS_R = (0
              //  & ~I2C_MCS_ACK     // negative data ack (last byte)
                  | I2C_MCS_STOP     // generate stop
              //  & ~I2C_MCS_START   // no start/restart
                  | I2C_MCS_RUN);    // master enable
			
	Delay_us(1);		// phai co delay moi lay du lieu duoc!
			
  while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for transmission done			
  receive += (uint16_t)(I2C1_MDR_R);       // LSB data sent last

	return receive;
}

//****************************************//
// I2C3 Initialize (use for MPU9150 sensor)
//****************************************//
void I2C3_Init(void){
  SYSCTL_RCGCI2C_R |= 0x0008;           // activate I2C3
  SYSCTL_RCGCGPIO_R |= 0x0008;          // activate port D
  while((SYSCTL_PRGPIO_R&0x0008) == 0){};// ready?

  GPIO_PORTD_AFSEL_R |= 0x03;           // 3) enable alt funct on PD1,0
  GPIO_PORTD_ODR_R |= 0x02;             // 4) enable open drain on PD1(SDA) only
  GPIO_PORTD_DEN_R |= 0x03;             // 5) enable digital I/O on PD1,0
                                        // 6) configure PD1,0 as I2C
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0xFFFFFF00)+0x00000033;
  GPIO_PORTD_AMSEL_R &= ~0x03;          // 7) disable analog functionality on PD1,0
  I2C3_MCR_R = I2C_MCR_MFE;      // 9) master function enable
  I2C3_MTPR_R = 9;              // 8) configure for 400 kbps clock
  // 20*(TPR+1)*12.5ns = 2500ns, with TPR=9
}

void I2C3_Init_8MHz(void){
  SYSCTL_RCGCI2C_R |= 0x0008;           // activate I2C3
  SYSCTL_RCGCGPIO_R |= 0x0008;          // activate port D
  while((SYSCTL_PRGPIO_R&0x0008) == 0){};// ready?

  GPIO_PORTD_AFSEL_R |= 0x03;           // 3) enable alt funct on PD1,0
  GPIO_PORTD_ODR_R |= 0x02;             // 4) enable open drain on PD1(SDA) only
  GPIO_PORTD_DEN_R |= 0x03;             // 5) enable digital I/O on PD1,0
                                        // 6) configure PD1,0 as I2C
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0xFFFFFF00)+0x00000033;
  GPIO_PORTD_AMSEL_R &= ~0x03;          // 7) disable analog functionality on PD1,0
  I2C3_MCR_R = I2C_MCR_MFE;      // 9) master function enable
  I2C3_MTPR_R = 0;              // 8) configure for 400 kbps clock
  // 20*(TPR+1)*125ns = 2500ns, with TPR=0
}
//========================================//
//	I2C3 Functions:
//========================================//
void I2C3_Write_Byte(uint8_t device_address, uint8_t device_register, uint8_t data){
	while(I2C3_MCS_R&I2C_MCS_BUSY){};				// wait for I2C ready
  I2C3_MSA_R = (device_address<<1)&0xFE;	// MSA[7:1] is slave address
  I2C3_MSA_R &= ~0x01;             				// MSA[0] is 0 for send
  I2C3_MDR_R = device_register;         	// register address
  I2C3_MCS_R = (0
                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                     //  & ~I2C_MCS_STOP    // no stop
                       | I2C_MCS_START    // generate start/restart
                       | I2C_MCS_RUN);    // master enable
	Delay_us(1);
		
  while(I2C3_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // check error bits
  if((I2C3_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
    I2C3_MCS_R = (0                // send stop if nonzero
                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // stop
                     //  & ~I2C_MCS_START   // no start/restart
                     //  & ~I2C_MCS_RUN   // master disable
                       );   
		Delay_us(1);
    return;
  }
  I2C3_MDR_R = data;         // send data
  I2C3_MCS_R = (0
                    //   & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // generate stop
                    //   & ~I2C_MCS_START   // no start/restart
                       | I2C_MCS_RUN);    // master enable
	Delay_us(1);
	
  while(I2C3_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
  return;
}

uint8_t I2C3_Read_Byte(uint8_t device_address, uint8_t device_register){
	// Set Register Address
	while(I2C3_MCS_R&I2C_MCS_BUSY){};				// wait for I2C ready
  I2C3_MSA_R = (device_address<<1)&0xFE;	// MSA[7:1] is slave address
  I2C3_MSA_R &= ~0x01;             				// MSA[0] is 0 for send
  I2C3_MDR_R = device_register;         	// register address
  I2C3_MCS_R = (0
                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                     //  & ~I2C_MCS_STOP   // no stop
                       | I2C_MCS_START    // generate start/restart
                       | I2C_MCS_RUN);    // master enable
	Delay_us(1);
		
  while(I2C3_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // check error bits
  if((I2C3_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
    I2C3_MCS_R = (0                // send stop if nonzero
                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // stop
                     //  & ~I2C_MCS_START   // no start/restart
                     //  & ~I2C_MCS_RUN   // master disable
                       );   
		Delay_us(1);
                                          // return error bits if nonzero
    return (I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
  }
	
	// Read data
  while(I2C3_MCS_R&I2C_MCS_BUSY){};					// wait for I2C ready
		
  I2C3_MSA_R = (device_address<<1)&0xFE;    // MSA[7:1] is slave address
  I2C3_MSA_R |= 0x01;              					// MSA[0] is 1 for receive

  I2C3_MCS_R = (0
              //  & ~I2C_MCS_ACK     // negative data ack (last byte)
                  | I2C_MCS_STOP     // generate stop
                  | I2C_MCS_START   // start
                  | I2C_MCS_RUN);    // master enable

	Delay_us(1);		// phai co delay moi lay du lieu duoc!
			
  while(I2C3_MCS_R&I2C_MCS_BUSY){};// wait for transmission done	
		
	return (uint8_t)I2C3_MDR_R;
}

void I2C3_Read_Bytes(uint8_t device_address, uint8_t device_register, uint8_t length, uint8_t * destination){
	// Set Register Address
	while(I2C3_MCS_R&I2C_MCS_BUSY){};				// wait for I2C ready
  I2C3_MSA_R = (device_address<<1)&0xFE;	// MSA[7:1] is slave address
  I2C3_MSA_R &= ~0x01;             				// MSA[0] is 0 for send
  I2C3_MDR_R = device_register;         	// register address
  I2C3_MCS_R = (0
                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                     //  & ~I2C_MCS_STOP   // no stop
                       | I2C_MCS_START    // generate start/restart
                       | I2C_MCS_RUN);    // master enable
	Delay_us(1);
		
  while(I2C3_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // check error bits
  if((I2C3_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
    I2C3_MCS_R = (0                // send stop if nonzero
                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // stop
                     //  & ~I2C_MCS_START   // no start/restart
                     //  & ~I2C_MCS_RUN   // master disable
                       );   
		Delay_us(1);
                                          // return error bits if nonzero
    return;
  }
	
	// Read data
	uint8_t received_data[14], i;
	
  while(I2C3_MCS_R&I2C_MCS_BUSY){};					// wait for I2C ready
		
  I2C3_MSA_R = (device_address<<1)&0xFE;    // MSA[7:1] is slave address
  I2C3_MSA_R |= 0x01;              					// MSA[0] is 1 for receive

	for(i = 0; i < length; i++){
		if(length == 1){
			I2C3_MCS_R = (0
              //  & ~I2C_MCS_ACK     // negative data ack (last byte)
                  | I2C_MCS_STOP     // generate stop
                  | I2C_MCS_START   // start
                  | I2C_MCS_RUN);    // master enable
		}
		else if(i == 0){
			I2C3_MCS_R = (0
              | I2C_MCS_ACK      // positive data ack
              //  & ~I2C_MCS_STOP    // no stop
              | I2C_MCS_START    // generate start/restart
              | I2C_MCS_RUN);    // master enable
		}
		else if(i == (length - 1)){
			I2C3_MCS_R = (0
              //  & ~I2C_MCS_ACK     // negative data ack (last byte)
                  | I2C_MCS_STOP     // generate stop
              //  & ~I2C_MCS_START   // no start/restart
                  | I2C_MCS_RUN);    // master enable
		}
		else{
			I2C3_MCS_R = (0
              | I2C_MCS_ACK      // positive data ack
              //  & ~I2C_MCS_STOP    // no stop
              //  & ~I2C_MCS_START   // no start/restart
              | I2C_MCS_RUN);    // master enable
		}
		Delay_us(1);		// phai co delay moi lay du lieu duoc!	
		while(I2C3_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
		
		received_data[i] = (uint8_t)I2C3_MDR_R;
		}
	
	for(i = 0; i < length; i++){
		destination[i] = received_data[i];
	}		
}

/* End of file*/
