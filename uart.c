#include "uart.h"
#include "stm32f030xx.h" // the modified Frank Duignan header file. (I started from his "Blinky" example). 

void InitUart(){
//USART1, 4800b8n1 (TX en SWCLK share a pin so this one is available on the pgming header, PA14 AF1)
USART1_BRR= 48000000/4800; //0x2710;        //48Mhz/4800b
USART1_CR1|= (1<<3|1<<0);  // enable transmitter, enable usart (must be done after config BR)

GPIOA_AFRH|=(1<<24);// switch PA14 to AF1, please note this does break debugging as that uses the same pin but as AF0
GPIOA_MODER|=(1<<29); // Switch PA14 to AF.
}

void UartSendByte(int data){
USART1_TDR=data&0x0000000F;
while(!(USART1_ISR&(1<<6))); // wait untill TC is set
}
