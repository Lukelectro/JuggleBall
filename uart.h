#ifndef __UART
#define __UART
#include "stm32f030xx.h" // the modified Frank Duignan header file. (I started from his "Blinky" example). 

void InitUart();
void UartSendByte(int);
#endif
