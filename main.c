#include "stm32f030xx.h" // the Frank Duignan header file. (I started from his "Blinky" example). 

// I realy should use ST provide files, so I'm not dependant on some guys' blog. (Includes, linkerscripts, makefile. Though I could (learn to) write my own...)

void delay(int dly)
{
  while( dly--);
}

void initClock()
{
// This is potentially a dangerous function as it could
// result in a system with an invalid clock signal - result: a stuck system
        // Set the PLL up
        // First ensure PLL is disabled
        RCC_CR &= ~BIT24;
        while( (RCC_CR & BIT25)); // wait for PLL ready to be cleared
        // set PLL multiplier to 12 (yielding 48MHz)
  // Warning here: if system clock is greater than 24MHz then wait-state(s) need to be
        // inserted into Flash memory interface
        FLASH_ACR |= BIT0;
        FLASH_ACR &=~(BIT2 | BIT1);

        // Turn on FLASH prefetch buffer
        FLASH_ACR |= BIT4;
        RCC_CFGR &= ~(BIT21 | BIT20 | BIT19 | BIT18);
        RCC_CFGR |= (BIT21 | BIT19 ); 

        // Need to limit ADC clock to below 14MHz so will change ADC prescaler to 4
        RCC_CFGR |= BIT14;


        // and turn the PLL back on again
        RCC_CR |= BIT24;        
        // set PLL as system clock source 
        RCC_CFGR |= BIT1;
        
        // enable pheripheral clock to timer. (Easily forgotten. Beginner mistake)
        RCC_APB1ENR |= BIT1;
}



	


int main()
{
	initClock();
	
	// Power up PORTA
	RCC_AHBENR |= BIT17;
	
	GPIOA_MODER |= BIT8; // make bits 14 an output (PA4 is PIN10 is BIT8)
	
	//set up timer 3 for PWM
	TIM3_PSC = 2; // prescaler. (48Mhz/psc=tim3clock)
        TIM3_ARR = 0xFFFF;  // 16 bit timer, AutoReloadRegister (frequency)
        TIM3_CCR1 = 0xEFFF; // Compare register 1, dutycycle on output 1 (It has 4)
        TIM3_CCMR1 |= (BIT3 | BIT5 | BIT6) ;      // PWM mode (per output bit 4:6). Set OC1PE (bit5)
        TIM3_CCER |= BIT0 ; // CC1P to set polarity of output (0=active high), CC1E (bit 0) to enable output
        TIM3_CR1 |= BIT7 ;        // Control register. Set ARPE (bit7). And CEN I suppose (Counter enable, bit 0)
        TIM3_EGR |= BIT0 ; // set UG to generate update event so registers are read to the timer
        TIM3_CR1 |= BIT0 ; // start after updating registers!

        // Set AF(AF1) en MODER to select PWM output on pin PA6 (Is the only option on this low-pincount stm32f030c4 device in SSOP20)
	GPIOA_AFRL |= BIT24 ; // Bit27:24 for AFR6 / PA6, that should get set to AF1 (0001) for TIM3CH1 
	GPIOA_MODER |= BIT13; // to set to AF (And not BIT12 for OUTPUT)
	while(1)
	{			
		GPIOA_ODR |= BIT4; // bit 4 for A4
		//TIM3_CCR1 = 0x01FF; // Compare register 1, dutycycle on output 1 (It has 4)
		//delay(1000000);
		for(int i=0;i<0xFFFF;i++) {
		        TIM3_CCR1 = i;
		        delay(100);        
		} 
		GPIOA_ODR &= ~BIT4; 
		//TIM3_CCR1 = 0x1EFF; // Compare register 1, dutycycle on output 1 (It has 4)
		//delay(1000000);
		for(int i=0xFFFF;i>0;i--) {
		        TIM3_CCR1 = i; 
		        delay(100);
		 }
	} 
	return 0;
}








