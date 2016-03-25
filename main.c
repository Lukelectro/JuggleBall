/*
NOTE/IDEA:
Clock ADC with dedictated 14Mhz clock. Set ADC sample rate by selecting sample time, if 71.5clks then adc samplerate should be 1/(71.5*(1/14E6)+12.5*(1/14E6)=166.67Khz
Check this by having ADC EOC interupt toggling a pin. 

not to self: Once I start sampling at multiple chs, check samplerate for each ch.

Goal: 3ch PWM LED switchmode current source. But for now a single ch. boost converter 3v3 ->5V. And before that just a ASC sampling at the right rate and a PMW driving a LED at a frequency that's lower then finaly will be used for the smps.

For the smps use tim3_psc=0 (48Mhz/1) and TIM3_ARR=2048, so 23.4Khz. Note that the compare might need to stay below ARR (Read The Fine Manual).
*/

#include "stm32f030xx.h" // the Frank Duignan header file. (I started from his "Blinky" example). 
// I realy should use ST provided files, so I'm not dependant on some guys' blog. (Includes, linkerscripts, makefile, init. Though I could (learn to) write my own...)

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
        
        RCC_APB2ENR |= BIT9; // enable clock to adc
        RCC_CR |= BIT0; // turn on HSI clock (For ADC)
        while(!(RCC_CR & BIT1)); // wait till HSI is stable
        
}



int adcresult; // can be read in debugger too. Global because set from ISR.
	


int main()
{
	initClock();
	
	// Power up PORTA
	RCC_AHBENR |= BIT17;
	
	GPIOA_MODER |= ( BIT8 | BIT13 | BIT6 | BIT7 ) ; // make PA4 an output (PA4 is PIN10 is BIT8), and PA6/pin12 (Bit13) AF (timer), and PA3/pin9 analog (Bit6 and 7)
	
	//Before enabling ADC, let it calibrate itself by settin ADCAL (And waiting 'till it is cleared again before enabling ADC)
        ADC_CR |= (BIT31); // set adcal	

	//set up timer 3 for PWM
	TIM3_PSC = 0; // prescaler. (48Mhz/psc+1=tim3clock)
        TIM3_ARR = 0xFFFF;  // 16 bit timer, AutoReloadRegister (frequency) (48E6/((TIM3_PSC+1)*TIM3_ARR)
        TIM3_CCR1 = 0xEFFF; // Compare register 1, dutycycle on output 1 (It has 4)
        TIM3_CCMR1 |= (BIT3 | BIT5 | BIT6) ;      // PWM mode (per output bit 4:6). Set OC1PE (bit5)
        TIM3_CCER |= BIT0 ; // CC1P to set polarity of output (0=active high), CC1E (bit 0) to enable output
        TIM3_CR1 |= BIT7 ;        // Control register. Set ARPE (bit7). And CEN I suppose (Counter enable, bit 0)
        TIM3_EGR |= BIT0 ; // set UG to generate update event so registers are read to the timer
        TIM3_CR1 |= BIT0 ; // start after updating registers!

        // Set AF(AF1) en MODER to select PWM output on pin PA6 (Is the only option on this low-pincount stm32f030c4 device in SSOP20)
	GPIOA_AFRL |= BIT24 ; // Bit27:24 for AFR6 / PA6, that should get set to AF1 (0001) for TIM3CH1 
	
        // Wait for ADCAL to be zero again:
        while (ADC_CR & (BIT31));
        // then power up and set up adc:
        ADC_CR |= (BIT0); // Set ADEN / enable power to adc BEFORE making settings!
        
        while (!(ADC_ISR&BIT0));// check ADCRDY (In ADC_ISR, bit0) to see if ADC is ready for further settings/starting a coversion
        
        // make rest of settings before starting conversion:
        ADC_CHSELR = (BIT3); // Ch3 = PA3, on pin9. (Set up channels)
        // It will scan all these channels, but it has only 1 data register for the result.
        ADC_CFGR1 |= (BIT12 ); // BIT12 set it to discard on overrun and overwrite with latest result (Since I'm only using one ch)
                               // BIT16: DISCEN Discontinues operation (Don't auto scan, wait for trigger to scan next ch, cannot be used when CONT=1)
                               // BIT13: CONT. automatically restart conversion once previous conversion is finished.
        // ADC_SMPR |= BIT2; TODO: Set sample rate (Default = as fast as it can: 1.5clk)     
        
        ADC_IER |= BIT2; // Enable end of conversion interrupt. TODO: Still get no interrupt...
        
        
        /* from code example, on howto enable interrupt in NVIC. But nowhere in datasheet does it say how to init NVIC whithout those functions... 
        NVIC_EnableIRQ(ADC1_COMP_IRQn); // enable ADC interrupt
        NVIC_SetPriority(ADC1_COMP_IRQn,2); // set priority (to 2)
        Fortunately the STM32F0xxx Cortex-M0 programming manual (PM0215) does document the NVIC. somewhat. And the CMSIS libs can be downloaded from st.com
        I could just use them... But I don't :)
        */
        
        ISER |= (BIT12); // Enable IRQ12, (That's the adc)
        IPR3 |= 96; // set priority for IRQ12 (4*IPRn+IRQn), starting from 0, so for IRQ12 that's IPR3 bits 7 downto 0
        //NVIC is not well documented imho, but the above is tested and WORKS. Read the relevant part of PM0215. IRC number is the position listed in RM0360 table 11.1.3.
        
        while (!(ADC_ISR&BIT0));// check ADCRDY (In ADC_ISR, bit0) to see if ADC is ready for starting a coversion
        
        ADC_CR |= (BIT2); // Set ADSTART to start conversion

	while(1)
	{	
	        /*	
	        move this to interrupt:
	        
	        // Poll for adc result ready, then copy and start a new one. (Could do this with DMA or interrupt... Should learn howto DMA)
                int adcresult; // can be read in debugger too.
                if(ADC_ISR&(BIT2)) // Check EOC (Could check EOS when the sequence is only 1 conversion long)
                {
                adcresult=ADC_DR;
                ADC_ISR|=(BIT2); // clear EOC flag.
                ADC_CR |= (BIT2); // ADSTART
                }
		*/
		
                // TODO: PWM LED with ADC result.

		//GPIOA_ODR |= BIT4; // bit 4 for A4
		for(int i=0;i<0xFFFF;i++) {
		        TIM3_CCR1 = i;
		        delay(100);        
		} 
		//GPIOA_ODR &= ~BIT4; 
		for(int i=0xFFFF;i>0;i--) {
		        TIM3_CCR1 = i; 
		        delay(100);
		 }
	} 
	return 0;
}



void ADC_Handler(){
        // toggle LED when handler gets run. (To check samplerate)
        GPIOA_ODR |= BIT4;
        GPIOA_ODR &= ~BIT4;
        
        if(ADC_ISR&(BIT2)) // Check EOC (Could check EOS when the sequence is only 1 conversion long)
                {
                adcresult=ADC_DR;
                // ADC_ISR|=(BIT2); // clear EOC flag. (Gets auto-cleared when reading ADC_DR)
                ADC_CR |= (BIT2); // ADSTART to start next conversion (Or set CONT in ADC_CFGR1)
                }
		
}




