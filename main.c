/*
NOTE/IDEA:
Clock ADC with dedictated 14Mhz clock. Set ADC sample rate by selecting sample time, if 71.5clks then adc samplerate should be 1/(71.5*(1/14E6)+12.5*(1/14E6)=166.67Khz
Check this by having ADC EOC interupt toggling a pin. (Yep! 167Khz!)

not to self: Once I start sampling at multiple chs, check samplerate for each ch.

Goal: 3ch PWM LED switchmode current source.
WORKS:  Subgoal 1: TIM3_CH2 PWM output (Pin13, PA7) , same freq, diff. D
TODO:   subgoal 2: TIM14_CH1 PWM output (Pin10, PA4), same freq, diff. D
        subgoal 3: Multichannel ADC measurements
        subgoal 4: Choose reference "voltage" (ADC value) / sense resistor values wiseley (for 0-30mA)
        Subgaol 5: Close the feedback loop. All 3. Not at once. And maybe test with resistors first so the led's stay intact.

For the smps use tim3_psc=0 (48Mhz/1) and TIM3_ARR=2048, so 23.4Khz. Note that the compare CCR should be below or equal to ARR (To be usefull)
*/

#include "stm32f030xx.h" // the Frank Duignan header file. (I started from his "Blinky" example). 
// I realy should use ST provided files, so I'm not dependant on some guys' blog. (Includes, linkerscripts, makefile, init. Though I could (learn to) write my own...)


#define SETPOINT 947 // (2^10/3v3 * 1.8*5/11.8) = 947 --- 5V uit, 3v3 ref. 10 bit adc 10k/1k8 div.

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



volatile int adcresult; // can be read in debugger too. Global because set from ISR.
	


int main()
{
	initClock();
	
	// Power up PORTA
	RCC_AHBENR |= BIT17;
	
	GPIOA_MODER |= ( BIT8 | BIT13 | BIT6 | BIT7 | BIT15) ; // make PA4 an output (PA4 is PIN10 is BIT8), and PA6/pin12 (Bit13) AF (timer), and PA3/pin9 analog (Bit6 and 7), PA7 AF (TIM3_CH2) Bit 15.
	//Before enabling ADC, let it calibrate itself by settin ADCAL (And waiting 'till it is cleared again before enabling ADC)
        ADC_CR |= (BIT31); // set adcal	

	//set up timer 3 for PWM
	TIM3_PSC = 0; // prescaler. (48Mhz/psc+1=tim3clock)
        TIM3_ARR = 2048;  // 16 bit timer, AutoReloadRegister (frequency) (48E6/((TIM3_PSC+1)*TIM3_ARR)
        //TIM3_CCR1 = 2048; // Compare register 1, dutycycle on output 1 (It has 4)
        TIM3_CCMR1 |= (BIT3 | BIT5 | BIT6 | BIT11 | BIT14 | BIT13) ;      // PWM mode (per output bit 4:6). Set OC1PE (bit5), OC2PE (bit11) preload enable, PWM mode for ch2 (BIT 14,13) 
        TIM3_CCER |= (BIT0 | BIT4) ; // CC1P to set polarity of output (0=active high), CC1E (bit 0) to enable output on ch1, bit4 for ch2.
        TIM3_CR1 |= BIT7 ;        // Control register. Set ARPE (bit7). And CEN I suppose (Counter enable, bit 0)
        TIM3_EGR |= BIT0 ; // set UG to generate update event so registers are read to the timer
        TIM3_CR1 |= BIT0 ; // start after updating registers!

        // Set AF(AF1) en MODER to select PWM output on pin PA6 (Is the only option on this low-pincount stm32f030c4 device in SSOP20)
	GPIOA_AFRL |= (BIT24 |BIT28); ; // Bit27:24 for AFR6 / PA6, that should get set to AF1 (0001) for TIM3CH1, BIT28 is idem for PA7/TIM3ch2.
	
        // Wait for ADCAL to be zero again:
        while (ADC_CR & (BIT31));
        // then power up and set up adc:
        ADC_CR |= (BIT0); // Set ADEN / enable power to adc BEFORE making settings!
        
        while (!(ADC_ISR&BIT0));// check ADCRDY (In ADC_ISR, bit0) to see if ADC is ready for further settings/starting a coversion
        
        // make rest of settings before starting conversion:
        ADC_CHSELR = (BIT3); // Ch3 = PA3, on pin9. (Set up channels)
        // It will scan all these channels, but it has only 1 data register for the result.
        ADC_CFGR1 |= (BIT12 | BIT13); // BIT12 set it to discard on overrun and overwrite with latest result (Since I'm only using one ch)
                               // BIT16: DISCEN Discontinues operation (Don't auto scan, wait for trigger to scan next ch, cannot be used when CONT=1)
                               // BIT13: CONT. automatically restart conversion once previous conversion is finished.
        ADC_SMPR |= (BIT2 | BIT1); // Set sample rate (Default = as fast as it can: 1.5clk, with bit1&2 set 71.5clk)     
        
        ADC_IER |= BIT2; // Enable end of conversion interrupt.
        
        
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
	     
	     for(int i=0;i<2048;i++){ // fade LED on TIM3CH2.
	     TIM3_CCR2 = i;
	     delay(1000);
	     }
	     for(int i=2048;i>0;i--){ // fade LED on TIM3CH2.
	     TIM3_CCR2 = i;
	     delay(1000);
	     }
	   
	     
		// TODO: Main loop. Because voltage regulation is all done in interrupt.
	} 
	return 0;
}



void ADC_Handler(){
        /* toggle LED when handler gets run. (To check samplerate)
        GPIOA_ODR |= BIT4;
        GPIOA_ODR &= ~BIT4;
        */
        static int pwm=0; // keep between invocations
        if(ADC_ISR&(BIT2)) // Check EOC (Could check EOS when the sequence is only 1 conversion long)
                {
                adcresult=ADC_DR; // read adc result for debugger/global use.
                pwm += (SETPOINT-adcresult); // integrating comparator.
                if (pwm<0) pwm= 0;
                if (pwm>1024) pwm=1024; //max 50% D.
                TIM3_CCR1 = pwm;
                }
                
        // TODO: enable & check OVF.	
}




