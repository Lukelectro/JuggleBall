/*
NOTE/IDEA:
Clock ADC with dedictated 14Mhz clock. Set ADC sample rate by selecting sample time, if 71.5clks then adc samplerate should be 1/(71.5*(1/14E6)+12.5*(1/14E6)=166.67Khz
Check this by having ADC EOC interupt toggling a pin. (Yep! 167Khz!) That's for one ch. For 3ch the interrupt will trigger at the same rate but per channel samplerate will be lower. (Maybe sample slower to make sure ISR can keep up, and then check what effect that has on voltage/current regulation)

not to self: Once I start sampling at multiple chs, check samplerate for each ch.

Goal: 3ch PWM LED switchmode current source.
WORKS:  Subgoal 1: TIM3_CH2 PWM output (Pin13, PA7) , same freq, diff. D
WORKS:  Subgoal 2: TIM14_CH1 PWM output (Pin10, PA4), same freq, diff. D
WORKS:  subgoal 3: Multichannel ADC measurements. 
WORKS:  subgoal 4: Choose reference "voltage" (ADC value) / sense resistor values wiseley (for 0-30mA):
WORKS:  Subgaol 5: Close the feedback loops. And maybe test with resistors first so the led's stay intact.

For the smps use tim3_psc=0 (48Mhz/1) and TIM3_ARR=2048, so 23.4Khz. Note that the compare CCR should be below or equal to ARR (To be usefull)

TODO: Cleanup comments/notes etc.
Next goal: Read MPU-6050 and control the LED's with it.
*/

#include "stm32f030xx.h" // the Frank Duignan header file. (I started from his "Blinky" example). 
// I realy should use ST provided files, so I'm not dependant on some guys' blog. (Includes, linkerscripts, makefile, init. Though I could (learn to) write my own...)


//MAX setpoints 
//(2^12/3v3 * 1.8*5/11.8) = 947 --- 5V uit, 3v3 ref. 12 bit adc 10k/1k8 div.
//(2^12/3v3 * 0.05 * 15)= 930 -- 50mA max uit, 3v3 ref, 12Bit adc, 15R sense resistor. (60mA is AMR for the LED's I use)
//(2^12/3v3 * 0.02 * 15)= 372 -- 20mA max uit, 3v3 ref, 12Bit adc, 15R sense resistor.
#define SETPOINT1 372 
#define SETPOINT2 372 
#define SETPOINT3 372 
#define SETPOINT 372

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
        
        // enable pheripheral clock to timer3 (BIT1) and TIM14 (BIT8). (Easily forgotten. Beginner mistake)
        RCC_APB1ENR |= (BIT1 | BIT8);
        
        RCC_APB2ENR |= BIT9; // enable clock to adc
        RCC_CR |= BIT0; // turn on HSI clock (For ADC) // TODO: I could use the prescaled main clock...
        while(!(RCC_CR & BIT1)); // wait till HSI is stable
        
}



volatile int adcresult; // can be read in debugger too.
int setpoints[3]={SETPOINT1,SETPOINT2,SETPOINT3};	


int main()
{
	initClock();
	
	// Power up PORTA
	RCC_AHBENR |= BIT17;
	
	GPIOA_MODER |= ( BIT0 | BIT9 | BIT13 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7 | BIT15) ; // make PA0 an output (Pin6, BIT1), PA4 (pin10, BIT9) to AF (TIM14CH1), PA6/pin12 (Bit13) AF (timer), and PA1,2,3/pin7,8,9 analog (BIT2,3,bit4,5,Bit6 and 7, reps), PA7 AF (TIM3_CH2) Bit 15. 
	
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

        // Set AF(AF1) en MODER to select PWM output on pins
	GPIOA_AFRL |= (BIT24 |BIT28 | BIT18); ; // Bit27:24 for AFR6 / PA6, that should get set to AF1 (0001) for TIM3CH1, BIT28 is idem for PA7/TIM3ch2. And bit 18 for  AF4 on PA4: TIM14CH1 (PWM)
	
        //set up timer 14 for PWM
        TIM14_PSC = 0; // prescaler. (48Mhz/psc+1=tim14clock)
        TIM14_ARR = 2048;  // 16 bit timer, AutoReloadRegister (frequency) (48E6/((TIM14_PSC+1)*TIM14_ARR)
        TIM14_CCMR1 |= (BIT3 | BIT5 | BIT6 ) ;      // PWM mode (per output bit 4:6). Set OC1PE (bit5) preload enable. 
        TIM14_CCER |= (BIT0) ; //  CC1E (bit 0) to enable output on ch1.
        TIM14_CR1 |= BIT7 ;        // Control register. Set ARPE (bit7). And CEN I suppose (Counter enable, bit 0)
        TIM14_EGR |= BIT0 ; // set UG to generate update event so registers are read to the timer
        TIM14_CR1 |= BIT0 ; // start after updating registers!
        


        // Wait for ADCAL to be zero again:
        while (ADC_CR & (BIT31));
        // then power up and set up adc:
        ADC_CR |= (BIT0); // Set ADEN / enable power to adc BEFORE making settings!
        
        while (!(ADC_ISR&BIT0));// check ADCRDY (In ADC_ISR, bit0) to see if ADC is ready for further settings/starting a coversion
        
        // make rest of settings before starting conversion:
        ADC_CHSELR = (BIT3 | BIT2 | BIT1); // Ch3 = PA3, on pin9 CH2 = PA2 pin 8, CH1 =PA1 pin 7. (Set up channels)
        // It will scan all these channels, but it has only 1 data register for the result. So it will scan them (Low-High is default, so CH1,2,3,1,2,3,)
        ADC_CFGR1 |= (BIT12 | BIT13); // BIT12 set it to discard on overrun and overwrite with latest result 
                               // BIT16: DISCEN Discontinues operation (Don't auto scan, wait for trigger to scan next ch, cannot be used when CONT=1)
                               // BIT13: CONT. automatically restart conversion once previous conversion is finished. 
        ADC_SMPR |= ( BIT1 | BIT2 | BIT3); // Set sample rate (Default = as fast as it can: 1.5clk, with bit1&2 set 71.5clk, with just bit 1: 13.5clk)  TODO:Adjust so no OVF   
        
        ADC_IER |=(BIT2|BIT3) ; // Enable end of conversion interrupt (Bit2), and EOSEQ (End of Sequence) bit 3. 
        
        
        /* from code example, on howto enable interrupt in NVIC. But nowhere in datasheet does it say how to init NVIC whithout those functions... 
        NVIC_EnableIRQ(ADC1_COMP_IRQn); // enable ADC interrupt
        NVIC_SetPriority(ADC1_COMP_IRQn,2); // set priority (to 2)
        Fortunately the STM32F0xxx Cortex-M0 programming manual (PM0215) does document the NVIC. somewhat. And the CMSIS libs can be downloaded from st.com
        I could just use them... But I don't :)
        */
        
        ISER |= (BIT12); // Enable IRQ12, (That's the adc)
        IPR3 |= 96; // set priority for IRQ12 (4*IPRn+IRQn), starting from 0, so for IRQ12 that's IPR3 bits 7 downto 0
        //Read the relevant part of PM0215. IRC number is the position listed in RM0360 table 11.1.3.
        
        while (!(ADC_ISR&BIT0));// check ADCRDY (In ADC_ISR, bit0) to see if ADC is ready for starting a coversion
        
        ADC_CR |= (BIT2); // Set ADSTART to start conversion

	while(1)
	{	
	
		//Fade R,G,B.
	   	setpoints[0]=0; //G
	   	setpoints[1]=SETPOINT; //R
	   	setpoints[2]=0; //B
	   	
	
		for(int i=0;i<SETPOINT;i++){
		setpoints[1]=SETPOINT-i;
		setpoints[2]=i;
		delay(5000);
		}	    
	    
	
		for(int i=0;i<SETPOINT;i++){
		setpoints[2]=SETPOINT-i;
		setpoints[0]=i;
		delay(5000);
		}	    
	
		
	
		for(int i=0;i<SETPOINT;i++){
		setpoints[0]=SETPOINT-i;
		setpoints[1]=i;
		delay(5000);
		}	    
		
	    
	     
		// TODO: Main loop. Because voltage regulation is all done in interrupt.
	} 
	return 0;
}



void ADC_Handler(){
        GPIOA_BSRR = (BIT0); // SET PA0 (To time handler)
        
        static int ch=0; // keep between invocations
        static int pwm[3];
        //static int setpoints[3]={SETPOINT1,SETPOINT2,SETPOINT3}; // TODO: later to be set from main.
        //static int OVFs = 0, resyncs=0; // for debug purposes.
        
        if(ADC_ISR&(BIT2)) // Check EOC (Could check EOSEQ when the sequence is only 1 conversion long)
                {
                adcresult=ADC_DR; // read adc result for debugger/global use.
                
                pwm[ch] += (setpoints[ch]-adcresult); // integrating comparator.  
                // This gets out of sync sometimes. (Feedback from one ch controlling another. Not good.)
                // Especially debugging throws it out of sync but sometimes after reset it is another ch as well.
            	// (Another ADC interrupt before ch is incremented, OK, for debug I understand how that can happen, but why it hapens after reset?)
                
                if (pwm[ch]<0) pwm[ch]= 0; else if (pwm[ch]>1024) pwm[ch]=1024; //max 50% D.
                TIM3_CCR1 = pwm[2];
                TIM3_CCR2 = pwm[1];
              	TIM14_CCR1 = pwm[0];
                //if(ch<2) ch++; else ch=0; // could do this entirely with OESEQ and just ch++ here... Is slightly faster
                ch++;
                }
        /*        
        if(ADC_ISR&(BIT4)){ // OVF monitoring (flag is set even if OVF interrupt is not enabled)
        	OVFs++; 
        	ADC_ISR&=BIT4; // reset flag	
        }
        */
        
        if(ADC_ISR&(BIT3)){ // EOSEQ is used to resync.
		//if(ch!=0) resyncs++; 
        	ch=0; 
        	ADC_ISR&=(BIT3); // reset flag
        }
        
        GPIOA_BSRR =(BIT16);//  clear PA0 after running this handler. (To time handler and check sample rate)
	
}




