/*
NOTES, IDEAS, CURRENT STATE, TITLE, LICENSE, ETC. SHOULD GO HERE.

Use tap, freefal, activity and inactivity interrupts from ADXL345.
Inactivity: Fade trough rainbow colours a few times (say, 10s) then go to sleep.
Activity: Wake up / Folow "juggle" protocol
tap: change mode on tripple tap
Freefall: Detect catch as end of freefall

juggle protocol: Change colour on catch (Jump through a set of predefined pretty coulours) and maybe a few other modes

(change mode on (tripple) tap, but only if there has not been a freefall for a while, to prevent mode switching during juggling)
 
Maybe TODO: Change color at higest point or start of freefal? Flash at catch?
TODO: code is getting messy so clean up;

*/
#include <stdbool.h>
#include "stm32f030xx.h" // the modified Frank Duignan header file. (I started from his "Blinky" example).
#include "adxl345.h" // for the register names
#include "config.h"  // to config max current / array of "pretty colors" etc.
#include "adxl_i2c.h"


volatile int adcresult; // can be read in debugger too.
int setpoints[3]={0,0,0};
unsigned int tick=0; // Gets ++t in adc handler and used for timeouts

void delay(int dly)
{
  while( dly--);
}
void setup_pins(){ // so this is the same in init and in return from sleep 
	//enable clock to Porta (BIT17), B (18), F (22)
	RCC_AHBENR |= BIT17 | BIT18 | BIT22;


	GPIOA_MODER |= ( BIT0 | BIT9 | BIT13 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7 | BIT15 | BIT19 | BIT21) ; // make PA0 an output (Pin6, BIT0), PA4 (pin10, BIT9) to AF (TIM14CH1), PA6/pin12 (Bit13) AF (timer), and PA1,2,3/pin7,8,9 analog (BIT2,3,bit4,5,Bit6 and 7, reps), PA7 AF (TIM3_CH2) Bit 15. PA9/10 AF4 (I2C1)

	// Set unused pins to a defined state so floating inputs do not consume power
	GPIOF_MODER |= BIT0|BIT1|BIT2|BIT3; // PF0 and PF1 to Analog Input
	GPIOB_MODER |= BIT2|BIT3; //PB1 to AIN
	
		//allright, just because the TSSOP20 package does not have those pins doesn't mean I should ignore them... lets try if this affects power consumption:
	GPIOC_MODER = 0xFFFFFFFF;
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
        //FLASH_ACR &=~(BIT2 | BIT1); // those are 0 after reset, so not nescesairy 

        // Turn on FLASH prefetch buffer
        FLASH_ACR |= BIT4;
 
       // set PLL multiplier to 12 (Because default PLLSRC is HSI/2 = 8/2=4 MHz, and 4*12=48)
        // RCC_CFGR &= ~(BIT21 | BIT20 | BIT19 | BIT18); // this is not nescesairy as those are allready 0 after reset.
        RCC_CFGR |= (BIT21 | BIT19 );

        // Need to limit ADC clock to below 14MHz so will change ADC prescaler to 4
        ADC_CFGR2|=(BIT31); // Clock ADC with PCLOCK/4 =48Mhz/4=12Mhz =OK :)


        // and turn the PLL back on again
        RCC_CR |= BIT24;
        // set PLL as system clock source
        RCC_CFGR |= BIT1;

        RCC_CFGR3 |= (BIT4) ; // select system clock as I2C clock.
        // enable pheripheral clock to timer3 (BIT1), TIM14 (BIT8), and I2C1 (BIT21).
        RCC_APB1ENR |= (BIT1 | BIT8 | BIT21);

        RCC_APB2ENR |= BIT9; // enable clock to adc
}


void setup_adc(){
	// Do ADC calibration here instead of "hidden" in main. That way it also gets recallibrated after waking up from standbye mode (It looses cal in standbye mode.)
	// calibration has to be done with ADEN cleared / 0, so before enabling adc. Then wait untill adcal gets 0 again before enabling adc.
	
	//Before enabling ADC, let it calibrate itself by settin ADCAL (And waiting 'till it is cleared again before enabling ADC)
	ADC_CR |= (BIT31); // set adcal
  	while (ADC_CR & (BIT31)); // Wait for ADCAL to be zero again before anbling ADC

	ADC_CR |= (BIT0); // Set ADEN / enable power to adc BEFORE making settings! (But calibrate before this.)

        while (!(ADC_ISR&BIT0));// check ADCRDY (In ADC_ISR, bit0) to see if ADC is ready for further settings/starting a coversion

        // make rest of settings before starting conversion:
        ADC_CHSELR = (BIT3 | BIT2 | BIT1); // Ch3 = PA3, on pin9 CH2 = PA2 pin 8, CH1 =PA1 pin 7. (Set up channels)
        // It will scan all these channels, but it has only 1 data register for the result. So it will scan them (Low-High is default, so CH1,2,3,1,2,3,)
        ADC_CFGR1 |= (BIT12 | BIT13); // BIT12 set it to discard on overrun and overwrite with latest result -- TODO: think about this in light of that desync issue... (Maybe catch the overrun interrupt to handle it and not desync...)
                               // BIT16: DISCEN Discontinues operation (Don't auto scan, wait for trigger to scan next ch, cannot be used when CONT=1)
                               // BIT13: CONT. automatically restart conversion once previous conversion is finished.
        ADC_SMPR |= ( BIT2 | BIT1 | BIT0); // Set sample rate (Default = as fast as it can: 1.5clk, with bit1&2 set 71.5clk, with just bit 1: 13.5clk, all three set: 239.5clck cycles. At 12Mhz that's ~50.1kHz for tick++ and 16.7Khz per ch.) // XXX note: Changed: set bit0,1,2 instead of 1,2,3!!  

        ADC_IER |=(BIT2|BIT3) ; // Enable end of conversion interrupt (Bit2), and EOSEQ (End of Sequence) bit 3.

        /// ***interrupts*** ///
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
}

void adxl_init(){
/*
Datasheet excerpt:
"
Therefore, some experimentation with values for the
DUR, latent, window, and THRESH_TAP registers is required.
In general, a good starting point is to set the DUR register to a
value greater than 0x10 (10 ms), the latent register to a value greater
than 0x10 (20 ms), the window register to a value greater than
0x40 (80 ms), and the THRESH_TAP register to a value greater
than 0x30 (3 g).
"
*/

// Determine treshhold values in actual application (Maybe even at runtime? Meh, no. just calibrate them once. Manually)
 i2c_write_byte(ADXL345_THRESH_TAP, 0x2D); // 62.5mg per increment
 i2c_write_byte(ADXL345_DUR, 0x10);	  // 625us per increment
 i2c_write_byte(ADXL345_LATENT, 0x80);

//TODO: tune these thressholds so it does not wake up during transport but wakes easily when juggling
 i2c_write_byte(ADXL345_THRESH_INACT, 20); //was 20 
 i2c_write_byte(ADXL345_THRESH_ACT, 20);   //was 20

 i2c_write_byte(ADXL345_TIME_INACT, 15); // seconds (30, 5 for test)
 i2c_write_byte(ADXL345_ACT_INACT_CTL, 0xFF); // look for activity/inactivity on all axes, AC coupled
 i2c_write_byte(ADXL345_THRESH_FF, 0x06); // Freefall thresshold. Reccomended between 0x05 and 0x09 (300/600m g)
 i2c_write_byte(ADXL345_TIME_FF, 0x14); //100ms (0x14 - 0x46) 350ms recommended.
 i2c_write_byte(ADXL345_TAP_AXES, 0x07); // detect taps on all axes.
 i2c_write_byte(ADXL345_BW_RATE, 0x1A); // Low power 100Hz (50uA active, lower yet in sleep).
 i2c_write_byte(ADXL345_POWER_CTL,0x08); // 8Hz in sleep, be active now, do not link inactivity/activity, do not autosleep on inactivity
 i2c_write_byte(ADXL345_INT_ENABLE,0x5C);// enable single tap, activity, inactivity & freefall interrupts
 i2c_write_byte(ADXL345_INT_MAP,0x10); // Only activity to INT2 pin
}


void goto_sleep(){

	ADC_IER &=~(BIT2|BIT3) ; //disable adc interrupts: end of conversion interrupt (Bit2), and EOSEQ (End of Sequence) bit 3.

	i2c_read_byte(0x30); // read interrupts (and clear them) from adxl
 	// because if it detects activity now, it's too soon to react too an thus will never be reacted too.
	i2c_write_byte(ADXL345_POWER_CTL,0x0C); // put ADXL to sleep at 8 Hz sample rate.
	//i2c_write_byte(ADXL345_POWER_CTL,0x0D); // put ADXL to sleep at 4 Hz sample rate.
	//i2c_write_byte(ADXL345_POWER_CTL,0x0E); // put ADXL to sleep at 2 Hz sample rate.
	//i2c_write_byte(ADXL345_POWER_CTL,0x0F); // put ADXL to sleep at 1 Hz sample rate.
	// lower sample rates do not save more power, so let's update at 8Hz.


	// Disable pheripherals / power them down.
	//Power down ADC -- Is slightly more complicated then clearing aden:
	if(ADC_CR&BIT2){// if adstart (If there is a ongoing conversion)
		ADC_CR|=BIT4;	// stop it by writing adstp
		while(ADC_CR&BIT4);	// wait till adc stopped
		}
	ADC_CR|=BIT1;	// then write addis to disable adc
	while(ADC_CR&BIT0);	// wait until aden = 0 to indicate adc is disabled

	while(I2C1_ISR&BIT15); // wait until I2C is no longer busy.
	I2C1_CR1 &=~ BIT0; // disable I2C1 module


	// Set uC to wakeup from EXTI (On pin 11/PA5) (So only that interrupt stays enabled for now!)
   	ISER |= (BIT7); // Enable IRQ7: enable EXTI4..15 interupt in NVIC
    	IPR1 |= (14<<24); // set priority for IRQ7 (4*IPRn+IRQn), starting from 0, so for IRQ7 that's IPR1 bits 31 downto 24
    	//Read the relevant part of PM0215. IRC number is the position listed in RM0360 table 11.1.3.
	// Enable exti interupt on PA5:
	EXTI_IMR |= (BIT5); // and enable it in EXTI_IMR
	EXTI_RTSR |= (BIT5); // For rising edge

	DBGMCU_CR = 0x00; //  Disable debug in STOP mode (But as late as possible)

    	GPIOA_BSRR = (BIT0); // SET PA0 (LED off)

	GPIOA_BSRR = (BIT20 | BIT22 | BIT23) ; // RESET PA4,6,7 (Reset is Bitn+16)
	// set pins to analog input (No power consumption in input buffer for floating inputs) or to output.
	GPIOA_MODER = 0xFFFF51FF; // GPIOA5 = digital input (interupt); PA4,6,7 are PWM outputs; rest analog input
	GPIOB_MODER = 0xFFFFFFFF;
	GPIOF_MODER = 0xFFFFFFFF;

	// output to fet's (PA4,6,7) as output driving ground. Also place pulldown between gate and gnd in hw.
	// without either of those, fet gate floating in sleep mode turns fet on, shorting battery through inductor, smoking inductor and/or fet.

	// set MCU to sleep (STOP mode)
	SCR |= (BIT2); //set sleepdeep (Bit2) in system control register
	PWR_CR &= ~(BIT1); //stop mode
	PWR_CR |= BIT0; //set LPDS in pwr_cr (Low power regulator)
	__asm("WFI");// Wait For Interrupt (WFI) / go to sleep

	/*SLEEPZZZzzzzzZZZZZZZZZZZzzzzzzzZZZZZZZZZZZZZzzzzzzzzzzzzzzzzzzZZZZZZZZZZZzzzzzzzzzzzzzzZZZZZZZZZZZZZZZZZZZZZz*/
	// 17 uA for MCU. 9 uA for Vreg, when ADXL powered down (0.1uA). But then it cannot wake the CPU, so with ADXL sleeping 68uA total (Of which 17uA MCU & 9 Vreg)

	initClock(); // NB: after wakeup it runs from HSI, so initClock() again.

	// set pins same as in setup
	GPIOA_MODER = 0x28000000; // reset value.
	setup_pins();

	// Re-enable pheripherals:
	I2C1_CR1 |= BIT0; // enable I2C1 module
	setup_adc(); // re enable / re setup adc, and its interrupts

	i2c_write_byte(ADXL345_POWER_CTL,0x00); // wake ADXL -> standbye
 	i2c_write_byte(ADXL345_POWER_CTL,0x08); // standbye -> measure (For lower noise)
 	i2c_read_byte(0x30); // read interrupts (and clear them) from adxl, so MCU does not get sent back to sleep again by inactivity.

}

void AllesUit(){
	setpoints[0]=0;
	setpoints[1]=0;
	setpoints[2]=0;
}

void blink (int num){
 GPIOA_BSRR = (BIT0); // SET PA0 (led UIT)
 delay(800000);

while(num--){
 GPIOA_BSRR = (BIT16); // CLEAR PA0 (LED aan)
 delay(400000);
  GPIOA_BSRR = (BIT0); // SET PA0 (led UIT)
 delay(800000);
 }

}

void rainbow(){
	for(int i=0;i<SETPOINT;i++){
		setpoints[2]=i;
		setpoints[0]=SETPOINT-i;
		delay(2000);
	}
	for(int i=0;i<SETPOINT;i++){
		setpoints[1]=i;
		setpoints[2]=SETPOINT-i;
		delay(2000);
	}
	for(int i=0;i<SETPOINT;i++){
		setpoints[1]=SETPOINT-i;
		setpoints[0]=i;
		delay(2000);
	}
}

int main()
{
	initClock();

	blink (4);

	setup_pins();
	
	//Set up I2C:
	GPIOA_OSPEEDR |= 0x0FC00000; // High speed IO for I2C (?)
	I2C1_TIMINGR = 0x50330309; //0x50330309; //calculated from tables in datasheet.
	// 0x00B01A4B; Wrong: /*from datasheet code example: set up timings for fast Mode @400kHz with I2CCLK = 48MHz, rise time = 140ns, fall time = 40ns */
	I2C1_CR1 |= BIT0; // enable I2C1 module

	//set up timer 3 for PWM
	TIM3_PSC = 0; // prescaler. (48Mhz/psc+1=tim3clock)
    TIM3_ARR = 2048;  // 16 bit timer, AutoReloadRegister (frequency) (48E6/((TIM3_PSC+1)*TIM3_ARR)
    
    TIM3_CCMR1 |= (BIT3 | BIT5 | BIT6 | BIT11 | BIT14 | BIT13) ;      // PWM mode (per output bit 4:6). Set OC1PE (bit5), OC2PE (bit11) preload enable, PWM mode for ch2 (BIT 14,13)
    TIM3_CCER |= (BIT0 | BIT4) ; // CC1P to set polarity of output (0=active high), CC1E (bit 0) to enable output on ch1, bit4 for ch2.
    TIM3_CR1 |= BIT7 ;        // Control register. Set ARPE (bit7). And CEN I suppose (Counter enable, bit 0)
    TIM3_EGR |= BIT0 ; // set UG to generate update event so registers are read to the timer
    TIM3_CR1 |= BIT0 ; // start after updating registers!

        // Set AF(AF1) en MODER to select PWM output on pins
	GPIOA_AFRL |= (BIT24 |BIT28 | BIT18); ; // Bit27:24 for AFR6 / PA6, that should get set to AF1 (0001) for TIM3CH1, BIT28 is idem for PA7/TIM3ch2. And bit 18 for  AF4 on PA4: TIM14CH1 (PWM)
	GPIOA_AFRH |= (BIT6 | BIT10); // AF4 for PA9 and PA10 (I2C SCL resp SDA)

    //set up timer 14 for PWM
    TIM14_PSC = 0; // prescaler. (48Mhz/psc+1=tim14clock)
    TIM14_ARR = 2048;  // 16 bit timer, AutoReloadRegister (frequency) (48E6/((TIM14_PSC+1)*TIM14_ARR)
    TIM14_CCMR1 |= (BIT3 | BIT5 | BIT6 ) ;      // PWM mode (per output bit 4:6). Set OC1PE (bit5) preload enable.
    TIM14_CCER |= (BIT0) ; //  CC1E (bit 0) to enable output on ch1.
    TIM14_CR1 |= BIT7 ;        // Control register. Set ARPE (bit7). And CEN I suppose (Counter enable, bit 0)
    TIM14_EGR |= BIT0 ; // set UG to generate update event so registers are read to the timer
    TIM14_CR1 |= BIT0 ; // start after updating registers!

    // power up and set up adc:
    setup_adc();

	//setup array of nice collors
	int colors[LEN_COLOR];
	for(int i = 0;i<LEN_COLOR;i++){
	colors[i]=colorset_percentage[i]*SETPOINT/100;
	}

    blink(2);

	adxl_init(); // power up and setup adxl345

	blink(2);

	//XXX remove this, is just for testing sleep mode (On real HW this time, so with ADXL)
	//goto_sleep();	
	//XXX (on the other hand, sleep after poweron means those bright LED's don't blind me while closing the enclosure... But it interferes with debugging.)	

	while(1){
		int x,y,z, intjes, buffer[6];
		static int cc=0, tap;
		static unsigned int prevtaptick, prevfftick; // timestamps for tap and freefall
		static bool Juggle=false, Catch=false, Flying=false; // Juggle in progress? Just catched?
		enum modes{direct, catchchange, freefall, TimeSinceLastFall, colorwheel, anothercolorwheel} mode;

		intjes = i2c_read_byte(0x30); // read adxl interrupt flags (to sense taps/freefall etc.)
		// reading resets them, so only read once a cycle

		if((intjes&BIT3)&&!(intjes&BIT4)){ // inactivity.
			for(int i = 0;i<3;i++) rainbow(); // Show rainbow fade and after that, go to sleep.
			goto_sleep();
		}// activity will wake it up, but that's hardware (INT2 Wired to EXTI_PA5)



		// tick updates at approx. 55 kHz.
		if(intjes&BIT2){ // detect freefall to keep time since last freefal to prevent modeswitch during juggle
			prevfftick=tick;
			Juggle = true;
			Flying=true;
		}else{

			if( tick > (unsigned int)(prevfftick+550000) ){ // If a freefall is about 10 seconds ago (55Khz * 10s = 550000) (TODO: check if this now is correct)
			Juggle = false;
			}

			if( tick > (unsigned int)(prevfftick+7000) ){ // If a freefall just ended, assume ball is catched
			//(increase wait time when falsely assumed, decrease when lagging too much)
			// (Uses tick to prevent false catch detection while still falling)
				if(Flying){ // Only detect catches when previously falling (flying). Otherwise lying still counts as a catch...
					Flying = false;
					Catch = true;
				}
			}

		}


		// TODO: see if tick-timing now is correct

		// switch mode triple tap
		if((intjes&BIT6)&&(!Juggle)){ // on tap but not while juggling
 			prevtaptick=tick;
 			tap++;
		}

		if( tick > (unsigned int)(prevtaptick+50000) ){
		// if( (tick-prevtick)> 100000 ), "modulo max_int" to handle overflows
		// TODO: check if tripple tap still works now tick ticks at the correct (slower) rate
			tap=0;
		}
		else if(tap>2){ // TRIPLE tap. 3 and up > 2 :)
			tap=0; // reset counter
			mode++; // resets due to default case, so no fuss here.
			AllesUit();
			blink(mode);
		}

		switch(mode){
		case direct: // colour change based on orientation to gravity / test mode

			i2c_read_n_bytes(0x32, 6, buffer); // read xyz in one go
			x=buffer[0]|(buffer[1]<<8);
			y=buffer[2]|(buffer[3]<<8);
			z=buffer[4]|(buffer[5]<<8);

			// because it is 2's complement, if bit 9 is set then bits 31 to 9 should also be set (To convert from 10 bit signed int to 32 bit signed int)
			if(x&1<<9) x|=0xFFFFFC00;
			if(y&1<<9) y|=0xFFFFFC00;
			if(z&1<<9) z|=0xFFFFFC00;

			//scale XYZ to SETPOINT as max
			float g,r,b;
			g=x*SETPOINT/(1<<8); // adxl is 10 bit, signed.
			r=y*SETPOINT/(1<<8);
			b=z*SETPOINT/(1<<8);

			if(g>SETPOINT) g=SETPOINT; // crowbar (force safe value)
			if(r>SETPOINT) r=SETPOINT; // crowbar (force safe value)
			if(b>SETPOINT) b=SETPOINT; // crowbar (force safe value)
			if(g<0) g=0; // crowbar (force safe value / discard if below zero)
			if(r<0) r=0; // crowbar (force safe value / discard if below zero)
			if(b<0) b=0; // crowbar (force safe value / discard if below zero)


			setpoints[1]=r;
			setpoints[0]=g;
			setpoints[2]=b;

			break;

		case catchchange:
		// clourchange on catch / juggle modue, End-Of-FreeFall based
			
			setpoints[0]=colors[1+cc]; // set colors
			setpoints[1]=colors[0+cc];
			setpoints[2]=colors[2+cc];
			
			if(Catch){ // on catch:
			//Change colour to the next one from the predefined list for catch
				if ((3+cc)>=LEN_COLOR) cc=0; else cc+=3;
				Catch=false;
			}

		/*
		// hmmz, lets test catch detection in a simpler way first:
			if(Catch){
				setpoints[1]=SETPOINT;
				Catch=false;
				}
				else AllesUit();
		*/

	     /*
		 // clourchange on catch / juggle modue, tap based, less accurate
			if(intjes&BIT6){ // on tap:
			//Change colour to the next one from the predefined list for tap
				if ((3+cc)>=LEN_COLOR) cc=0; else cc+=3;
				setpoints[0]=colors[1+cc];
				setpoints[1]=colors[0+cc];
				setpoints[2]=colors[2+cc];
			}

		*/

			break;

		case freefall: // red while freefalling / test mode
			if(intjes&BIT2){ // on freefall:
			//TODO: Change colour to the next one from the 2nd predefined list for freefall (Uhm, nope as there are multiple interrupts per fall.)
			// for now, be red
			setpoints[1]=SETPOINT;
			delay(100000);
			}else AllesUit();
			break;

		case TimeSinceLastFall: // test mode: blue while modechange is prevented.
			if(Juggle) setpoints[2]=SETPOINT; else AllesUit(); // remain blue for .. ticks after freefall
			break;


		// a case that keeps changing colour while juggle is true. (juggleball misbehaved as such while testing colorchange on catch and it kind of seems like a nice idea too)
		case colorwheel:
			if(Juggle){ 
					if ((3+cc)>=LEN_COLOR) cc=0; else cc+=3;
				setpoints[0]=colors[1+cc];
				setpoints[1]=colors[0+cc];
				setpoints[2]=colors[2+cc];
				delay(300000);
			}
			break;
			
		case anothercolorwheel:
			if(Juggle) rainbow();
			break;

		default:
			mode = 0;
			break;
		}
	}
	return 0;
}



void ADC_Handler(){
	tick++;
        //GPIOA_BSRR = (BIT0); // SET PA0 (To time handler) (55.5KHz now. 3.4 / 3.6us active, 14 us not)

        static int ch=0; // keep between invocations
        static int pwm[3];

        if(ADC_ISR&(BIT2)){ // Check EOC (Could check EOSEQ when the sequence is only 1 conversion long)
                adcresult=ADC_DR; // read adc result for debugger/global use.

                pwm[ch] += (setpoints[ch]-adcresult); // integrating comparator.
                // This gets out of sync sometimes, why? TODO
                // (Feedback from one ch controlling another. Not good.)
                // Especially debugging throws it out of sync but sometimes after reset it is another ch as well.
            	// (Another ADC interrupt before ch is incremented, OK, for debug I understand how that can happen, but why it hapens after reset?)
		// possibly overrun? (not reading result because stoppend in debug?)

                if (pwm[ch]<0) pwm[ch]= 0; else if (pwm[ch]>1024) pwm[ch]=1024; //max 50% D.
                TIM3_CCR1 = pwm[2];
                TIM3_CCR2 = pwm[1];
              	TIM14_CCR1 = pwm[0];
                ch++;
                }

        if(ADC_ISR&(BIT3)){ // EOSEQ 
        	ch=0; // resync; (Just in case, but turned out usefull)
        	ADC_ISR&=(BIT3); // reset flag
        }

        //GPIOA_BSRR =(BIT16);//  clear PA0 after running this handler. (To time handler and check sample rate) // TODO: might it be inverted? CLEAR=LED ON and SET=LED OFF?
}

void EXTI_Handler(void){
// empty for now, could do wake up stuff here later
EXTI_PR |=(BIT5); // clear the flag.
}

