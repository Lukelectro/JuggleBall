/*
NOTES, IDEAS, CURRENT STATE, TITLE, LICENSE, ETC. SHOULD GO HERE.

Use tap, freefal, activity and inactivity interrupts from ADXL345.
Inactivity: Fade trough rainbow colours a few times (say, 10s) then go to sleep.
Activity: Wake up / Folow "juggle" protocol
Tap: Part of juggle protocol: Change colour (Jump through a set of predefined pretty coulours)
Freefall: change colour.
That should make for a nice lightshow when juggling: ball changing colour at catch and in fall.

Idea: maybe change mode on doubletap, with mode one: change colour on tap, mode 2: change on tap and fall, mode 3: raw acellerodata to LED's?

TODO: Smooth modeswitching needs more work; code is getting messy so clean up; Hardware smoked out (FET stayed on somehow, but was during debug) -> repair.

*/

#include "stm32f030xx.h" // the modified Frank Duignan header file. (I started from his "Blinky" example). 
#include "adxl345.h" // for the register names

//MAX setpoints 
//(2^12/3v3 * 1.8*5/11.8) = 947 --- 5V uit, 3v3 ref. 12 bit adc 10k/1k8 div.
//(2^12/3v3 * 0.05 * 15)= 930 -- 50mA max uit, 3v3 ref, 12Bit adc, 15R sense resistor. (60mA is AMR for the LED's I use)
//(2^12/3v3 * 0.02 * 15)= 372 -- 20mA max uit, 3v3 ref, 12Bit adc, 15R sense resistor.
#define SETPOINT1 372 
#define SETPOINT2 372 
#define SETPOINT3 372 
#define SETPOINT 372

#define I2C_ADR 0xA6 // adxl 345 alternate adres/sdo low (0x53+rw dus 0xA6/7)

//TODO: a few (const) structs or array or something predefining "pretty" LED coulours.
// maybe it's a good idea to scale this to max setpoint.
// certainly it's a bad idea to hardcode values
// Does cortex M0 have a hw divider? Float multiplier? Does my compiler use them?
// Meh, premature optimalisation. Besides, I could just use floats, precalculate the arrays, and then use the calculated onces :)
#define LEN_COLOR 24
const int colorset_percentage[LEN_COLOR] = //r,g,b
{
 100,0,0,    //red
 100,30,100, //white-ish bluegreen
 100,0,30,   //magenta/pink purple 
 0,100,0,    //green
 100,40,0,   //warmyellow
 0,0,100,     //blue
 100,60,0,    //coldyellow
 100,100,100, // white
 }; 
// How this works:  const array is filled with percentages of SETPOINT (Max). 
// then setpoints are calculated as percentage of max and stored in a variable aray
// as start of main, further down the variable arrai is used
// of course this calculation could have been done at compile time as the values never change...



//Because I want to change color differently on tap and freefall, I'll offset one of them a bit but use the same colorset.
//If that does not work out, I make a 2nd colourset. Could even pick a random color from set on both events, instead of moving in a predetermined patern?

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



volatile int adcresult; // can be read in debugger too.
int setpoints[3]={SETPOINT1,SETPOINT2,SETPOINT3};
unsigned int tick=0; // Gets ++t in adc handler and used for timeouts	


// spiekbriefje: I2C_CR2 |= (NBYTES 23 downto 16)|(SLADR) ;// BIT25=AutoEnd, Bit14 = Manualy generate a STOP, bit13 = generate a START BIT10=R/!W
// Slave adres on 7:1 with bit 0 don't care (For 7 bit adr. slaves such as MPU6050/adxl345)

int i2c_read_byte(int addr) { 
    I2C1_CR2 = (BIT13)|(1<<16)|(I2C_ADR); // Write 1 byte to I2C_ADR and sent start
    I2C1_TXDR = addr;
    while (!(I2C1_ISR & BIT0)); // wait for TX empty before changing CR2 and sending next byte

    I2C1_CR2 = BIT10 | BIT13 | (1<<16) | I2C_ADR | BIT25; // read (BIT10) one byte (1<<16) from I2C_ADR, generate start (BIT13), and generate stop when done (BIT25)
    
    while (!(I2C1_ISR & BIT2)); // wait till data in receive buffer 
    return I2C1_RXDR;
    
}

void i2c_read_n_bytes(int addr, int n, int* buff) { 
    I2C1_CR2 = (BIT13)|(1<<16)|(I2C_ADR); // Write 1 byte to I2C_ADR and sent start
    I2C1_TXDR = addr;
    while (!(I2C1_ISR & BIT0)); // wait for TX empty before changing CR2 and sending next byte

    I2C1_CR2 = BIT10 | BIT13 | ((n&0xFF)<<16) | I2C_ADR | BIT25; // read (BIT10) n byte (<<16) from I2C_ADR, generate start (BIT13), and generate stop when done (BIT25)
    
    
    for(int i=0;i<n;i++){
    	while (!(I2C1_ISR & BIT2)); // wait till data in receive buffer 
    	buff[i]=I2C1_RXDR;
    }
}

void i2c_write_byte(int addr, int data) { 
    I2C1_CR2 = (BIT13 | (2<<16) | I2C_ADR | BIT25);
  
    I2C1_TXDR = addr;
    while (!(I2C1_ISR & BIT0));

    I2C1_TXDR = data;
    while (!(I2C1_ISR & BIT0));
}


void setup_adc(){
	ADC_CR |= (BIT0); // Set ADEN / enable power to adc BEFORE making settings!
        
        while (!(ADC_ISR&BIT0));// check ADCRDY (In ADC_ISR, bit0) to see if ADC is ready for further settings/starting a coversion
        
        // make rest of settings before starting conversion:
        ADC_CHSELR = (BIT3 | BIT2 | BIT1); // Ch3 = PA3, on pin9 CH2 = PA2 pin 8, CH1 =PA1 pin 7. (Set up channels)
        // It will scan all these channels, but it has only 1 data register for the result. So it will scan them (Low-High is default, so CH1,2,3,1,2,3,)
        ADC_CFGR1 |= (BIT12 | BIT13); // BIT12 set it to discard on overrun and overwrite with latest result 
                               // BIT16: DISCEN Discontinues operation (Don't auto scan, wait for trigger to scan next ch, cannot be used when CONT=1)
                               // BIT13: CONT. automatically restart conversion once previous conversion is finished. 
        ADC_SMPR |= ( BIT1 | BIT2 | BIT3); // Set sample rate (Default = as fast as it can: 1.5clk, with bit1&2 set 71.5clk, with just bit 1: 13.5clk, all three set: 239.5clck cycles. At 12Mhz that's ~50.1kHz for tick++ and 16.7Khz per ch.)     
        
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
// TODO: Determine treshhold values in actual application (Maybe even at runtime? Meh, no. just calibrate them once. Manually)
 i2c_write_byte(ADXL345_THRESH_TAP, 50); //TODO: Tune
 i2c_write_byte(ADXL345_DUR, 5);
 i2c_write_byte(ADXL345_THRESH_INACT, 20);
 i2c_write_byte(ADXL345_THRESH_ACT, 20);
 
 i2c_write_byte(ADXL345_TIME_INACT, 10); // 10s
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
	//i2c_write_byte(ADXL345_POWER_CTL,0x00); // Put ADXL in standbye (TODO: Maybe sleep? If it can still wake the MCU then?)
 	i2c_read_byte(0x30); // read interrupts (and clear them) from adxl
 	// because if it detects activity now, it's too soon to react too an thus will never be reacted too.
	
	// Disable interrupts
	ADC_IER &=~(BIT2|BIT3) ; //disable end of conversion interrupt (Bit2), and EOSEQ (End of Sequence) bit 3. 
        
         
         // then stop timer? No, deepsleep should stop all clocks, does the same.
	DBGMCU_CR = 0x00; //  Disable debug in STOP mode
	
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
	// Removing clock from things won't save power, as all clocks STOP in STOP mode.
	// But turning the PLL off might help. However, then I need another system clock first:
	RCC_CFGR &= ~(BIT1|BIT0); // HSI as system clock again
        RCC_CR &= ~BIT24; // disable PLL
       
	
	// Set uC to wakeup from EXTI (On pin 11/PA5) (So only that interrupt stays enabled for now!)
	GPIOA_PUPDR|=(BIT11); // enable pulldown op PA5.
        ISER |= (BIT7); // Enable IRQ7: enable EXTI4..15 interupt in NVIC
        IPR1 |= (14<<24); // set priority for IRQ7 (4*IPRn+IRQn), starting from 0, so for IRQ7 that's IPR1 bits 31 downto 24
        //Read the relevant part of PM0215. IRC number is the position listed in RM0360 table 11.1.3.
	// Enable exti interupt on PA5:
	EXTI_IMR |= (BIT5); // and enable it in EXTI_IMR
	EXTI_RTSR |= (BIT5); // For rising ende
 	
 	//Set output pins to whatever makes them Low Power 
	 TIM3_CCR1 = 0; // set timer outputs 0 TODO:FET/coil smoked on sleep entry. Why does fet stay on?
         TIM3_CCR2 = 0;
         TIM14_CCR1 = 0;
         
         
         //TODO: force output pins of timer to 0 (TIMx_CCMRx)
         TIM14_CCMR1 &= ~(BIT5); // it was once set to PWM mode 1, with OC1M to 110. Those are 6downto4. By clearing bit 5 its set to force low. 
         			// maybe should build in a little data hiding here
         TIM3_CCMR1 &= ~(BIT5|BIT13); // switch TIM3 outputs to inactive in the same way (2 ch, so OCM1 and OCM2)
         
         
         GPIOA_BSRR = (BIT0); // SET PA0 (LED off)
        
	
	// set MCU to sleep (STOP mode)
	SCR |= (BIT2); //set sleepdeep (Bit2) in system control register
	PWR_CR &= ~(BIT1); //Clear PDDS pwr_cr (Low power regulator)
	PWR_CR |= BIT0; //set LPDS in pwr_cr (Low power regulator)
	__asm("WFI");// Wait For Interrupt (WFI) / go to sleep
	
	/*SLEEPZZZzzzzzZZZZZZZZZZZzzzzzzzZZZZZZZZZZZZZzzzzzzzzzzzzzzzzzzZZZZZZZZZZZzzzzzzzzzzzzzzZZZZZZZZZZZZZZZZZZZZZz*/
	
	initClock(); // NB: after wakeup it runs from HSI, so initClock() again. 
	
	TIM14_CCMR1 |= (BIT5); // set TIM14 back to PWM mode
	TIM3_CCMR1 |= (BIT5|BIT13); // idem for TM3
	
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
 delay(200000);
 
while(num--){
 GPIOA_BSRR = (BIT16); // CLEAR PA0 (LED aan)
 delay(100000);
  GPIOA_BSRR = (BIT0); // SET PA0 (led UIT)
 delay(200000); 
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


int main() // TODO: Lots of cleanup!
{
	initClock();

	// enable clock to Porta
	RCC_AHBENR |= BIT17;
	
	GPIOA_MODER |= ( BIT0 | BIT9 | BIT13 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7 | BIT15 | BIT19 | BIT21) ; // make PA0 an output (Pin6, BIT0), PA4 (pin10, BIT9) to AF (TIM14CH1), PA6/pin12 (Bit13) AF (timer), and PA1,2,3/pin7,8,9 analog (BIT2,3,bit4,5,Bit6 and 7, reps), PA7 AF (TIM3_CH2) Bit 15. PA9/10 AF4 (I2C1) 
	
	// Set unused pins to a defined state so floating inputs do not consume power
	GPIOF_MODER |= BIT0|BIT1|BIT2|BIT3; // PF0 and PF1 to Analog Input
	GPIOB_MODER |= BIT2|BIT3; //PB1 to AIN
		
	//Set up I2C:
	GPIOA_OSPEEDR |= 0x0FC00000; // High speed IO for I2C (?)
	I2C1_TIMINGR = 0x50330309; //0x50330309; //calculated from tables in datasheet.
	// 0x00B01A4B; Wrong: /*from datasheet code example: set up timings for fast Mode @400kHz with I2CCLK = 48MHz, rise time = 140ns, fall time = 40ns */
	I2C1_CR1 |= BIT0; // enable I2C1 module
	
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
	GPIOA_AFRH |= (BIT6 | BIT10); // AF4 for PA9 and PA10 (I2C SCL resp SDA)
	
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
        setup_adc();

	int dummy; // XXX
	
	setpoints[0] = 0;
	setpoints[1] = 0;
	setpoints[2] = SETPOINT/2;
	
	//setup array of nice collors
	int colors[LEN_COLOR];
	for(int i = 0;i<LEN_COLOR;i++){
	colors[i]=colorset_percentage[i]*SETPOINT/100;
	}
	
	adxl_init(); // power up and setup adxl345
	delay(900000);
	setpoints[1] = SETPOINT/2; // to see how much time
	setpoints[2]=0;

	blink(4);
	
		
	while(1){	
		int x,y,z, intjes, buffer[6]; 
		static int cc=0, mode = 0, tap;
		static unsigned int prevtick;
		enum mode{Direct=0, ChangeOnTap, freefall};
		
		intjes = i2c_read_byte(0x30); // read adxl interrupt flags (to sense taps/freefall etc.)
		// reading resets them, so only read once a cycle
		
		if((intjes&BIT3)&&!(intjes&BIT4)){ // inactivity.
		
		for(int i = 0;i<3;i++) rainbow(); // Show rainbow fade and after that, go to sleep.
	
		goto_sleep();
		}// activity will wake it up, but that's hardware (INT2 Wired to EXTI_PA5)
		
		
		// switch mode triple tap TODO: refine for reliablillity and should not trigger in normal juggling
		if(intjes&BIT6){ // on tap
 			prevtick=tick;
 			tap++;
		}
		if( (tick-prevtick)> 150000 ){ // tick updates at 50104.384Hz (50kHz), so this is ... seconds. (Should be 1.5s)
			tap=0; 
		}
		else if(tap>3){
			tap=0; // reset counter
			mode++; // resets due to default case, so no fuss here.
			AllesUit();
			blink(mode);
		}
		
		// TODO: and enmum for clarity
	
		switch(mode){
		case 0:
		
			// TODO: only read when new data is available
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
		
		case 1:
		
			if(intjes&BIT6){ // on tap:
			//Change colour to the next one from the predefined list for tap
				if ((2+cc)>LEN_COLOR) cc=0; else cc+=3;
				setpoints[0]=colors[1+cc];
				setpoints[1]=colors[0+cc];
				setpoints[2]=colors[2+cc];
			}
			break;
			// TODO: Debounce taps
			
		case 2:
		
			if(intjes&BIT2){ // on freefall:
			//TODO: Change colour to the next one from the 2nd predefined list for freefall
			// for now, be red
			setpoints[1]=SETPOINT;
			delay(100000);
			}else AllesUit();
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
        //GPIOA_BSRR = (BIT0); // SET PA0 (To time handler)
        
        static int ch=0; // keep between invocations
        static int pwm[3];
        
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
                ch++;
                }
        
        if(ADC_ISR&(BIT3)){ // EOSEQ is used to resync. 
        	ch=0; 
        	ADC_ISR&=(BIT3); // reset flag
        }
        
        //GPIOA_BSRR =(BIT16);//  clear PA0 after running this handler. (To time handler and check sample rate)
	
}

void EXTI_Handler(void){
// empty for now, could do wake up stuff here later TODO
EXTI_PR |=(BIT5); // clear the flag. 
}

