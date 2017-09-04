#ifndef __CONFIG
#define __CONFIG


//MAX setpoints 
//(2^12/3v3 * 1.8*5/11.8) = 947 --- 5V uit, 3v3 ref. 12 bit adc 10k/1k8 div.
//(2^12/3v3 * 0.05 * 15)= 930 -- 50mA max uit, 3v3 ref, 12Bit adc, 15R sense resistor. (60mA is AMR for the LED's I use)
//(2^12/3v3 * 0.02 * 15)= 372 -- 20mA max uit, 3v3 ref, 12Bit adc, 15R sense resistor.
#define SETPOINT1 372 
#define SETPOINT2 372 
#define SETPOINT3 372 
#define SETPOINT 372

//TODO: a few (const) structs or array or something predefining "pretty" LED coulours.
// maybe it's a good idea to scale this to max setpoint.
// certainly it's a bad idea to hardcode values
// Does cortex M0 have a hw divider? Float multiplier? Does my compiler use them?
// Meh, premature optimalisation. Besides, I could just use floats, precalculate the arrays, and then use the calculated onces :)
#define LEN_COLOR 33
const int colorset_percentage[LEN_COLOR] = //r,g,b
{
 100,0,0,    //red
 100,30,100, //white-ish bluegreen
 100,0,30,   //magenta/pink purple
 100,60,0,   //yellow
 0,0,100,     //blue
 0,100,0,    //green
 30,0,100,		// untested... Blue Purple-ish?
 100,100,100, // white 
 0,100,100,    //bluegreen
 100,0,100,   // purple
 0,100,60	// untested... greenish Blueish?
 }; 
// How this works:  const array is filled with percentages of SETPOINT (Max). 
// then setpoints are calculated as percentage of max and stored in a variable aray
// as start of main, further down the variable arrai is used
// of course this calculation could have been done at compile time as the values never change...

//Because I want to change color differently on catch and (Maybe?) freefall (or other events), I'll offset one of them a bit but use the same colorset.
//If that does not work out, I make a 2nd colourset. Could even pick a random color from set on events, instead of moving in a predetermined patern?


// TODO: more colours, maybe figure out how not to use RAM, then again, plenty of ram and cpu time avaialable...
#endif

