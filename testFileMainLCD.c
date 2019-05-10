// THIS IS A TEST FILE, guideline by gio. This file is not to be used.
/* Basically, when outputting the serial outputs to the lcd, what you need is:
         #include "LCD.h"      --> as a header file
    And then, for the functions, what we need is:
         openLCD():                --> void input, void output. Just to open LCD
         putsLCD(message1); --> message 1 is outputted
         cmd2LCD(0xC0);       --> put to next line, and then to the FIRST column
         putsLCD(message2); --> message 2 is outputted
    Messages are outputted in format of string.
*/

#include <hidef.h>            /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "LCD.h"               /* LCD enabling funcs*/

void main(void) {
	char *msg1 = "WOOHOOO ";
	char *msg2 = "LCD works!!!!!!!!";
	
	// To get to next row next column, this is the file functionto use
	int nextRowNextCol = 0xC0;
	
	// Messages are placed line by line indepedently
	// There might be other ways to control the LCD so that for example we can shift data on LCD:
	//  we can displace the same message, or we can do something with the cursor shift and data to do so.
	openLCD();
	putsLCD(msg1);
	cmd2LCD(nextRowNextCol); // move cursor to 2nd row, 1st column
	putsLCD(msg2);
	while(1);  
}
