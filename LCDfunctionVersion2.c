// LCD Printing File by Giorgio Gunawan  [OU resource ECE 470/570 - modified to throw away assembly part]

#include <hidef.h>            /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */

// FUNC DECLARATION ======================================================================================
void cmd2LCD (char cmd);
void openLCD(void);
void putcLCD(char cx);
void putsLCD(char *ptr); 
void delayby10us(int k);

// In Assembly, we use PTK. In C, we use PORTK
// PORTK: bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
//                  DB7  DB6  DB5  DB4  EN   RS

// THIS ONE PUTS THE CHAR IN NEXT LINE AND FIRST COLUMN ===================================================
void cmd2LCD (char cmd) {  // cmd: two nibbles. First the higher is sent over DB7-DB4 pins. This is actually DB7-DB4
                          //   Then, the lower nibble is sent over DB7-DB4 pins. This is actually DB3-DB0
    char hnibble, lnibble;
    // note that here, RS is always 0
    
    PORTK = 0x00; // EN, RS=0    

    // sending higher nibble        
    hnibble = cmd &0xF0;
    PORTK = 0x02; // EN = 1    
    hnibble >>=2; // shift two bits to the right to align with PORTK positions
    PORTK = hnibble|0x02;
    asm("nop");  // Does no work except stalls the program for very little time --> 41.67 ns (1 cycle)
    asm("nop");
    asm("nop");
    PORTK = 0x00; // EN,RS=0    
    
    // Sending lower nibble:
    lnibble = cmd &0x0F;
    PORTK = 0x02; // EN = 1
    lnibble <<= 2; // shift two bits to the left to align with PORTK positions
    PORTK = lnibble|0x02;
    asm("nop");    // Does no work except stalls the program for very little time --> 41.67 ns (1 cycle)
    asm("nop");
    asm("nop");    
    PORTK = 0x00; // EN, RS = 0
    
    // Wait 50 us for order to complete (enough time for most IR instructions)
    //asm_mydelay10us(5);
    delayby10us(5);
}

void openLCD(void) {
    DDRK = 0xFF; // PortK configured as outputs
    //asm_mydelay1ms(100); // wait for 100 ms
    delayby10us(10000);   // wait for 100 ms
    cmd2LCD(0x28); // set 4-bit data, 2-line display, 5x8 font
    cmd2LCD(0x0F); // turn on display, cursor, blinking
    cmd2LCD(0x06); // move cursor right
    cmd2LCD(0x80);  // set DDRAM address. DDRAM data are ent after this setting. address=0
    cmd2LCD(0x01); // clear screen, move cursor to home
    //asm_mydelay1ms(2); // wait for 2 ms. The 'clear display' instruction requires this
    delayby10us(200); // wait for 2 ms;
    
}

// PUT CHAR ON SCREEN =============================================================================
void putcLCD(char cx) {
    char hnibble, lnibble;
    // temp = cx;
    // note that here, RS is always 1
    
    PORTK = 0x01; // RS=1, EN=0
    
    // sending higher nibble
    hnibble = cx&0xF0;
    PORTK = 0x03; // RS=1, EN=1
    hnibble >>= 2; // shift two bits to the right to align with PORTK positions
    PORTK = hnibble|0x03;
    asm("nop");  // Does no work except stalls the program for very little time --> 41.67 ns (1 cycle)
    asm("nop");  
    asm("nop");    
    PORTK = 0x01; // RS=1, E=0
    
    // sending lower nibble
    lnibble = cx & 0x0F;
    PORTK = 0x03; // RS=1, E=1
    lnibble <<= 2;  // shift two bits to the left to align with PORTK positions        
    PORTK = lnibble|0x03;
    asm("nop");     // Does no work except stalls the program for very little time --> 41.67 ns (1 cycle)
    asm("nop");
    asm("nop");    
    PORTK = 0x01;// RS=1, E=0
    
    // Wait 50 us for order to complete (enough time for most IR instructions)    
    //asm_mydelay10us(5); // wait for 50  us
    delayby10us(5);
}

// CONCATENATE PUT CHARS TO MAKE PUT STRIG. MORE USED! ==================================================
void putsLCD (char *ptr) {
    while (*ptr) {
        putcLCD(*ptr);
        ptr++;
    }
}

// DELAY BY 10 MICRO SECOND *( INT K) ==================================================================
void delayby10us(int k)
{
      int ix;                               // Index ix for looping
      TSCR1=0x90;                           // Enable timer counter and enable fast clear for TOF and CH5
      TSCR2=0x00;                           // Timer Pre-scale factor = 1. E-clock=24MHz. Period: 1/24 us (microsecond)
      TIOS = 0x01;                          // Channel 0: Output Compare
      TC0 = TCNT + 240;                     // + 240 means 240 timer cycle where 1 period is 1/24 us so 10 us (microseconds)
      for (ix=0;ix<k;ix++) {                // FOR loop to go through ix (for twice in this specific program)
      
      
        // Wait until TFLG1(0) = 1, 
        // while (TFLG1&0x01 != 0x01); //the same, 
        // but always FALSE (because of the Compiler)
        // TLFG1_CnF_MASK is NOT the same as TLFG1_CnF. 
        // For n=0, they are the same by chance.
        while (!(TFLG1&TFLG1_C0F_MASK));    
        
        TC0 = TC0 + 240; // when we reach 'k', in this program specific is 2,this statement will just run on its own 
      }
      TIOS = 0x00; // Then disable as we are done with using the output compare channel
}

