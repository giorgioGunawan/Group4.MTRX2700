/* mixasm.h. The name of the ASM file must be mixasm.asm*/
// By gio, changed from asm file - mixasm so it should still work
#ifndef _LCD_H_
#define _LCD_H_

void cmd2LCD (char cmd);
void openLCD(void);
void putcLCD(char cx);
void putsLCD(char *ptr); 
void delayby10us(int k);

#endif /* _LCD_H_ */