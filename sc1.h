#ifndef _sci1_H_
#define _sci1_H_

void SCI1_Init(unsigned short baudRate);
char SCI1_InChar(void);
void SCI1_OutChar(char data);
char SCI1_InStatus(void);
char SCI1_OutStatus(void);
void SCI1_OutString(char *pt);
unsigned short SCI1_InUDec(void);
unsigned long SCI1_InULDec(void);
signed int SCI1_InSDec(void);
signed long SCI1_InSLDec(void);
void SCI1_OutUDec(unsigned short n);
unsigned short SCI1_InUHex(void);
void SCI1_OutUHex(unsigned short number);
void SCI1_InString(char *string, unsigned short max);



#endif