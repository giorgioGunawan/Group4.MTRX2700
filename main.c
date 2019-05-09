// Demonstration functions for IIC to read inertial sensor values
//
// The program will send read values to the serial port.
// you need to connect the serial port to a terminal to verify operation
// port speed 9600 bauds
//
// Eduardo Nebot  29 January 2016
// Sensors implemented:
// Gryro L2g2400D  (3 axis )
// Accelerometer  ADXL345
// Magnetometer HM5883
// Laser Lidarlight:  the driver is working with version 1 but not with version 2
// Version 2 work in progress: 
// Laser Interface will be done by measuring the pulse PWM pulse width
// Last version: 29/1/16
// 
//  This version installed interrupts with the simple model
//  The user should create a project selecting small memory model
//  and minimal startup code. This form will not intiliase any variables !
//  Your program will need to intialize the variables !
//
// Resources used: This program is using Timer 6 for the sampling time
// 
// the iic drivers are using Timer 7. ( You cannot use this timer in your program)
// Do not change the prescaler. If you do you need to change some code in iic.

// FILE INCLUDE, DECLARATIONS ----------------------------------------------------------------------------------------------------------
#include <hidef.h>        // common defines and macros */
#include "derivative.h"   // derivative-specific definitions */
#include "iic.h"          // Header File - iic.c function declaration (not a built in func)
#include "pll.h"	  // Header File - pll.c function declaration (not a built in function)
#include "sci1.h"         // Header File - sci1.c function declaration not a built in
#include "l3g4200.h"      // register's definitions    ( not used by ed )

volatile uint8_t alarmSignaled1 = 0; /* Flag set when alarm 1 signaled */
volatile uint16_t currentTime1 = 0; /* variables private to timeout routines */
uint16_t alarmTime1 = 0;
volatile uint8_t alarmSet1 = 0;

// void INTERRUPT timer6(void);
void setAlarm1(uint16_t msDelay1);
void delay1(uint16_t msDelay1);
void Init_TC6 (void);

// CONSTANTS ---------------------------------------------------------------------------------------------------------------------------
#define laser_wr  0xc4
#define laser_rd  0xc5

#define gyro_wr 0xD2
#define gyro_rd 0xD3

#define accel_wr 0xA6    //
#define accel_rd 0xA7    // 
#define ADXL345_TO_READ 6
 
#define ADXL345_POWER_CTL 0x2D
#define ADXL345_DATAX0 0x32
#define ADXL345_DATA_FORMAT 0x31
 
#define ADXL345_OFSX 0x1E
#define ADXL345_OFSY 0x1F
#define ADXL345_OFSZ 0x20
 
#define ALPHA 0.5

#define magnet_wr  0x3C
#define magnet_rd  0x3D

#define HM5883_MODE_REG 0x02
#define HM5883_DATAX0 0x03

#define BUFF_SIZE	100

// CHARACTER INITIATION -------------------------------------------------------------------------------------------------------------
char buff[BUFF_SIZE];
int gxraw[BUFF_SIZE];
int gyraw[BUFF_SIZE],gzraw[BUFF_SIZE];	

int axraw[BUFF_SIZE];
int ayraw[BUFF_SIZE],azraw[BUFF_SIZE];	

int mxraw[BUFF_SIZE];
int myraw[BUFF_SIZE],mzraw[BUFF_SIZE];	

int k;

// FUNCTION DECLARATION ---------------------------------------------------------------------------------------------------------------
void adxl345_getrawdata(int *axraw, int *ayraw, int *azraw);
void accel_init(void);
void accel_test(void);

void hm5883_getrawdata(int *mxraw, int *myraw, int *mzraw);
void magnet_init(void);
void magnet_test(void);

void l3g4200d_getrawdata(int *gxraw, int *gyraw, int *gzraw);
void gyro_init(void);
void gyro_test(void);

void laser_init (void);
void laser_data(uint16_t *Dist) ;

unsigned char LidarWriteAddr = 0xc4;
uint8_t LidarReadAddr = 0xc5;
uint8_t Lidar2ByteRead = 0x8f;
uint16_t Dist;

// MAIN FUNCTION ----------------------------------------------------------------------------------------------------------------------
void main(void) {
  /* put your own code here */
 	
 char  myString[20];
 char character;
  
 uint8_t D2W, D3R, WHO, who;
 int  res1, res2,  res3, *p;
 float acc;
 
 // The next 4 lines just to make sure all is working
 // are not needed for final prograM
 DDRB= 0xFF;   /* Port B output */
 DDRJ= 0xFF;   // Port J to Output
 PTJ = 0x00;   // enable LEDs
 PORTB=0x55;     // debuging info
 
// Make sure we are running at 24 Mhz,no input, no output.
// File is a general file and will say 48MHz but its modified to run on 24MHz
 PLL_Init();  
 
EnableInterrupts;

 // This program will send the gyro, accelerometer adn magnetometer data
 // to a serial port. Set it to 9600 bauds 

 // Initialises serial port with input = baud rate in bits/sec; output = none!
 SCI1_Init(BAUD_9600);   // capped at 9600, if PLL inactive (4 MHz bus) 
 SCI1_OutString("Program Starting ");      // should display this
 
 // If interrupt is triggered, this function will set timer on channel 6 with a prescaler 1 on 24MHz
 Init_TC6();

 // Initialise I2C
 // IBFD --> -0x23 --> Meet timing requirements for start and stop. The specific address gives 100kHz operation
 iicinit();
	
 // gyro_test(); // make sure a l3g is connected
 
 // Initialise all 3 sensors
 gyro_init();     // l3g4200 setup
 accel_init();
 magnet_init();

 // Start forever loop waiting for interrupt
 while(1) {
	 
 // Delay = 3000 ms for readability and time for change	 
 delay1(3000); 
	 
 // GYROSCOPE L3G4200d---------------------------------------------------------------------------------------------------------------- 
  
 // Obtain Raw Data fotr GYRO
 l3g4200d_getrawdata( &gxraw, &gyraw, &gzraw) ;        
 
 // X axis
 SCI1_OutString("Gyro Gx:");
 SCI1_OutUDec((unsigned short) gxraw[0]); 
 SCI1_OutString("\r\n"); 
 // Y axis	 
 SCI1_OutString(" Gy:"); 
 SCI1_OutUDec((unsigned short) gyraw[0]) ;
 SCI1_OutString("\r\n"); 
 // Z axis	 
 SCI1_OutString(" Gz:"); 
 SCI1_OutUDec((unsigned short) gzraw[0]) ;       
 SCI1_OutString("\r\n");
 // ACCELEROMETER ADCL345 --------------------------------------------------------------------------------------------------------------
 
 // Obtain raw data
 adxl345_getrawdata( &axraw, &ayraw, &azraw) ;  

 // X axis
 SCI1_OutString("Accel Ax:");
 SCI1_OutUDec((unsigned short) axraw[0]); 
 SCI1_OutString("\r\n"); 
 // Y axis 
 SCI1_OutString(" Ay:"); 
 SCI1_OutUDec((unsigned short) ayraw[0]) ;
 SCI1_OutString("\r\n"); 
 // Z axis	 
 SCI1_OutString(" Az:"); 
 SCI1_OutUDec((unsigned short) azraw[0]) ;       
 SCI1_OutString("\r\n"); 
 // MAGNETOMETER HM5883 ----------------------------------------------------------------------------------------------------------------
 
 // Obtain raw data
 hm5883_getrawdata(&mxraw, &myraw, &mzraw);

 // X axis 
 SCI1_OutString("Magn Mx:"); 
 SCI1_OutUDec((unsigned short) mxraw[0]); 
 SCI1_OutString("\r\n");
 // Y axis
 SCI1_OutString(" My:"); 
 SCI1_OutUDec((unsigned short) myraw[0]) ;
 SCI1_OutString("\r\n");
 // Z axis
 SCI1_OutString(" Mz:"); 
 SCI1_OutUDec((unsigned short) mzraw[0]) ;       
 SCI1_OutString("\r\n\r\n\r\n");
 }
 
 
  /*    Not using the laser IIC, not compatible with this lasr
  laser_data (&Dist);
  SCI1_OutString("\r\n laser:");
  SCI1_OutUDec((unsigned short) Dist) ;
  SCI1_OutString("\n\r");
  */  
}

// END MAIN ----------------------------------------------------------------------------------------------------------------------------

// Magnetometer -----------------------------------------------------------------------------------------------------------------------

void magnet_init(void){  
  int  res1; 
  res1=iicstart(magnet_wr);
  res1=iictransmit(HM5883_MODE_REG );   
  res1=iictransmit(0x00 );
  iicstop();  
}

void magnet_test(void){
  
}

void hm5883_getrawdata(int *mxraw, int *myraw, int *mzraw)
{
 uint8_t i = 0;
 uint8_t buff[6];
 int res1;
	
 res1=iicstart(magnet_wr);
 res1=iictransmit(HM5883_DATAX0 );
 res1= iicrestart(magnet_rd); 
 iicswrcv();
 
 for(i=0; i<4  ;i++) {
  buff[i]=iicreceive();
 }
 
 buff[i]= iicreceivem1();
 buff[i+1]= iicreceivelast();

	*mxraw = ((buff[0] << 8) | buff[1]);
	*myraw = ((buff[2] << 8) | buff[3]);
	*mzraw = ((buff[4] << 8) | buff[5]);
}  

// Accelerometer ----------------------------------------------------------------------------------------------------------------------


void accel_init (void){
  
 int  res1;
 
 res1=iicstart(accel_wr);
 res1=iictransmit(ADXL345_POWER_CTL );  //  
 res1=iictransmit(0x08 );
  
 res1=iictransmit(ADXL345_DATA_FORMAT );  // ; 
 res1=iictransmit(0x08 );
  
 iicstop();  
}

void accel_test(void){
}


void adxl345_getrawdata(int *axraw, int *ayraw, int *azraw){
  
 uint8_t i = 0;
 uint8_t buff[6];
 int res1;
	
 res1=iicstart(accel_wr);
 res1=iictransmit(ADXL345_DATAX0 );
 res1= iicrestart(accel_rd); 
 iicswrcv();
 
 for(i=0; i<4  ;i++) {
  buff[i]=iicreceive();
 }
 
 buff[i]= iicreceivem1();
 buff[i+1]= iicreceivelast();

	*axraw = ((buff[1] << 8) | buff[0]);
	*ayraw = ((buff[3] << 8) | buff[2]);
	*azraw = ((buff[5] << 8) | buff[4]);
}  
  


// test the precense of Gyro , should get 211 on return 
// 

void gyro_test(void) {
 int res1,who; 
 
 res1=iicstart(0xD2);
 res1=iictransmit(L3G4200D_WHO_AM_I);
  
 res1=iicrestart(0xD3);
 who=iicreceiveone();
 //who=who & 0x00ff;     Debugging  info
 //PORTB=  who ;

}


 //  Gyro Initialisation
 
 void gyro_init (void) {
  
 int  res1;
 
 res1=iicstart(gyro_wr);
 res1=iictransmit(L3G4200D_CTRL_REG1 );  // ; 100hz, 12.5Hz, Power up
 res1=iictransmit(0x0f );
 iicstop();  
 }
 
 
// Function to get a set of gyro data
// Eduardo Nebot,30 July 2015 

void l3g4200d_getrawdata(int *gxraw, int *gyraw, int *gzraw) {
 	uint8_t i = 0;
	uint8_t buff[6];
	int res1;
	
   res1=iicstart(gyro_wr);
   res1=iictransmit(L3G4200D_OUT_XYZ_CONT );
   res1= iicrestart(gyro_rd); 
 
 iicswrcv();
 
 for(i=0; i<4  ;i++) {
  buff[i]=iicreceive();
 }
 
buff[i]= iicreceivem1();
buff[i+1]= iicreceivelast();

	*gxraw = ((buff[1] << 8) | buff[0]);
	*gyraw = ((buff[3] << 8) | buff[2]);
	*gzraw = ((buff[5] << 8) | buff[4]);
}

// ********************





 // Laser  initialisation, Eduardo nebot , 2/8/15 
 void laser_init (void) {
  int  res1;
  //k=iicstart(LidarWriteAddr);
  k=iicstart(laser_wr);   // 0xc4
  //delay1(10);
 
  k=iictransmit(0x00 );  // ; write register 0x00 with value 0x04
                          // (This performs a DC stabilization cycle,
                          // Signal Acquisition, Data processing).  
                          //Refer to the section “I2C Protocol Summary
                          // in this manual for more information about
                          // I2C Communications
 

  //delay1(10);
  k=iictransmit(0x04 );
  iicstop();  
 }

 // Laser IIC Function  ( Does not work with current laser )
 // Read the PWM signal
void laser_data(uint16_t *Dist) {

  uint8_t buff[2],k1;
  int res1;
	
  laser_init();
  delay1(20);      
  k=iicstart(laser_wr);    //0xc4
  k=iictransmit(0x8f);
  delay1(20);
  k= iicrestart(laser_rd);   //c5
  //   delay1(10)  ;
  iicswrcv();
  delay1(20);
  buff[0]= iicreceivem1();      // read 2 bytes
  buff[1]= iicreceivelast();
	*Dist = ((buff[0] << 8) | buff[1]);	
 //   delay1(3)  ;
 PORTB=*Dist &0x00ff;     //debugging   
}

void setAlarm1(uint16_t msDelay1)
{
    alarmTime1 = currentTime1 + msDelay1;
    alarmSet1 = 1;
    alarmSignaled1 = 0;
}

void delay1(uint16_t msec)
{
    TC6 = TCNT + 24000; // Set initial time
    setAlarm1(msec);
    while(!alarmSignaled1) {};
}

/*  Interrupt   EMN */

// interrupt(((0x10000-Vtimch7)/2)-1) void TC7_ISR(void){
// the line above is to make it portable between differen
// Freescale processors
// The symbols for each interrupt ( in this case Vtimch7 )'
// are defined in the provided variable definition file
// I am usign a much simpler definition ( vector number) 
// that is easier to understand

interrupt 14 void TC6_ISR(void) {
   
  TC6 =TCNT + 24000;   // interrupt every msec
  TFLG1=TFLG1 | TFLG1_C6F_MASK;
  currentTime1++;
    if (alarmSet1 && currentTime1 == alarmTime1)
    {
        alarmSignaled1 = 1;
        alarmSet1 = 0;
    }
   //PORTB=PORTB+1;        // count   (debugging)
}

// Initialise timer on channel 6
void Init_TC6 (void) {
  
_asm SEI;

TSCR1=0x80;
TSCR2=0x00;   // prescaler 1
TIOS=TIOS | TIOS_IOS6_MASK;
TCTL1=0x40;
TIE=TIE | 0x40;;

 _asm CLI;
 
}
