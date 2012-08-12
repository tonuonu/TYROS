#include "ior32c111.h"
#include <intrinsics.h>
#include "gyro.h"
#include "hwsetup.h"
#include "main.h"
#include "SPI.h"

/* Buffer to store the received data	*/
char gyro_RecBuff[8];
int gyrowhoami=0;
signed char gyrox=0,gyroy=0,gyroz=0;
int gyrowhoamistatus=0;
void
SPI6_Init(void) { // Gyro
#define	f1_CLK_SPEED 24000000
    u6brg = (unsigned char)(((f1_CLK_SPEED)/(2*1000000))-1);

    CS6d = PD_OUTPUT;
    CS6=1; // CS is high, means disabled

    CLOCK6d = PD_OUTPUT;
    CLOCK6s = PF_UART;

    TX6d = PD_OUTPUT;
    TX6s = PF_UART;

    RX6s = PF_UART;

    smd0_u6mr  = 1;                                        // \ 
    smd1_u6mr  = 0;                                        //  | Synchronous Serial Mode
    smd2_u6mr  = 0;                                        // /

    ckdir_u6mr = 0;                                        // 0=internal clock 
    stps_u6mr  = 0;                                        // 0=1 stop bit, 0 required
    pry_u6mr   = 0;                                        // Parity, 0=odd, 0 required 
    prye_u6mr  = 0;                                        // Parity Enable? 0=disable, 0 required 
    iopol_u6mr = 0;                                        // IO Polarity, 0=not inverted, 0 required

    clk0_u6c0 = 0;                                         // Clock source f1 for u4brg
    clk1_u6c0 = 0;                                         // 
    txept_u6c0 = 0;                                        // Transmit register empty flag 
    crd_u6c0 = 1;                                          // CTS disabled when 1
    nch_u6c0 = 0;                                          // 0=Output mode "open drain" for TXD and CLOCK pin 
    ckpol_u6c0 = 0;                                        // CLK Polarity 0 rising edge, 1 falling edge
    uform_u6c0 = 1;                                        // 1=MSB first

    te_u6c1 = 1;                                          // 1=Transmission Enable
    ti_u6c1 = 0;                                           // Must be 0 to send or receive
    re_u6c1 = 1;                                           // Reception Enable when 1
    ri_u6c1 = 0;                                           // Receive complete flag - U4RB is empty.
    u6irs_u6c1 = 1;                                        // Interrupt  when transmission  is completed, U4TB is empty. 
    u6rrm_u6c1 = 0;                                        // Continuous receive mode off
    u6lch_u6c1 = 0;                                        // Logical inversion off 

    u6smr = 0x00;                                          // Set 0 
    u6smr2 = 0x00;                                         // Set 0 

    sse_u6smr3 = 0;                                        // SS is disabled when 0
    ckph_u6smr3 = 0;                                       // Non clock delayed 
    dinc_u6smr3 = 0;                                       // Master mode when 0
    nodc_u6smr3 = 0;                                       // Select a clock output mode "push-pull" when 0 
    err_u6smr3 = 0;                                        // Error flag, no error when 0 
    dl0_u6smr3 = 0;                                        // Set 0 for no  delay 
    dl1_u6smr3 = 0;                                        // Set 0 for no  delay 
    dl2_u6smr3 = 0;                                        // Set 0 for no  delay 

    u6smr4 = 0x00;       
    
    DISABLE_IRQ
    ilvl_s6ric =0x04;       
    ir_s6ric   =0;            
    ENABLE_IRQ
    pu11 = 1; // gyro RX interface needs pullup on RX6 or p4_6
}


#pragma vector = UART6_RX
__interrupt void _uart6_receive(void) {

  /* Used to reference a specific location in the array while string the
  received data.   */
  static unsigned char uc_cnt=0;
  /* Copy the received data to the global variable 'gyro_RecBuff'	*/
  gyro_RecBuff[uc_cnt] = (char) u6rb ;
 
  switch(gyrowhoamistatus) {
  case 0: // Request sent, sending dummy byte to get the answer
  case 6:
  case 8:
  case 10:
      u6tb=0xFF;
      break;
  case 1: // WHOAMI answer received. Sending request to write ctrl_REG2
      gyrowhoami=(int)gyro_RecBuff[uc_cnt];
      CS6=1;
//      uDelay(5);
      CS6=0;
      u6tb=(L3G4200D_CTRL_REG2 | 0x00) ;   
      break;
  case 2: // REG2 written, writing  into it
      u6tb=16|32; 
      break;
  case 3: 
      CS6=1;
//      uDelay(5);
      CS6=0;
      u6tb=(L3G4200D_CTRL_REG1 | 0x00) ; 
      break;
  case 4: // REG1 written, Enabling X,Y,Z axes and normal mode
      u6tb=1|2|4|8; 
      break;
  case 5: // written. Trying to get XOUTL
      CS6=1;
//      uDelay(5);
      CS6=0;
      u6tb=L3G4200D_OUT_X_L | 0x80;
      break;
  case 7: // XOUTL sent, trying to read answer
      gyrox=(int)gyro_RecBuff[uc_cnt];
      CS6=1;
//      uDelay(5);
      CS6=0;
      u6tb=L3G4200D_OUT_Y_L | 0x80;
      break;
  case 9: // YOUTL sent, trying to read answer
      gyroy=(int)gyro_RecBuff[uc_cnt];
      CS6=1;
//      uDelay(5);
      CS6=0;
      u6tb=L3G4200D_OUT_Z_L | 0x80;
      break;
  case 11: // ZOUTL sent, trying to read answer
      gyroz=(int)gyro_RecBuff[uc_cnt];
      CS6=1;
//      uDelay(5);
      CS6=0;
      u6tb=L3G4200D_WHOAMI | 0x80;
      gyrowhoamistatus=-1;
      break;
  } 
  gyrowhoamistatus++;

  /* Check if the buffer size is exceed. If it is then reset the 'uc_cnt'
  variable.    */
  if(uc_cnt++ >= sizeof(gyro_RecBuff)) {
    /* Reinitialize the buffer reference.	*/
    uc_cnt = 0;
  }

  /* Clear the 'reception complete' flag.	*/
  ir_s6ric = 0;
  
}


#if 0

// Reads the 3 gyro channels and stores them in vector g
void read()
{
//	Wire.beginTransmission(GYR_ADDRESS);
	// assert the MSB of the address to get the gyro 
	// to do slave-transmit subaddress updating.
//	Wire.write(L3G4200D_OUT_X_L | (1 << 7)); 
//	Wire.endTransmission();
//	Wire.requestFrom(GYR_ADDRESS, 6);

//	while (Wire.available() < 6);
	
//	unsigned char xla = Wire.read();
//	unsigned char xha = Wire.read();
//	unsigned char yla = Wire.read();
//	unsigned char yha = Wire.read();
//	unsigned char zla = Wire.read();
//	unsigned char zha = Wire.read();

//	g.x = xha << 8 | xla;
//	g.y = yha << 8 | yla;
//	g.z = zha << 8 | zla;
}
void L3G4200D::vector_cross(const vector *a,const vector *b, vector *out)
{
  out->x = a->y*b->z - a->z*b->y;
  out->y = a->z*b->x - a->x*b->z;
  out->z = a->x*b->y - a->y*b->x;
}

float L3G4200D::vector_dot(const vector *a,const vector *b)
{
  return a->x*b->x+a->y*b->y+a->z*b->z;
}

void L3G4200D::vector_normalize(vector *a)
{
  float mag = sqrt(vector_dot(a,a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}
#endif