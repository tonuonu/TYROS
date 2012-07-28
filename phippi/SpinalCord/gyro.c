#include "ior32c111.h"
#include <intrinsics.h>
#include "gyro.h"
#include "hwsetup.h"
#include "main.h"
#include "SPI.h"

signed char 
gyro_read(unsigned char r) {
    unsigned char c;
    CS6=0; // enable gyro
    uDelay(100);
    /*
     * 0x80 is most significant bit=1 and indicates we 
     * want to read register, not write. 
     */
    SPI6_send(r | 0x80); 
    uDelay(100);
    c=SPI6_receive() & 0xFF; // strip off anything over 8 bits
    uDelay(100);
    CS6=1; // disable gyro  
    return c;
}

void
gyro_write(unsigned char r, unsigned char data) {
        CS6=0; // enable gyro
        uDelay(100);
        /*
         * 0x80 is most significant bit=1 and indicates we 
         * want to read register, not write. 0x20 is "turn on"
         * and should be responded by 1101 0011 or 211 in dec or 0xd3 in hex 
         */
        SPI6_send(r | 0x00); 
        uDelay(100);
        SPI6_send(data); 
        uDelay(100);
        CS6=1; // disable gyro
        uDelay(100);
}


void gyro_Init (void) {
#if 0
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG6 ?
#define Reference 0x25
#define INT1_THS  0x
#define INT1_DUR  0x38
#define INT1_CFG  0x30
#define CTRL_REG5 0x24
#define CTRL_REG1 0x20
#endif
    gyro_write(CTRL_REG2, 16|32);
    
    gyro_write(CTRL_REG1, 1|2|4|8);
        
}

#if 0

// Turns on the L3G4200D's gyro and places it in normal mode.
void enableDefault(void)
{
	// 0x0F = 0b00001111
	// Normal power mode, all axes enabled
	writeReg(CTRL_REG1, 0x0F);
}

// Reads a gyro register
char readReg(char reg)
{
	char value;
	
//	Wire.beginTransmission(GYR_ADDRESS);
//	Wire.write(reg);
//	Wire.endTransmission();
//	Wire.requestFrom(GYR_ADDRESS, 1);
//	value = Wire.read();
//	Wire.endTransmission();
	
	return value;
}

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