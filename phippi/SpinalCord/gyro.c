/*
 *  Copyright (c) 2011,2012 Tonu Samuel
 *  All rights reserved.
 *
 *  This file is part of TYROS.
 *
 *  TYROS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  TYROS is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with TYROS.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "ior32c111.h"
#include <intrinsics.h>
#include "gyro.h"
#include "hwsetup.h"
#include "main.h"
#include "SPI.h"

unsigned char gyrowhoami=0;
signed int gyrox=0,gyroy=0,gyroz=0;
signed int gyrorawx=0,gyrorawy=0,gyrorawz=0;
signed char gyrotemp=0;
int gyrowhoamistatus=0;
/* 
 * During startup we read CALIBRATIONSAMPLES values from sensor for all X,Y,Z acces to 
 * calculate and negate error. 
 */
int gyrocalcnt=0;
int gyromaxx=0,gyromaxy=0,gyromaxz=0;
int gyrominx=0,gyrominy=0,gyrominz=0;
static int tmp;

#define CALIBRATIONSAMPLES 500
void
SPI6_Init(void) {
#define	f1_CLK_SPEED 24000000
    // 10MHz max clock
    // u6brg = (unsigned char)(((f1_CLK_SPEED)/(2*5000000))-1);
    u6brg = (unsigned char)(((f1_CLK_SPEED)/(2*2000000))-1);

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

    clk0_u6c0 = 0;                                         // Clock source f1 for u6brg
    clk1_u6c0 = 0;                                         // 
    txept_u6c0 = 0;                                        // Transmit register empty flag 
    crd_u6c0 = 1;                                          // CTS disabled when 1
    nch_u6c0 = 0;                                          // 0=Output mode "open drain" for TXD and CLOCK pin 
    ckpol_u6c0 = 0;                                        // CLK Polarity 0 rising edge, 1 falling edge
    uform_u6c0 = 1;                                        // 1=MSB first

    te_u6c1 = 1;                                           // 1=Transmission Enable
    ti_u6c1 = 0;                                           // Must be 0 to send or receive
    re_u6c1 = 1;                                           // Reception Enable when 1
    ri_u6c1 = 0;                                           // Receive complete flag - U6RB is empty.
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
    /* 
     * Lowest interrupt priority
     * we do not care about speed
     */
    ilvl_s6ric =1; 
    ir_s6ric   =0;            
    ENABLE_IRQ
    pu11 = 1; // gyro RX interface needs pullup on RX6 or p4_6
}

void 
gyro_read_reg(unsigned char b) {
    CS6=1;
    uDelay(5);      
    CS6=0;
    u6tb=b | 0x80;
}

void 
gyro_write_reg(unsigned char b) {
    CS6=1;
    uDelay(5);      
    CS6=0;
    u6tb=b ;
}
void 
gyro_write_data(unsigned char b) {
    u6tb=b ;
}

#pragma vector = UART6_RX
__interrupt void _uart6_receive(void) {

    signed char b=(signed char)u6rb & 0xFF;
    switch(gyrowhoamistatus) {
    case 1: // WHOAMI answer received. Sending request to write ctrl_REG2
        gyrowhoami=(unsigned char)b;
        gyro_write_reg(L3G4200D_CTRL_REG2) ;   
        break;
    case 2: // REG2 written, writing  into it
        gyro_write_data(
            L3G4200D_REG2_HPM1| // Normal mode?
            L3G4200D_REG2_HPCF0|
            L3G4200D_REG2_HPCF3 // Pass anything higher than 0.1Hz
        );
        break;
    case 3: // Sending request to write ctrl_REG3
        gyro_write_reg(L3G4200D_CTRL_REG3);
        break;
    case 4: // REG3 written, writing  into it
        gyro_write_data(
            L3G4200D_REG3_I2_DRDY|   // Making sure INT2 will be called when data is ready
            L3G4200D_REG3_H_LACTIVE| // Low is Active, not high
            L3G4200D_REG3_PP_OD);     // Open drain
        break;
    case 5: // Sending request to write ctrl_INT1_CFG
        gyro_write_reg(L3G4200D_INT1_CFG);
        break;
    case 6: // REG3 written, writing  into it
        gyro_write_data(
            L3G4200D_REG_INT1_CFG_XHIE| // Generate interrupt if threshold is exceeded on X
            L3G4200D_REG_INT1_CFG_YHIE| // Generate interrupt if threshold is exceeded on Y
            L3G4200D_REG_INT1_CFG_ZHIE  // Generate interrupt if threshold is exceeded on Z
        );
        break;
    case 7:
        gyro_write_reg(L3G4200D_CTRL_REG1); 
        break;
    case 8: // REG1 written, Enabling X,Y,Z axes and normal mode
        gyro_write_data(L3G4200D_REG1_XEN|
            L3G4200D_REG1_YEN|
            L3G4200D_REG1_ZEN|
            L3G4200D_REG1_PD |
            L3G4200D_REG1_BW0| // Low pass cut off 110Hz, 
            L3G4200D_REG1_BW1|
            L3G4200D_REG1_DR0| // Data rate 800Hz
            L3G4200D_REG1_DR1
        ); 
        break;
    case 9: // written. Trying to get TEMP
        gyro_read_reg(L3G4200D_OUT_TEMP);
        break;
    case 11: // TEMP answer received. Sending request to get STATUS_REG
        gyrotemp=(signed int)b;
        gyro_read_reg(L3G4200D_STATUS_REG);
        break;
    case 13: // statusreg answer received. See if there is a new data in gyro available
/*      if(b & (1<<3)) { // no data for gyro yet, go to beginning
          gyro_read_reg(L3G4200D_WHOAMI);
          gyrowhoamistatus=-1;
          ERRORLED=0;
      } else { // new data is ready to load*/
          gyro_read_reg(L3G4200D_OUT_X_L);
/*          ERRORLED=1;
      };*/
        break;
    case 15: // OUT_X_L answer received
        tmp=(unsigned int)b;
        gyro_read_reg(L3G4200D_OUT_X_H);
        break;
    case 17: // OUT_X_H answer received
        gyrorawx= tmp+((signed char)b * 0x100);
        if(gyrocalcnt<CALIBRATIONSAMPLES) {
            if(gyrorawx>gyromaxx)
                gyromaxx=gyrorawx;
            if(gyrorawx<gyrominx)
                gyrominx=gyrorawx;
            gyrox=0;
        } else if(gyrorawx<gyrominx || gyrorawx>gyromaxx){
            gyrox=gyrorawx;
        } else {
            gyrox=0;
        }
        gyro_read_reg(L3G4200D_OUT_Y_L);
        break;
    case 19: // OUT_Y_L answer received
        tmp=(unsigned int)b;
        gyro_read_reg(L3G4200D_OUT_Y_H);
        break;
    case 21: // OUT_Y_H answer received
        gyrorawy = tmp+((signed char)b * 0x100);
        if(gyrocalcnt<CALIBRATIONSAMPLES) {
            if(gyrorawy>gyromaxy)
                gyromaxy=gyrorawy;
            if(gyrorawy<gyrominy)
                gyrominy=gyrorawy;
            gyroy=0;
        } else if(gyrorawy<gyrominy || gyrorawy>gyromaxy){
            gyroy=gyrorawy;
        } else {
            gyroy=0;
        }
        gyro_read_reg(L3G4200D_OUT_Z_L);
        break;
    case 23: // OUT_Z_L answer received
        tmp=(unsigned int)b;
        gyro_read_reg(L3G4200D_OUT_Z_H);
        break;
    case 25: // OUT_Z_H answer received
        gyrorawz= tmp+((signed char)b * 0x100);
        if(gyrocalcnt<CALIBRATIONSAMPLES) {
            if(gyrorawz>gyromaxz)
                gyromaxz=gyrorawz;
            if(gyrorawz<gyrominz)
                gyrominz=gyrorawz;
            gyroz=0;
        } else if(gyrorawz<gyrominz || gyrorawz>gyromaxz){
            gyroz=gyrorawz;
        } else {
            gyroz=0;
        }
        gyro_read_reg(L3G4200D_WHOAMI);
        gyrowhoamistatus=-1;
        gyrocalcnt++;
        break;
    default: // Request sent, sending dummy byte to get the answer
        uDelay(5);      
        gyro_write_data(0xFF);
        break;
    } 
    gyrowhoamistatus++;

    /* Clear the 'reception complete' flag */
    ir_s6ric = 0;
    //ERRORLED=0;  

}
