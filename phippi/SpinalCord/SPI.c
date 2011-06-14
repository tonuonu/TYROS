#include "types.h"
#include "ior32c111.h"
#include <intrinsics.h>
#include "hwsetup.h"

#define PDIR_INPUT    (0)
#define PDIR_OUTPUT    (1)
#define PF_SPI_CLK    (p4_1s)
#define PF_SPI_DATA   (p4_3s)
#define PF_SPI        (3)

void
SPI4_Init(void)
{
    /* 
     * CS
     */
    prc2=1;
    pd9_4 = PDIR_OUTPUT;

    p9_4=1;
    /* 
     * CLK4 
     */
    prc2=1;
    pd9_5 = PDIR_OUTPUT;

    prc2=1;
    p9_5s = PF_SPI;
    /* 
     * TXD4 
     */
    prc2=1;
    pd9_6 = PDIR_OUTPUT;

    prc2=1;
    p9_6s = PF_SPI;
    pu27=1;

    smd0_u4mr  = 1;                                        // \ 
    smd1_u4mr  = 0;                                        //  | Synchronous Serial Mode
    smd2_u4mr  = 0;                                        // /

    ckdir_u4mr = 0;                                        // 0=internal clock 
    
    stps_u4mr  = 0;                                        // 0=1 stop bit, 0 required
    pry_u4mr   = 0;                                        // Parity, 0=odd, 0 required 
    prye_u4mr  = 0;                                        // Parity Enable? 0=disable, 0 required 
    iopol_u4mr = 0;                                        // IO Polarity, 0=not inverted, 0 required

    clk0_u4c0 = 0;                                         // Clock source f1 for u4brg
    clk1_u4c0 = 0;                                         // 
    txept_u4c0 = 0;                                        // Transmit register empty flag 
    crd_u4c0 = 1;                                          // CTS disabled when 1
    nch_u4c0 = 0;                                          // 0=Output mode "push-pull" for TXD and CLOCK pin 
    ckpol_u4c0 = 0;                                        // CLK Polarity 
    uform_u4c0 = 1;                                        // 1=MSB first

    te_u4c1 = 1;                                           // 1=Transmission Enable
    ti_u4c1 = 0;                                           // Must be 0 to send or receive
    re_u4c1 = 1;                                           // Reception Enable when 1
    ri_u4c1 = 1;                                           // Receive complete flag - U4RB is empty.
    u4irs_u4c1 = 0;                                        // Interrupt  when transmission  is completed, U4TB is empty. 
    u4rrm_u4c1 = 0;                                        // Continuous receive mode off
    u4lch_u4c1 = 0;                                        // Logical inversion off 

    u4smr = 0x00;                                          // Set 0 
    u4smr2 = 0x00;                                         // Set 0 

    sse_u4smr3 = 0;                                        // SS is disabled when 0
    ckph_u4smr3 = 0;                                       // Non clock delayed 
    dinc_u4smr3 = 0;                                       // Master mode when 0
    nodc_u4smr3 = 0;                                       // Select a clock output  mode "push-pull" when 0 
    err_u4smr3 = 0;                                        // Error flag, no error when 0 
    dl0_u4smr3 = 0;                                        // Set 0 for no  delay 
    dl1_u4smr3 = 0;                                        // Set 0 for no  delay 
    dl2_u4smr3 = 0;                                        // Set 0 for no  delay 

    u4smr4 = 0x00;                                         // Set 0. u4c0 must be set before this function

//    u4brg = 55 /* 435kHz */ ;                                             // (unsigned char)(((f1_CLK_SPEED)/(2*BIT_RATE))-1);
    u4brg = 0xFF /* 435kHz */ ;                                             // (unsigned char)(((f1_CLK_SPEED)/(2*BIT_RATE))-1);
    s4tic = 0x0;
}

void
SPI3_Init(void)
{
    /* 
     * CLK3 
     */
    pd4_1 = PDIR_OUTPUT;
    PF_SPI_CLK = PF_SPI;
    /* 
     * TXD3 
     */
    pd4_3 = PDIR_OUTPUT;
    PF_SPI_DATA = PF_SPI;
    smd0_u3mr = 1;                                        // \ 
    smd1_u3mr = 0;                                         // >    // Synchronous Serial Mode
    smd2_u3mr = 0;                                         // /
    ckdir_u3mr = 0;                                        // internal clock , 243
    stps_u3mr = 0;                                         // 0 required
    pry_u3mr = 0;                                          // 0 required
    prye_u3mr = 0;                                         // 0 required
    iopol_u3mr = 0;                                        // 0 required
    clk0_u3c0 = 0;                                         // \ Clock
    clk1_u3c0 = 0;                                         // /
    txept_u3c0 = 0;                                        // Transmit
    crd_u3c0 = 1;                                          // CTS disabled 
    nch_u3c0 = 0;                                          // Output mode
    ckpol_u3c0 = 0;                                        // Polarity
    uform_u3c0 = 1;                                        // MSB first
    te_u3c1 = 1;                                           // Transmission 
    ti_u3c1 = 0;                                           // Must be 0 to 
    re_u3c1 = 0;                                           // Reception is 
    ri_u3c1 = 0;                                           // Receive
    u3irs_u3c1 = 0;                                        // 1 when
    u3rrm_u3c1 = 1;                                        // Continuous
    u3lch_u3c1 = 0;                                        // Logical
    u3smr = 0x00;                                          // Set 0
    u3smr2 = 0x00;                                         // Set 0 
    sse_u3smr3 = 0;                                        // SS is
    ckph_u3smr3 = 0;                                       // Non clock
    dinc_u3smr3 = 0;                                       // Master mode
    nodc_u3smr3 = 0;                                       // Select a
    err_u3smr3 = 0;                                        // Error flag,
    dl0_u3smr3 = 0;                                        // Set 0 for no 
    dl1_u3smr3 = 0;                                        // Set 0 for no 
    dl2_u3smr3 = 0;                                        // Set 0 for no 
    u3smr4 = 0x00;                                         // Set 0 (page
    u3brg = 3;                                             
    s3tic = 0x0;
}

