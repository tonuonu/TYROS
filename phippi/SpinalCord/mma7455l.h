/*
 *  Copyright (c) 2011, Tonu Samuel
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
extern int acccalcnt;
extern  signed char accx,accy,accz,acctout;

#define MMA7455L_WHOAMI_MAGIC 0x55

#define MMA7455L_REG_XOUTL  0x00
#define MMA7455L_REG_XOUTH  0x01
#define MMA7455L_REG_YOUTL  0x02
#define MMA7455L_REG_YOUTH  0x03
#define MMA7455L_REG_ZOUTL  0x04
#define MMA7455L_REG_ZOUTH  0x05
#define MMA7455L_REG_XOUT8  0x06
#define MMA7455L_REG_YOUT8  0x07
#define MMA7455L_REG_ZOUT8  0x08
#define MMA7455L_REG_STATUS  0x09
#define MMA7455L_REG_DETSRC  0x0a
#define MMA7455L_REG_TOUT  0x0b
#define MMA7455L_REG_RESERVED1  0x0c
#define MMA7455L_REG_I2CAD  0x0d
#define MMA7455L_REG_USRINF  0x0e
#define MMA7455L_REG_WHOAMI  0x0f
#define MMA7455L_REG_XOFFL  0x10
#define MMA7455L_REG_XOFFH  0x11
#define MMA7455L_REG_YOFFL  0x12
#define MMA7455L_REG_YOFFH  0x13
#define MMA7455L_REG_ZOFFL  0x14
#define MMA7455L_REG_ZOFFH  0x15
#define MMA7455L_REG_MCTL  0x16
#define MMA7455L_REG_INTRST  0x17
#define MMA7455L_REG_CTL1  0x18
#define MMA7455L_REG_CTL2  0x19
#define MMA7455L_REG_LDTH  0x1a
#define MMA7455L_REG_PDTH  0x1b
#define MMA7455L_REG_PW  0x1c
#define MMA7455L_REG_LT  0x1d
#define MMA7455L_REG_TW  0x1e
#define MMA7455L_REG_RESERVED2  0x1f

#define MMA7455_REG_I2CDIS (1<<7)

#define MMA7455L_STATUS_XDA  0x08
#define MMA7455L_STATUS_YDA  0x10
#define MMA7455L_STATUS_ZDA  0x20

#define MMA7455L_MODE_STANDBY  0
#define MMA7455L_MODE_MEASUREMENT  1
#define MMA7455L_MODE_LEVELDETECTION  0x42
#define MMA7455L_MODE_PULSEDETECTION  0x43
#define MMA7455L_MODE_MASK  0x43

#define MMA7455L_GSELECT_8  0x0
#define MMA7455L_GSELECT_2  0x4
#define MMA7455L_GSELECT_4  0x8
#define MMA7455L_GSELECT_MASK  0xC
#define MMA7455L_WRITE_BIT 0x80
