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

// register addresses
#define L3G4200D_WHOAMI        0x0F
#define L3G4200D_WHOAMI_MAGIC  211

#define L3G4200D_CTRL_REG1     0x20
#define L3G4200D_CTRL_REG2     0x21
#define L3G4200D_CTRL_REG3     0x22
#define L3G4200D_CTRL_REG4     0x23
#define L3G4200D_CTRL_REG5     0x24
#define L3G4200D_REFERENCE     0x25
#define L3G4200D_OUT_TEMP      0x26
#define L3G4200D_STATUS_REG    0x27

#define L3G4200D_OUT_X_L       0x28
#define L3G4200D_OUT_X_H       0x29
#define L3G4200D_OUT_Y_L       0x2A
#define L3G4200D_OUT_Y_H       0x2B
#define L3G4200D_OUT_Z_L       0x2C
#define L3G4200D_OUT_Z_H       0x2D

#define L3G4200D_FIFO_CTRL_REG 0x2E
#define L3G4200D_FIFO_SRC_REG  0x2F

#define L3G4200D_INT1_CFG      0x30
#define L3G4200D_INT1_SRC      0x31
#define L3G4200D_INT1_THS_XH   0x32
#define L3G4200D_INT1_THS_XL   0x33
#define L3G4200D_INT1_THS_YH   0x34
#define L3G4200D_INT1_THS_YL   0x35
#define L3G4200D_INT1_THS_ZH   0x36
#define L3G4200D_INT1_THS_ZL   0x37
#define L3G4200D_INT1_DURATION 0x38

#define L3G4200D_REG3_I2_DRDY    (1 << 3)
#define L3G4200D_REG3_PP_OD      (1 << 4)
#define L3G4200D_REG3_H_LACTIVE  (1 << 5)
#define L3G4200D_REG3_I1_INT     (1 << 7)

#define L3G4200D_REG2_HPCF0  (1 << 0)
#define L3G4200D_REG2_HPCF1  (1 << 1)
#define L3G4200D_REG2_HPCF2  (1 << 2)
#define L3G4200D_REG2_HPCF3  (1 << 3)
#define L3G4200D_REG2_HPM0   (1 << 4)
#define L3G4200D_REG2_HPM1   (1 << 5)

#define L3G4200D_REG1_XEN    (1 << 0)
#define L3G4200D_REG1_YEN    (1 << 1)
#define L3G4200D_REG1_ZEN    (1 << 2)
#define L3G4200D_REG1_PD     (1 << 3)
#define L3G4200D_REG1_BW0    (1 << 4)
#define L3G4200D_REG1_BW1    (1 << 5)
#define L3G4200D_REG1_DR0    (1 << 6)
#define L3G4200D_REG1_DR1    (1 << 7)

#define L3G4200D_REG_INT1_CFG_ANDOR  (1 << 7)
#define L3G4200D_REG_INT1_CFG_LIR    (1 << 6)
#define L3G4200D_REG_INT1_CFG_ZHIE   (1 << 5)
#define L3G4200D_REG_INT1_CFG_ZLIE   (1 << 4)
#define L3G4200D_REG_INT1_CFG_YHIE   (1 << 3)
#define L3G4200D_REG_INT1_CFG_YLIE   (1 << 2)
#define L3G4200D_REG_INT1_CFG_XHIE   (1 << 1)
#define L3G4200D_REG_INT1_CFG_XLIE   (1 << 0)
