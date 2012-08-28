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

extern void uart8_init(void);
extern void uart5_init(void);
extern void uart7_init(void);
extern void uart7_send(unsigned char);
extern void uart8_send (unsigned char);

#define TX_BUFF_SIZE  256 // 512
#define RX_BUFF_SIZE  1024

extern char tx5_buff[TX_BUFF_SIZE ];    
extern unsigned short tx5_ptr;        
extern volatile unsigned short tx5_ptrr;   
extern char rx8_buff[RX_BUFF_SIZE ];   
extern unsigned short rx8_ptr;        
extern unsigned short rx8_ptrr;      

#define OK 1
#define ERROR -1
#define FULL 0x80
#define EMPTY 0

#define rd_buff     rx8_buff
#define datat_buf   rx8_ptr


extern unsigned char get_uart5(void);

/* 
 * Many computer terminals and terminal emulators support colour and cursor 
 * control through a system of escape sequences. One such standard is commonly 
 * referred to as ANSI Colour. Several terminal specifications are based on the 
 * ANSI colour standard, including VT100.
 * http://www.termsys.demon.co.uk/vtansi.htm
 */

#define VT100RESET            "\x1b" "c"       // Reset device
#define VT100RESETATTR        "\x1b" "0m"      // Reset all display attributes (color, blink, hidden, reverse, underscore,...)
#define VT100ERASESTARTOFLINE "\x1b" "[1K"     // Erases from the current cursor position to the start of the current line.
#define VT100CURSORHOME       "\x1b" "[15;0H"  // Sets the cursor position where subsequent text will begin
#define VT100CURSORNULL       "\x1b" "[00;0H"  // Set cursor to nullpoint
#define VT100ERASESCREEN      "\x1b" "[2J"     // Erases the screen with the background colour and moves the cursor to home.

#define VT100CURSORODOMETRY   "\x1b" "[02;1H"  // Set cursor to nullpoint
#define VT100CURSORMELEXISE   "\x1b" "[03;1H"  // Set cursor to nullpoint
#define VT100CURSORMELEXISL   "\x1b" "[04;1H"  // Set cursor to nullpoint
#define VT100CURSORMELEXISR   "\x1b" "[05;1H"  // Set cursor to nullpoint
#define VT100CURSORBALL       "\x1b" "[06;1H"  // Set cursor to nullpoint
#define VT100CURSORPANDA      "\x1b" "[07;1H"  // Set cursor to nullpoint
#define VT100CURSORCHARGER    "\x1b" "[08;1H"  // Set cursor to nullpoint
#define VT100CURSORBATTERY    "\x1b" "[09;1H"  // Set cursor to nullpoint
#define VT100CURSORGYRO       "\x1b" "[10;1H"  // Set cursor to nullpoint
#define VT100CURSORLEFTMOTOR  "\x1b" "[11;1H"  // Set cursor to nullpoint
#define VT100CURSORRIGHTMOTOR "\x1b" "[12;1H"  // Set cursor to nullpoint
#define VT100CURSORCAPACITOR  "\x1b" "[12;1H"  // Set cursor to nullpoint
#define VT100CURSORPROMPT     "\x1b" "[14;1H"  // Set cursor to nullpoint
#define VT100SCROLLSCREEN     "\x1b" "[15;40r" // Enable scrolling from row {start} to row {end}.
#define VT100CURSORACC        "\x1b" "[16;1H"  // Set cursor to nullpoint
#define VT100SCROLLUP         "\x1"  "bM"      // Scroll display up one line.
#define VT100CURSORSAVE       "\x1b" "7"       // Save cursor and attributes
#define VT100CURSORRESTORE    "\x1b" "8"       // Restore cursor and attributes

void write(char *c) ;
void writeln(char *c);
