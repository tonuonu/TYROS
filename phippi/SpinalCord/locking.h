/*
 *  Copyright (c) 2012 Tonu Samuel
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
extern volatile unsigned int the_lock1;
extern volatile unsigned int the_lock2;

/*
 * IAR manual describes __monitor keyword:
 * A monitor function causes interrupts to be disabled during execution of the 
 * function. At function entry, the status register is saved and interrupts are 
 * disabled. At function exit, the original status register is restored, and 
 * thereby the interrupt status that existed before the function call is also 
 * restored
 */

__monitor static inline int get_lock1(void) {
    if (the_lock1 == 0) {
        /* Success, we managed to lock the lock. */
        the_lock1 = 1;
        return 1;
    } else {
        /* Failure, someone else has locked the lock. */
       return 0;
    }
}

__monitor static inline int get_lock2(void) {
    if (the_lock2 == 0) {
        /* Success, we managed to lock the lock. */
        the_lock2 = 1;
        return 1;
    } else {
        /* Failure, someone else has locked the lock. */
       return 0;
    }
}

__monitor static inline void release_lock1(void) {
    the_lock1 = 0;
}

__monitor static void release_lock2(void) {
    the_lock2 = 0;
}
