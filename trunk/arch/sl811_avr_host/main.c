/*
 * Copyright (c) 2006, Benedikt Sauter <sauter@ixbat.de>
 * All rights reserved.
 *
 * Short descripton of file:
 * Example usage of usbstack in combination with an atmega32 and sl811hs
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright 
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above 
 *     copyright notice, this list of conditions and the following 
 *     disclaimer in the documentation and/or other materials provided 
 *     with the distribution.
 *   * Neither the name of the FH Augsburg nor the names of its 
 *     contributors may be used to endorse or promote products derived 
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* avr special defintions and includes */
#include <avr/io.h>
#include <avr/interrupt.h>
#include "wait.h"

#include "uart.h"

/* usbstack includes */
#include <core/core.h>
#include <host/host.h>
#include <drivers/mon/mon.h>
#include <drivers/class/hub.h>
#include <drivers/class/storage.h>

SIGNAL(SIG_INTERRUPT0)
{
  hcdi_irq();
}

/// interrupt vector for rx uart
SIGNAL(SIG_UART_RECV)
{
  #if USBMON
  usb_mon_stdin(UARTGetChar());
  #endif
}


int main(void)
{
  /* configure int0 from atmega32 */ 
  MCUCR |=  (1 << ISC01) | (1<<ISC00); // raising edge 
  GICR |= (1 << INT0);

  /* initial uart */
  #if USBMON
  //UARTInit();
  //usb_mon_stdout(&UARTWrite);
  #endif
  
  /* activate global interrupts */
  sei();	
 
  usb_init();

  //usb_hub_init();
  //usb_storage_init();

  
  while(1){
    wait_ms(100);
    usb_periodic();
  }

}

