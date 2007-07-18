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

#include "uart.h"
#include "wait.h"

/* usbstack includes */
#include <core/core.h>
#include <host/host.h>
#include <drivers/mon/mon.h>
#include <drivers/class/hub.h>
#include <drivers/class/storage.h>

/*

SIGNAL(SIG_INTERRUPT0)
{
  hcdi_irq();
}

/// interrupt vector for rx uart
SIGNAL(SIG_UART_RECV)
{
  #if USBMON
  //usb_mon_stdin(UARTGetChar());
  #endif
}

*/

ISR(USB_GEN_vect)
{
  UARTWrite("H");
  USART_WriteHex(UHINT);
  //UARTWrite("\r\n");
  UHINT = 0x00;
}


int main(void)
{

  /* initial uart */
  USART_Init();
  usb_mon_stdout(&UARTWrite);
  
  /* activate global interrupts */
 
  usb_init();
  //sei();	
 
  while(1){
    wait_ms(1000);
    usb_periodic();
  }

}

