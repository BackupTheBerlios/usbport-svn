/*
 * Copyright (c) 2007, Benedikt Sauter <sauter@ixbat.de>
 * All rights reserved.
 *
 * Short descripton of file: sl811hs-io.c 
 * connected to processor bus
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

#include "sl811hs-io.h"
#include <lib/types.h>

#include <avr/io.h>
#include "wait.h"

//TODO remove this include!!
#include "uart.h"


void sl811_init()
{
  int mem;		
  /* init */
  CNTRL_DDR = 0xf8;	//  init output rst,cs,wr,rd,a0 as output ; int as input
  CNTRL = 0x04;

  sl811_reset();

  CNTRL &= ~(1<<nRST);	// low
  CNTRL |= (1<<A0);	// high
  CNTRL |= (1<<nRD);	// high
  CNTRL |= (1<<nWR);	// high
  CNTRL &= ~(1<<nCS);	// low
  wait_ms(1);
  CNTRL |= (1<<nRST);	// high
  wait_ms(10);

  //DATA_DDR = 0xff; 		// change with macros 
  //DATA_OUT = 0x00;
  
  //DDRD = 0x00;
  //PORTD = 0x00;
  /* clear memory */
  sl811_write(0x00,0x00);
  for(mem=0x00;mem<0xFF;mem++)
  	sl811_write_burst(0x00);
}

void sl811_reset()
{
  CNTRL &= ~(1<<nRST);    // low
  wait_ms(1);
  CNTRL |= (1<<nRST);     // high
  wait_ms(4);
}


void sl811_write(u8 addr, u8 data)
{

  DATA_DDR = 0xFF;

  CNTRL &= ~(1<<nCS);	// low
  CNTRL &= ~(1<<A0);	// low
    	
  DATA_OUT = addr;       	// load address

  CNTRL &= ~(1<<nWR);	// low
  asm("nop");
  asm("nop");
  CNTRL |= (1<<nWR);	// high
  CNTRL |= (1<<A0);	// high

  DATA_OUT = data;

  CNTRL &= ~(1<<nWR);    // low
  asm("nop");
  CNTRL |= (1<<nWR);      // high
  CNTRL |= (1<<nCS);      // high
  
}

void sl811_write_burst(u8 data)
{
  CNTRL |= (1<<A0);	// high
  CNTRL &= ~(1<<nCS);	// low

  //DATA_DDR = 0xFF;
    	
  DATA_OUT = data;

    CNTRL &= ~(1<<nWR);    // low
  asm("nop");
  asm("nop");
  CNTRL |= (1<<nWR);      // high
  CNTRL |= (1<<nCS);      // high
  
  //DATA_DDR = 0x00;
  //DATA_OUT = 0xff;
}

u8 sl811_read(u8 addr)
{
  u8 result;
  /* read */

  DATA_DDR = 0xFF;

  CNTRL &= ~(1<<nCS);	// low
  CNTRL &= ~(1<<A0);	// low
    	
  DATA_OUT = addr;       	// load address

  CNTRL &= ~(1<<nWR);	// low
  asm("nop");
  asm("nop");
  CNTRL |= (1<<nWR);	// high
  CNTRL |= (1<<A0);	// high

  //DATA_OUT = 0x00;	// be sure that port is now clean
  DATA_DDR = 0x00;	// switch to input mode
  DATA_OUT = 0xff;	// pullups 
  //DATA_IN = 0x00;	// be sure that port is now clean

  CNTRL &= ~(1<<nRD);    	// low
  asm("nop");
  result = DATA_IN;	// get value of address
  CNTRL |= (1<<nRD);      // high
  CNTRL |= (1<<nCS);      // high
  
  DATA_DDR = 0x00;
  //DATA_OUT = 0xff;	// pullups 
  return result;
}


u8 sl811_read_burst()
{
  u8 result;
  CNTRL |= (1<<A0);	// high
  CNTRL &= ~(1<<nCS);	// low

  DATA_DDR = 0x00;
  DATA_OUT = 0xff;	// pullups 

  CNTRL &= ~(1<<nRD);    	// low
  asm("nop");
  asm("nop");
  result = DATA_IN;	// get value of address
  CNTRL |= (1<<nRD);      // high
  CNTRL |= (1<<nCS);      // high

  return result;
}

void sl811_write_buf(u8 addr, unsigned char *buffer,u8 size)
{
  u8 i=1;
  sl811_write(addr,buffer[0]);
  size--;
  while(size!=0)
  {
    sl811_write_burst(buffer[i]);
    i++;
    size--;
  }
}

void sl811_read_buf(u8 addr, unsigned char *buffer,u16 size)
{
  u8 i=1;
  if(size<=0)
  	return;

  buffer[0]=sl811_read(addr);
  size--;
  while(size!=0)
  {
    buffer[i] = sl811_read_burst();
    i++;
    size--;
  }
}

