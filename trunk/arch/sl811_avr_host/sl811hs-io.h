/*
 * Copyright (c) 2007, Benedikt Sauter <sauter@ixbat.de>
 * All rights reserved.
 *
 * Short descripton of file: sl811hs-io.h
 *
 * supports read and write register functions for sl811hs
 *
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

#ifndef _SL811HS_IO_H_
#define _SL811HS_IO_H_
#include <lib/types.h>



#define DATA_DDR           	DDRA  //select data direction
#define DATA_OUT          	PORTA // used as outputs
#define DATA_IN          	PINA  // used as inputs

#define CNTRL_DDR		DDRD
#define CNTRL			PORTD

#define A0			PD7   // selects address pointer PB7 0x80
#define nRD			PD6   //Read Strobe Input PB6 0x40 	
#define nWR			PD5   //Write Strobe Input PB3        0x08
#define nCS			PD4   //Active LOW Chip Select PB4 -- pass 0x010
#define nRST			PD3   //SL811HS Device Active LOW Reset Input PB5 0x20

//Master/Slave Select (Host=0/Slave=1) PB2 0x04 
//#define MS			PB2	

//Interrupt to external Controller 
#define INTRQ			PD2

#define DATAOUT		DATA_DDR 0xff
#define DATAIN		DATA_DDR 0x00

/* connected to processer interface of sl811hs */
void sl811_init();
void sl811_reset();
void sl811_write(u8 addr, u8 data);
u8 sl811_read(u8 addr);

void sl811_write_burst(u8 data);
u8 sl811_read_burst();

void sl811_write_buf(u8 addr, unsigned char *buffer,u8 size);
void sl811_read_buf(u8 addr, unsigned char *buffer,u16 size);

void sl811_dump();


#endif
