/*
 * Copyright (c) 2006, Benedikt Sauter <sauter@ixbat.de>
 * All rights reserved.
 *
 * Short descripton of file(monitor.c):
 * command line interface for usbstack host monitor
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

#include "monitor.h"


monitor* debug_monitor_init()
{
	monitor * new = (monitor*)malloc(sizeof(monitor));
	debug_print("\r\n\r\nWelcome to the USB Host Stack Debug Monitor\r\n");
	debug_print("(c) 2006 by Benedikt Sauter\r\n\r\n");
	debug_print("usb> ");
	new->bufindex=0;
	//clear buffer
	int i;
	for(i=0;i<30;i++)
		new->buffer[i]=0x00;


	return new;
}

void debug_monitor_print(char *msg)
{
	debug_print(msg);
	debug_print("\r\nusb> ");
}

void debug_monitor_terminal(monitor *self,char input)
{
	// get chars, check if return was pressed
	if(input!=0x0D)	// enter
	{
		self->buffer[self->bufindex]=input;
		self->bufindex++;
		char p[2];
		p[0]=input;
		p[1]=0x00;
		debug_print(p);
	}
	else 
	{
		debug_print("\r\n");

		if (debug_monitor_check("help",(char*)self->buffer))
		{
			debug_print("Debugger Commands:\r\n");

			//debug_print("usb bus:\r\n");
			debug_print("\tenumerate <id>\t- Enumerate Device\r\n");
			debug_print("\tdevices \t- Display all online Devices\r\n");
			debug_print("\treset \t\t- Reset and reenumerarte USB Bus\r\n");

			//debug_print("usb controller:\r\n");
			debug_print("\tid \t\t- Print Version of USB Controller\r\n");
			debug_print("\treg \t\t- Print Registers of USB Controller\r\n");
			debug_print("\tmem \t\t- Print Memory Map of USB Controller\r\n");
			debug_print("\tinit \t\t- Initial USB Controller\r\n");

			//debug_print("misc:\r\n");
			debug_print("\thelp \t\t- Display this information\r\n");
			debug_print("\tversion \t- Show current version\r\n");
		}
		else if (debug_monitor_check("version",(char*)self->buffer))
		{
			debug_print("rev. 78 from the SVN tree 2006-07-22\r\n");
		}
		else if (debug_monitor_check("reg",(char*)self->buffer))
		{
			debug_print("Registers:\r\n");
			usb_hcd_debug_showregisters();
		}

		else if (debug_monitor_check("id",(char*)self->buffer))
		{
			usb_hcd_debug_id();
		}

		else if (debug_monitor_check("init",(char*)self->buffer))
		{
			usb_hcd_init();
			//usb_hcd_generate_reset  ();
		}
	
		else if (debug_monitor_check("enu",(char*)self->buffer))
		{
			 //sl811_enumerate_downstream(0);	
		}
		else if (debug_monitor_check("mem",(char*)self->buffer))
		{
			debug_print("Memory Map:\r\n");
			int j,i;	

			debug_print("    ");
			for(i=0;i<16;i++){
				debug_print_hex(i);
				debug_print(" ");
			}
			debug_print("\r\n");
			
			for(i=0;i<16;i++){
				debug_print_hex(i*16);
				debug_print(": ");
				for(j=0;j<16;j++){
					debug_print_hex(sl811_read((i*16)+j));
					debug_print(" ");
				}
				debug_print("\r\n");
			}
		}	
		else 
		{
			debug_print("unknown command (type help)\r\n");
			// parse input
		}
		//clear buffer
		int i;
		for(i=0;i<30;i++)
			self->buffer[i]=0x00;

		self->bufindex=0;
		debug_print("\r\nusb> ");
	}
}


int debug_monitor_check(char *string1, char *string2)
{
	int i=1,hits=0;
	while(1)
	{
		// compare same positions
		if(string2[i-1]==string1[i-1])
			hits++;
		
		if( (string1[i-1]==0x00) || string2[i-1]==0x20) //space
			break;

		if(i>10)
			break;
		i++;
	}
	// are the first signs same, return 1
	if( (hits==i) || ( (hits==(i-1)) && (string2[i-1]==0x20)) )
		return 1;
	else
		return 0;
}

