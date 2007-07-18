// AT90USB/usart_drv.h
// Basic output routines for USART, adapted to Atmels AT90USB microcontrollers
// Based on AT90USB datasheet (7593D-AVR-07/06), chapter 18 and examples from
// http://www.mikrocontroller.net/articles/AVR-GCC-Tutorial and
// http://www.roboternetz.de/wissen/index.php/UART_mit_avr-gcc
// S. Salewski 29-JAN-2007

#ifndef _USART_DRV_H_
#define _USART_DRV_H_

#include <stdint.h>
#include "defines.h"

void UARTWrite(char *s);

void USART_Init(void); // initialize USART, 8N1 mode
void USART_WriteChar(char c);
void USART_WriteHex(unsigned char c);
void USART_WriteHexW(uint16_t w);
void USART_WriteString(char *s);
void USART_NewLine(void);

#endif
