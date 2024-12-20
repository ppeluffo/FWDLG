/* 
 * File:   xprintf.h
 * Author: pablo
 *
 * Created on 8 de marzo de 2022, 10:55 AM
 */

#ifndef XPRINTF_H
#define	XPRINTF_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <avr/pgmspace.h>
//#include "usart.h"
    
    
void XPRINTF_init(void);
int xprintf( const char *fmt, ...);
int xfprintf( int fd, const char *fmt, ...);
int xprintf_P( PGM_P fmt, ...);
int xfprintf_P( int fd, PGM_P fmt, ...);
int xputs( const char *str );
int xfputs( int fd, const char *str );

int xnprintf( int fd, const char *pvBuffer, const uint16_t xBytes );

void putch(char c);
void xputChar(unsigned char c);
void xfputChar(int fd, unsigned char c);

void xputCharNS(unsigned char c);

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"

#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0') 

#ifdef	__cplusplus
}
#endif

#endif	/* XPRINTF_H */

