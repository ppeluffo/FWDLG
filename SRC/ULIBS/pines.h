    /* 
 * File:   pines.h
 * Author: pablo
 *
 * Created on 11 de febrero de 2022, 06:02 PM
 */

#ifndef PINES_H
#define	PINES_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <avr/io.h>
#include "stdbool.h"
    
//------------------------------------------------------------------------------
void SET_RTS_RS485(void);
void CLEAR_RTS_RS485(void);
void CONFIG_RTS_485(void);

void SET_EN_PWR_CPRES(void);
void CLEAR_EN_PWR_CPRES(void);
void CONFIG_EN_PWR_CPRES(void);

void SET_EN_PWR_SENSEXT(void);
void CLEAR_EN_PWR_SENSEXT(void);
void CONFIG_EN_PWR_SENSEXT(void);

void SET_EN_PWR_QMBUS(void);
void CLEAR_EN_PWR_QMBUS(void);
void CONFIG_EN_PWR_QMBUS(void);

void SET_PWR_SENSORS(void);
void CLEAR_PWR_SENSORS(void);
void CONFIG_PWR_SENSORS(void);

void SET_EN_SENS3V3(void);
void CLEAR_EN_SENS3V3(void);
void CONFIG_EN_SENS3V3(void);

void SET_EN_SENS12V(void);
void CLEAR_EN_SENS12V(void);
void CONFIG_EN_SENS12V(void);

#ifdef	__cplusplus
}
#endif

#endif	/* PINES_H */

