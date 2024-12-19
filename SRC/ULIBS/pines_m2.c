
#include "pines.h"

 
// RS485_RTS (OK))
#define RTS_RS485_PORT         PORTC
#define RTS_RS485              2
#define RTS_RS485_PIN_bm       PIN2_bm
#define RTS_RS485_PIN_bp       PIN2_bp

    
// EN_PWR_CPRES (output)
#define EN_PWR_CPRES_PORT         PORTD
#define EN_PWR_CPRES              6
#define EN_PWR_CPRES_PIN_bm       PIN6_bm
#define EN_PWR_CPRES_PIN_bp       PIN6_bp

    
// EN_PWR_SENSEXT (output)
#define EN_PWR_SENSEXT_PORT         PORTF
#define EN_PWR_SENSEXT              0
#define EN_PWR_SENSEXT_PIN_bm       PIN0_bm
#define EN_PWR_SENSEXT_PIN_bp       PIN0_bp


// EN_PWR_QMBUS (output)
#define EN_PWR_QMBUS_PORT         PORTF
#define EN_PWR_QMBUS              1
#define EN_PWR_QMBUS_PIN_bm       PIN1_bm
#define EN_PWR_QMBUS_PIN_bp       PIN1_bp
    
// PWR_SENSORS (sensors de 4-20, output) (OK)
#define PWR_SENSORS_PORT         PORTD
#define PWR_SENSORS              1
#define PWR_SENSORS_PIN_bm       PIN1_bm
#define PWR_SENSORS_PIN_bp       PIN1_bp
    
// EN_SENS3V3 (output)
#define EN_SENS3V3_PORT         PORTD
#define EN_SENS3V3              5
#define EN_SENS3V3_PIN_bm       PIN5_bm
#define EN_SENS3V3_PIN_bp       PIN5_bp
    
// EN_SENS12V (output)
#define EN_SENS12V_PORT         PORTD
#define EN_SENS12V              4
#define EN_SENS12V_PIN_bm       PIN4_bm
#define EN_SENS12V_PIN_bp       PIN4_bp

// -----------------------------------------------------------------------------

void SET_RTS_RS485(void)
{
    ( RTS_RS485_PORT.OUT |= RTS_RS485_PIN_bm );
}
//------------------------------------------------------------------------------
void CLEAR_RTS_RS485(void)
{
    ( RTS_RS485_PORT.OUT &= ~RTS_RS485_PIN_bm );
}
//------------------------------------------------------------------------------
void CONFIG_RTS_485(void)
{
    RTS_RS485_PORT.DIR |= RTS_RS485_PIN_bm;
}
//------------------------------------------------------------------------------
void SET_EN_PWR_CPRES(void)
{
    ( EN_PWR_CPRES_PORT.OUT |= EN_PWR_CPRES_PIN_bm );
}
//------------------------------------------------------------------------------
void CLEAR_EN_PWR_CPRES(void)
{
    ( EN_PWR_CPRES_PORT.OUT &= ~EN_PWR_CPRES_PIN_bm );
}
//------------------------------------------------------------------------------
void CONFIG_EN_PWR_CPRES(void)
{
    EN_PWR_CPRES_PORT.DIR |= EN_PWR_CPRES_PIN_bm;
}
//------------------------------------------------------------------------------
void SET_EN_PWR_SENSEXT(void)
{
    ( EN_PWR_SENSEXT_PORT.OUT |= EN_PWR_SENSEXT_PIN_bm );
}
//------------------------------------------------------------------------------
void CLEAR_EN_PWR_SENSEXT(void)
{
    ( EN_PWR_SENSEXT_PORT.OUT &= ~EN_PWR_SENSEXT_PIN_bm );
}
//------------------------------------------------------------------------------
void CONFIG_EN_PWR_SENSEXT(void)
{
    EN_PWR_SENSEXT_PORT.DIR |= EN_PWR_SENSEXT_PIN_bm;
}
//------------------------------------------------------------------------------
void SET_EN_PWR_QMBUS(void)
{
    ( EN_PWR_QMBUS_PORT.OUT |= EN_PWR_QMBUS_PIN_bm );
}
//------------------------------------------------------------------------------
void CLEAR_EN_PWR_QMBUS(void)
{
    ( EN_PWR_QMBUS_PORT.OUT &= ~EN_PWR_QMBUS_PIN_bm );
}
//------------------------------------------------------------------------------
void CONFIG_EN_PWR_QMBUS(void)
{
    EN_PWR_QMBUS_PORT.DIR |= EN_PWR_QMBUS_PIN_bm;
}
//------------------------------------------------------------------------------
void SET_PWR_SENSORS(void)
{
    ( PWR_SENSORS_PORT.OUT |= PWR_SENSORS_PIN_bm );
}
//------------------------------------------------------------------------------
void CLEAR_PWR_SENSORS(void)
{
    ( PWR_SENSORS_PORT.OUT &= ~PWR_SENSORS_PIN_bm );
}
//------------------------------------------------------------------------------
void CONFIG_PWR_SENSORS(void)
{
    PWR_SENSORS_PORT.DIR |= PWR_SENSORS_PIN_bm;
}
//------------------------------------------------------------------------------
void SET_EN_SENS3V3(void)
{
    ( EN_SENS3V3_PORT.OUT |= EN_SENS3V3_PIN_bm );
}
//------------------------------------------------------------------------------
void CLEAR_EN_SENS3V3(void)
{
    ( EN_SENS3V3_PORT.OUT &= ~EN_SENS3V3_PIN_bm );
}
//------------------------------------------------------------------------------
void CONFIG_EN_SENS3V3(void)
{
    EN_PWR_CPRES_PORT.DIR |= EN_SENS3V3_PIN_bm;
}
//------------------------------------------------------------------------------
void SET_EN_SENS12V(void)
{
    ( EN_SENS12V_PORT.OUT |= EN_SENS12V_PIN_bm );
}
//------------------------------------------------------------------------------
void CLEAR_EN_SENS12V(void)
{
    ( EN_SENS12V_PORT.OUT &= ~EN_SENS12V_PIN_bm );
}
//------------------------------------------------------------------------------
void CONFIG_EN_SENS12V(void)
{
    EN_PWR_CPRES_PORT.DIR |= EN_SENS12V_PIN_bm;
}
//------------------------------------------------------------------------------
