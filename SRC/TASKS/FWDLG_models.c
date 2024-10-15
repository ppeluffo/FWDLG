
#include "FWDLG.h"


#ifdef MODEL_M3


#define TERM_SENSE_PORT         PORTA
#define TERM_SENSE_PIN              3
#define TERM_SENSE_PIN_bm       PIN3_bm
#define TERM_SENSE_PIN_bp       PIN3_bp

#define CONFIG_TERM_SENSE_DIR()     TERM_SENSE_PORT.DIR &= ~TERM_SENSE_PIN_bm;
#define CONFIG_TERM_SENSE_PIN()     TERM_SENSE_PORT.PIN3CTRL |= PORT_PULLUPEN_bm;
#define READ_TERM_SENSE()       ( ( TERM_SENSE_PORT.IN & TERM_SENSE_PIN_bm ) >> TERM_SENSE_PIN)

#define BAT3V3_FACTOR ( 2.5 * 2 / 4095 )
#define BAT12V_FACTOR ( 2.5 * 6.6 / 4095 )

//------------------------------------------------------------------------------
float p_read_bat3v3(bool debug)
{

uint16_t adc = 0;
float bat3v3 = 0.0;

    /*
     Leo el ADC con 64 muestras
     */

    SET_EN_SENS3V3();
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    SYSTEM_ENTER_CRITICAL();
    
    adc = ADC_read_sens3v3();
    bat3v3 += 1.0 * adc;

    SYSTEM_EXIT_CRITICAL();
    bat3v3 *= BAT3V3_FACTOR;
            
    CLEAR_EN_SENS3V3();
    if(debug) {
        xprintf_P(PSTR("BAT3v3: adc=%d, bat3v3=%0.2f\r\n"), adc, bat3v3);
    }
    return (bat3v3);
}
//------------------------------------------------------------------------------
float p_read_bat12v(bool debug)
{
uint16_t adc = 0;
float bat12v = 0.0;

    /*
     Como acumulo en el ADC 8 samples, el resultado debo dividirlo /8
     */

    SET_EN_SENS12V();
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    SYSTEM_ENTER_CRITICAL();
    
    adc = ADC_read_sens12v();
    bat12v += 1.0 * adc;
    
    SYSTEM_EXIT_CRITICAL();
    // Convierto a voltaje
    bat12v *= BAT12V_FACTOR;
    
    CLEAR_EN_SENS12V();
    if(debug) {
        xprintf_P(PSTR("BAT12v: adc=%d, bat12v=%0.2f\r\n"), adc, bat12v);
    }
    return (bat12v);
}
//------------------------------------------------------------------------------

#endif

#ifdef MODEL_M2


#define TERM_SENSE_PORT         PORTF
#define TERM_SENSE_PIN              5
#define TERM_SENSE_PIN_bm       PIN5_bm
#define TERM_SENSE_PIN_bp       PIN5_bp

#define CONFIG_TERM_SENSE_DIR()     TERM_SENSE_PORT.DIR &= ~TERM_SENSE_PIN_bm;
#define CONFIG_TERM_SENSE_PIN()     TERM_SENSE_PORT.PIN5CTRL |= PORT_PULLUPEN_bm;
#define READ_TERM_SENSE()       ( ( TERM_SENSE_PORT.IN & TERM_SENSE_PIN_bm ) >> TERM_SENSE_PIN)

#define BAT12V_FACTOR ( 2.5 * 6.6 / 4095 )

// -----------------------------------------------------------------------------
float p_read_bat3v3(bool debug)
{

    return (-1.0);
}
//------------------------------------------------------------------------------
float p_read_bat12v(bool debug)
{

    return (-1.0);
}
//------------------------------------------------------------------------------

#endif

//------------------------------------------------------------------------------
void p_config_termsense(void)
{
    /*
     * Configuro el pin TERMSENSE para que sea input, sin pull-up
     */
    cli();
    CONFIG_TERM_SENSE_DIR();
    CONFIG_TERM_SENSE_PIN();
    sei();
    
}
//------------------------------------------------------------------------------
uint8_t p_read_termsense(void)
{
    return  ( READ_TERM_SENSE() );
}
// -----------------------------------------------------------------------------
