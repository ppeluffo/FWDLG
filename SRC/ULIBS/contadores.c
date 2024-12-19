
#include "contadores.h"
#include <avr/interrupt.h>

#define PF4_INTERRUPT               ( PORTF.INTFLAGS & PIN4_bm )
#define PF4_CLEAR_INTERRUPT_FLAG    ( PORTF.INTFLAGS &= PIN4_bm )

#define PE6_INTERRUPT               ( PORTE.INTFLAGS & PIN6_bm )
#define PE6_CLEAR_INTERRUPT_FLAG    ( PORTE.INTFLAGS &= PIN6_bm )

#ifdef MODEL_M3

    #define CNT0_PORT	PORTF
    #define CNT0_PIN    4   
    #define CNT0_PIN_bm	PIN4_bm
    #define CNT0_PIN_bp	PIN4_bp

    #define CNT0_PIN_CONFIG()           PORTF.PIN4CTRL |= PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;

#endif

#ifdef MODEL_M2

    #define CNT0_PORT	PORTE
    #define CNT0_PIN    6   
    #define CNT0_PIN_bm	PIN6_bm
    #define CNT0_PIN_bp	PIN6_bp

    #define CNT0_PIN_CONFIG()           PORTE.PIN6CTRL |= PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;

#endif

static bool f_debug_counters = false;

void pv_count_pulse(void);

TaskHandle_t *l_xHandle_tkCtl;

//------------------------------------------------------------------------------
void counter_init_outofrtos( TaskHandle_t *xHandle )
{
    /*
     * El pin del contador es INPUT, PULLOPEN e interrumpe en flanco de bajada.
     * 
     * Utilizo el handler de tkCtl para mostrar el debug.
     * 
     */
    
    l_xHandle_tkCtl = xHandle;
    // Los CNTx son inputs
    CNT0_PORT.DIR &= ~CNT0_PIN_bm;
    // Habilito a interrumpir, pullup, flanco de bajada.
    cli();
    CNT0_PIN_CONFIG();
    //PORTF.PIN4CTRL |= PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
    sei();
}
// ----------------------------------------------------------------------------- 
void counter_init( void )
{
    contador.fsm_ticks_count = 0;
}
// ----------------------------------------------------------------------------- 
void counter_config_defaults( void )
{
    /*
     * Realiza la configuracion por defecto de los canales digitales.
     */

    //strncpy( counter_conf.name, "X", CNT_PARAMNAME_LENGTH );
    strlcpy( counter_conf.name, "X", CNT_PARAMNAME_LENGTH );
    counter_conf.enabled = false;
    counter_conf.magpp = 1;
    counter_conf.modo_medida = CAUDAL;

}
//------------------------------------------------------------------------------
void counter_print_configuration( void )
{
    /*
     * Muestra la configuracion de todos los canales de contadores en la terminal
     * La usa el comando tkCmd::status.
     */
    

    xprintf_P(PSTR("Counter:\r\n"));
    xprintf_P(PSTR(" debug: "));
    f_debug_counters ? xprintf_P(PSTR("on\r\n")) : xprintf_P(PSTR("off\r\n"));
    
        
    if ( counter_conf.enabled ) {
        xprintf_P( PSTR(" c0: +"));
    } else {
        xprintf_P( PSTR(" c0: -"));
    }
                
    xprintf_P( PSTR("[%s,magpp=%.03f,"), counter_conf.name, counter_conf.magpp );
    if ( counter_conf.modo_medida == CAUDAL ) {
        xprintf_P(PSTR("CAUDAL]\r\n"));
    } else {
        xprintf_P(PSTR("PULSO]\r\n"));
    }
      
}
//------------------------------------------------------------------------------
bool counter_config_channel( char *s_enable, char *s_name, char *s_magpp, char *s_modo )
{
	// Configuro un canal contador.
	// channel: id del canal
	// s_param0: string del nombre del canal
	// s_param1: string con el valor del factor magpp.
	//
	// {0..1} dname magPP

bool retS = false;

    //xprintf_P(PSTR("DEBUG COUNTERS: en=%s,name=%s,magpp=%s,modo=%s,rbsize=%s\r\n"), s_enable,s_name,s_magpp,s_modo,s_rb_size  );

	if ( s_name == NULL ) {
		return(retS);
	}

    // Enable ?
    if (!strcmp_P( strupr(s_enable), PSTR("TRUE"))  ) {
        counter_conf.enabled = true;
    } else if (!strcmp_P( strupr(s_enable), PSTR("FALSE"))  ) {
        counter_conf.enabled = false;
    }
        
    // NOMBRE
	//snprintf_P( cnt->channel[ch].name, CNT_PARAMNAME_LENGTH, PSTR("%s"), s_name );
    //strncpy( counter_conf.name, s_name, CNT_PARAMNAME_LENGTH );
    strlcpy( counter_conf.name, s_name, CNT_PARAMNAME_LENGTH );

	// MAGPP
	if ( s_magpp != NULL ) { counter_conf.magpp = atof(s_magpp); }

    // MODO ( PULSO/CAUDAL )
    if ( s_modo != NULL ) {
		if ( strcmp_P( strupr(s_modo), PSTR("PULSO")) == 0 ) {
			counter_conf.modo_medida = PULSOS;

		} else if ( strcmp_P( strupr(s_modo) , PSTR("CAUDAL")) == 0 ) {
			counter_conf.modo_medida = CAUDAL;

		} else {
			//xprintf_P(PSTR("ERROR: counters modo: PULSO/CAUDAL only!!\r\n"));
            return (false);
		}
    }
        
    retS = true;
	return(retS);

}
//------------------------------------------------------------------------------
void counter_config_debug(bool debug )
{
    if ( debug ) {
        f_debug_counters = true;
    } else {
        f_debug_counters = false;
    }
}
//------------------------------------------------------------------------------
bool counter_read_debug(void)
{
    return (f_debug_counters);
}
//------------------------------------------------------------------------------
uint8_t counter_read_pin(void)
{
    return ( ( CNT0_PORT.IN & CNT0_PIN_bm ) >> CNT0_PIN) ;
}
// -----------------------------------------------------------------------------
counter_value_t counter_read(void)
{
    return(contador);
}
// -----------------------------------------------------------------------------
void counter_clear(void)
{
   /*
    * Una vez por periodo ( timerpoll ) borro los contadores.
    * Si en el periodo NO llegaron pulsos, aqui debo entonces en los
    * caudales agregar un 0.0 al ring buffer para que luego de un tiempo
    * converja a 0.
    * 
    */
    contador.caudal = 0.0;
    contador.pulsos = 0;
}
//------------------------------------------------------------------------------
uint8_t counter_hash(void)
{
    
uint8_t j;
uint8_t hash = 0;
uint8_t l_hash_buffer[64];
char *p;

    // Calculo el hash de la configuracion de los contadores


    memset( l_hash_buffer, '\0', sizeof(l_hash_buffer));
    j = sprintf_P( (char *)&l_hash_buffer, PSTR("[C0:") );
    
    if ( counter_conf.enabled ) {
        j += sprintf_P( (char *)&l_hash_buffer[j], PSTR("TRUE,") );
    } else {
       j += sprintf_P( (char *)&l_hash_buffer[j], PSTR("FALSE,") );
    }
        
    j += sprintf_P( (char *)&l_hash_buffer[j], PSTR("%s,"), counter_conf.name );
    j += sprintf_P( (char *)&l_hash_buffer[j], PSTR("%.03f,"), counter_conf.magpp );
        
    if ( counter_conf.modo_medida == 0 ) {
        j += sprintf_P( (char *)&l_hash_buffer[j], PSTR("CAUDAL]"));
    } else {
        j += sprintf_P( (char *)&l_hash_buffer[j], PSTR("PULSOS]"));
    }
    
    p = (char *)l_hash_buffer;
    while (*p != '\0') {
        hash = u_hash(hash, *p++);
    }
        
    //xprintf_P(PSTR("HASH_CNT:<%s>, hash=%d(0x%02x)\r\n"), l_hash_buffer, hash, hash );
 
    return(hash);
    
}
//------------------------------------------------------------------------------
void pv_count_pulse(void)
{
  
BaseType_t xHigherPriorityTaskWoken = pdTRUE;

    contador.now_ticks = xTaskGetTickCountFromISR();
    contador.T_ticks = contador.now_ticks - contador.start_pulse_ticks;
    // Guardo el inicio del pulso para medir el caudal del proximo pulso
    contador.start_pulse_ticks = contador.now_ticks;
    
    // No cuento pulsos menores de 100 ticks de ancho
    if (contador.T_ticks < 10) {
        return;
    }
    
    contador.pulsos++;
    contador.T_secs =  (float)(1.0 * contador.T_ticks);    // Duracion en ticks
    contador.T_secs /= configTICK_RATE_HZ;                 // Duracion en secs.

    if ( contador.T_secs > 0 ) {
        contador.caudal = counter_conf.magpp / contador.T_secs;      // En mt3/s 
        contador.caudal *= 3600;                                     // En mt3/h
    } else {
        contador.caudal = 0.0;
    } 
            
    if (f_debug_counters) {
        xTaskNotifyFromISR( *l_xHandle_tkCtl,
                       0x01,
                       eSetBits,
                       &xHigherPriorityTaskWoken );
        
    }
            
}
//------------------------------------------------------------------------------
/*
 * Note: If the flag is not cleared, the interrupt will keep triggering, so 
 * it is essential that the flag is always cleared before exiting the ISR. 
 * Any algorithm that relies on not clearing the interrupt flag is highly 
 * discouraged because this effectively overloads the ISR responsibility. 
 * The resulting responsibilities of the ISR will be to handle the interrupt 
 * and to keep firing until the flag is cleared. 
 * This violates the Single Responsibility principle in software design 
 * and extreme care must be taken to avoid bugs.
 */
#ifdef MODEL_M3

ISR(PORTF_PORT_vect)
{
    // Borro las flags.
    if (PF4_INTERRUPT) {
        //led_toggle();
        pv_count_pulse();
                    
        // Se borra la flag de interrupcion para habilitarla de nuevo
        // Si no la borro antes de salir, se vuelve a generar la int.
        PF4_CLEAR_INTERRUPT_FLAG;
    }

}
#endif
//------------------------------------------------------------------------------
#ifdef MODEL_M2

ISR(PORTE_PORT_vect)
{
    // Borro las flags.
    if (PE6_INTERRUPT) {
        //led_toggle();
        pv_count_pulse();
                    
        // Se borra la flag de interrupcion para habilitarla de nuevo
        // Si no la borro antes de salir, se vuelve a generar la int.
        PE6_CLEAR_INTERRUPT_FLAG;
    }

}
#endif
//------------------------------------------------------------------------------
     