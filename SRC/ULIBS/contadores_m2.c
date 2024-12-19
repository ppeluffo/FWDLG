
#include "contadores.h"

#define PE6_INTERRUPT               ( PORTE.INTFLAGS & PIN6_bm )
#define PE6_CLEAR_INTERRUPT_FLAG    ( PORTE.INTFLAGS &= PIN6_bm )

//------------------------------------------------------------------------------
ISR(PORTE_PORT_vect)
{

    // Borro las flags.
    if (PE6_INTERRUPT ) {
        
        if ( contador.fsm_ticks_count == 0) {
            // Arranca el timer que va a hacer el debounced
            xTimerStart(counter_xTimer, 10);   
        }
            
        // Se borra la flag de interrupcion para habilitarla de nuevo
        PE6_CLEAR_INTERRUPT_FLAG;
    }

}
//------------------------------------------------------------------------------

