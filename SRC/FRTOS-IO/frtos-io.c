/*
 * frtos-io.c
 *
 *  Created on: 11 jul. 2018
 *      Author: pablo
 */

#include "frtos-io.h"
#include "pines.h"

//------------------------------------------------------------------------------
int16_t frtos_open( file_descriptor_t fd, uint32_t flags)
{
	// Funcion general para abrir el puerto que invoca a una mas
	// especializada para c/periferico.
	// Es la que invoca la aplicacion.
	// Retorna -1 en error o un nro positivo ( fd )

int16_t xRet = -1;

	switch(fd) {
   
#ifdef MODEL_M3
	case fdTERM:
        frtos_open_uart(&USART0, flags);
        xRet=0;
		break;
         
    case fdRS485A:
        frtos_open_uart(&USART4, flags);
        xRet=0;
		break;
        
    case fdWAN:
        frtos_open_uart(&USART3, flags);
        xRet=0;
		break;

    case fdI2C0:
		xRet = frtos_open_i2c( &TWI0, &xBusI2C0, fd, &I2C0_xMutexBuffer, flags );
		break;
        
    case fdI2C1:
		xRet = frtos_open_i2c( &TWI1, &xBusI2C1, fd, &I2C1_xMutexBuffer, flags );
		break;
        
    case fdNVM:
		xRet = frtos_open_nvm( &xNVM, fd, &NVM_xMutexBuffer, flags );
		break;

#endif
      
#ifdef MODEL_M2
	case fdTERM:
        frtos_open_uart(&USART2, flags);
        xRet=0;
		break;
         
    case fdRS485A:
        frtos_open_uart(&USART1, flags);
        xRet=0;
		break;
        
    case fdWAN:
        frtos_open_uart(&USART3, flags);
        xRet=0;
		break;

    case fdI2C0:
		xRet = frtos_open_i2c( &TWI0, &xBusI2C0, fd, &I2C0_xMutexBuffer, flags );
		break;
        
    case fdI2C1:
		xRet = frtos_open_i2c( &TWI1, &xBusI2C1, fd, &I2C1_xMutexBuffer, flags );
		break;
        
    case fdNVM:
		xRet = frtos_open_nvm( &xNVM, fd, &NVM_xMutexBuffer, flags );
		break;

#endif
        
	default:
		break;
	}


	return(xRet);
}
//------------------------------------------------------------------------------
int16_t frtos_ioctl( file_descriptor_t fd, uint32_t ulRequest, void *pvValue )
{

int16_t xRet = -1;

	switch(fd) {
   
#ifdef MODEL_M3
        case fdTERM:
            xRet = frtos_ioctl_uart0( ulRequest, pvValue );
            break;

        case fdRS485A:
            xRet = frtos_ioctl_uart4( ulRequest, pvValue );
            break;

        case fdWAN:
            xRet = frtos_ioctl_uart3( ulRequest, pvValue );
            break;
            
        case fdNVM:
            xRet = frtos_ioctl_nvm( &xNVM, ulRequest, pvValue );
            break;

        case fdI2C0:
            xRet = frtos_ioctl_i2c( &TWI0, &xBusI2C0, ulRequest, pvValue );
            break;
            
        case fdI2C1:
            xRet = frtos_ioctl_i2c( &TWI1, &xBusI2C1, ulRequest, pvValue );
            break;
#endif 

#ifdef MODEL_M2
        case fdTERM:
            xRet = frtos_ioctl_uart2( ulRequest, pvValue );
            break;

        case fdRS485A:
            xRet = frtos_ioctl_uart1( ulRequest, pvValue );
            break;

        case fdWAN:
            xRet = frtos_ioctl_uart3( ulRequest, pvValue );
            break;
            
        case fdNVM:
            xRet = frtos_ioctl_nvm( &xNVM, ulRequest, pvValue );
            break;

        case fdI2C0:
            xRet = frtos_ioctl_i2c( &TWI0, &xBusI2C0, ulRequest, pvValue );
            break;
            
        case fdI2C1:
            xRet = frtos_ioctl_i2c( &TWI1, &xBusI2C1, ulRequest, pvValue );
            break;
#endif 

        default:
            break;
	}

	return(xRet);
}
//------------------------------------------------------------------------------
int16_t frtos_write( file_descriptor_t fd ,const char *pvBuffer, const uint16_t xBytes )
{

int16_t xRet = -1;

	switch(fd) {
  
#ifdef MODEL_M3
        
	case fdTERM:
		xRet = frtos_write_uart0( pvBuffer, xBytes );
		break;

    case fdRS485A:
		xRet = frtos_write_uart4_rs485( pvBuffer, xBytes );
		break;
        
    case fdWAN:
		xRet = frtos_write_uart3( pvBuffer, xBytes );
		break;

    case fdI2C0:
		xRet = frtos_write_i2c( &TWI0, &xBusI2C0, pvBuffer, xBytes );
		break;
        
    case fdI2C1:
		xRet = frtos_write_i2c( &TWI1, &xBusI2C1, pvBuffer, xBytes );
		break;
        
    case fdNVM:
		xRet = frtos_write_nvm( &xNVM, pvBuffer, xBytes );
		break;
  
#endif
      
#ifdef MODEL_M2
        
	case fdTERM:
		xRet = frtos_write_uart2( pvBuffer, xBytes );
		break;

    case fdRS485A:
		xRet = frtos_write_uart1_rs485( pvBuffer, xBytes );
		break;
        
    case fdWAN:
		xRet = frtos_write_uart3( pvBuffer, xBytes );
		break;

    case fdI2C0:
		xRet = frtos_write_i2c( &TWI0, &xBusI2C0, pvBuffer, xBytes );
		break;
        
    case fdI2C1:
		xRet = frtos_write_i2c( &TWI1, &xBusI2C1, pvBuffer, xBytes );
		break;
        
    case fdNVM:
		xRet = frtos_write_nvm( &xNVM, pvBuffer, xBytes );
		break;
  
#endif
        
	default:
		break;
	}

	return(xRet);
}
//------------------------------------------------------------------------------
int16_t frtos_read( file_descriptor_t fd , char *pvBuffer, uint16_t xBytes )
{

int16_t xRet = -1;

	switch(fd) {
  
#ifdef MODEL_M3
       
	case fdTERM:
		xRet = frtos_read_uart0( pvBuffer, xBytes );
		break;

    case fdRS485A:
		xRet = frtos_read_uart4( pvBuffer, xBytes );
		break;

    case fdWAN:
		xRet = frtos_read_uart3( pvBuffer, xBytes );
		break;

    case fdI2C0:
		xRet = frtos_read_i2c( &TWI0, &xBusI2C0, pvBuffer, xBytes );
		break;
                
    case fdI2C1:
		xRet = frtos_read_i2c( &TWI1, &xBusI2C1, pvBuffer, xBytes );
		break;
        
    case fdNVM:
		xRet = frtos_read_nvm( &xNVM, pvBuffer, xBytes );
		break;
   
#endif
      
#ifdef MODEL_M2
       
	case fdTERM:
		xRet = frtos_read_uart2( pvBuffer, xBytes );
		break;

    case fdRS485A:
		xRet = frtos_read_uart1( pvBuffer, xBytes );
		break;

    case fdWAN:
		xRet = frtos_read_uart3( pvBuffer, xBytes );
		break;

    case fdI2C0:
		xRet = frtos_read_i2c( &TWI0, &xBusI2C0, pvBuffer, xBytes );
		break;
                
    case fdI2C1:
		xRet = frtos_read_i2c( &TWI1, &xBusI2C1, pvBuffer, xBytes );
		break;
        
    case fdNVM:
		xRet = frtos_read_nvm( &xNVM, pvBuffer, xBytes );
		break;
   
#endif
        
 	default:
		break;
	}

	return(xRet);
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// FUNCIONES ESPECIFICAS DE UART's
//------------------------------------------------------------------------------
void frtos_open_uart( volatile USART_t *uart, uint32_t baudrate)
{
   
    if (uart == &USART0 ) {
        drv_uart0_init(baudrate);
        
    } else if (uart == &USART1 ) {
        drv_uart1_init(baudrate);
        
    } else if (uart == &USART2 ) {
        drv_uart2_init(baudrate);
        
    } else if (uart == &USART3 ) {
        drv_uart3_init(baudrate);
        
    } else if (uart == &USART4 ) {
        drv_uart4_init(baudrate);
    }

}
//------------------------------------------------------------------------------
int16_t frtos_write_uart0( const char *pvBuffer, const uint16_t xBytes )
{
    
uint16_t i;
    
    // Transmision x poleo ( No hablito al INT x DRIE )
    //for( i = 0; i < strlen(pvBuffer); i++) {
    for( i = 0; i < xBytes; i++) {
        // Espero que el TXDATA reg. este vacio.
        while( (USART0.STATUS & USART_DREIF_bm) == 0 )
            ;
        USART_PutChar(&USART0, pvBuffer[i]);
    }
    // Espero que salga el ultimo byte
    while( ( USART0.STATUS &  USART_TXCIF_bm) == 0 )
            ;
    // Borro la flag
    USART0.STATUS |= ( 1 << USART_TXCIF_bp);
    
    vTaskDelay( ( TickType_t)( 1 ) );
    return(xBytes);   
}
//------------------------------------------------------------------------------
int16_t frtos_write_uart1( const char *pvBuffer, const uint16_t xBytes )
{
    
uint16_t i;
    
    // Transmision x poleo ( No hablito al INT x DRIE )
    //for( i = 0; i < strlen(pvBuffer); i++) {
    for( i = 0; i < xBytes; i++) {
        // Espero que el TXDATA reg. este vacio.
        while( (USART1.STATUS & USART_DREIF_bm) == 0 )
            ;
        USART_PutChar(&USART1, pvBuffer[i]);
    }

    // Espero que salga el ultimo byte
    while( ( USART1.STATUS &  USART_TXCIF_bm) == 0 )
            ;
    // Borro la flag
    USART1.STATUS |= ( 1 << USART_TXCIF_bp);
    
    vTaskDelay( ( TickType_t)( 1 ) );
    return(xBytes);   
}
//------------------------------------------------------------------------------
int16_t frtos_write_uart2( const char *pvBuffer, const uint16_t xBytes )
{
    
uint16_t i;
    
    // Transmision x poleo ( No hablito al INT x DRIE )
    //for( i = 0; i < strlen(pvBuffer); i++) {
    for( i = 0; i < xBytes; i++) {
        // Espero que el TXDATA reg. este vacio.
        while( (USART2.STATUS & USART_DREIF_bm) == 0 )
            ;
        USART_PutChar(&USART2, pvBuffer[i]);
    }

    // Espero que salga el ultimo byte
    while( ( USART2.STATUS &  USART_TXCIF_bm) == 0 )
            ;
    // Borro la flag
    USART2.STATUS |= ( 1 << USART_TXCIF_bp);
    vTaskDelay( ( TickType_t)( 1 ) );
    return(xBytes);   
}
//------------------------------------------------------------------------------
int16_t frtos_write_uart3( const char *pvBuffer, const uint16_t xBytes )
{
    
uint16_t i;
    
    // Transmision x poleo ( No hablito al INT x DRIE )
    //for( i = 0; i < strlen(pvBuffer); i++) {
    for( i = 0; i < xBytes; i++) {
        while( (USART3.STATUS & USART_DREIF_bm) == 0 )
            ;
        USART_PutChar(&USART3, pvBuffer[i]);
    }
    // Espero que salga el ultimo byte
    while( ( USART3.STATUS &  USART_TXCIF_bm) == 0 )
            ;
    // Borro la flag
    USART3.STATUS |= ( 1 << USART_TXCIF_bp);
    vTaskDelay( ( TickType_t)( 1 ) );
    return(xBytes);   
}
//------------------------------------------------------------------------------
int16_t frtos_write_uart4( const char *pvBuffer, const uint16_t xBytes )
{
    
uint16_t i;
    
    // Transmision x poleo ( No hablito al INT x DRIE )
    //for( i = 0; i < strlen(pvBuffer); i++) {
    for( i = 0; i < xBytes; i++) {
        while( (USART4.STATUS & USART_DREIF_bm) == 0 )
            ;
        USART_PutChar(&USART4, pvBuffer[i]);
    }
    // Espero que salga el ultimo byte
    while( ( USART4.STATUS &  USART_TXCIF_bm) == 0 )
            ;
    // Borro la flag
    USART4.STATUS |= ( 1 << USART_TXCIF_bp);
    vTaskDelay( ( TickType_t)( 1 ) );
    return(xBytes);   
}
//------------------------------------------------------------------------------
int16_t frtos_write_uart1_rs485( const char *pvBuffer, const uint16_t xBytes )
{
    
uint16_t i;
    
    // RTS ON. Habilita el sentido de trasmision del chip.
	SET_RTS_RS485();
	vTaskDelay( ( TickType_t)( 5 ) );
    
    // Transmision x poleo ( No hablito al INT x DRIE )
    for( i = 0; i < xBytes; i++) {
        // Espero que el TXDATA reg. este vacio.
        while( (USART1.STATUS & USART_DREIF_bm) == 0 )
            ;
        USART_PutChar(&USART1, pvBuffer[i]);
    }
    
    // Espero que salga el ultimo byte
    while( ( USART1.STATUS &  USART_TXCIF_bm) == 0 )
            ;
    // Borro la flag
    USART1.STATUS |= ( 1 << USART_TXCIF_bp);
    
    // Para evitar el loopback del puerto RS485
    frtos_ioctl( fdRS485A, ioctl_UART_CLEAR_RX_BUFFER, NULL );
    vTaskDelay( ( TickType_t)( 2 ) );
    
	// RTS OFF: Habilita la recepcion del chip
	CLEAR_RTS_RS485();
    vTaskDelay( ( TickType_t)( 1 ) );
    return(xBytes);   
}
//------------------------------------------------------------------------------
int16_t frtos_write_uart4_rs485( const char *pvBuffer, const uint16_t xBytes )
{
    
uint16_t i;
    
    // RTS ON. Habilita el sentido de trasmision del chip.
	SET_RTS_RS485();
	vTaskDelay( ( TickType_t)( 5 ) );
    
    // Transmision x poleo ( No hablito al INT x DRIE )
    for( i = 0; i < xBytes; i++) {
        // Espero que el TXDATA reg. este vacio.
        while( (USART4.STATUS & USART_DREIF_bm) == 0 )
            ;
        USART_PutChar(&USART4, pvBuffer[i]);
    }
    
    // Espero que salga el ultimo byte
    while( ( USART4.STATUS &  USART_TXCIF_bm) == 0 )
            ;
    // Borro la flag
    USART4.STATUS |= ( 1 << USART_TXCIF_bp);
    
    // Para evitar el loopback del puerto RS485
    frtos_ioctl( fdRS485A, ioctl_UART_CLEAR_RX_BUFFER, NULL );
    vTaskDelay( ( TickType_t)( 2 ) );
    
	// RTS OFF: Habilita la recepcion del chip
	CLEAR_RTS_RS485();
    vTaskDelay( ( TickType_t)( 1 ) );
    return(xBytes);   
}
//------------------------------------------------------------------------------
int16_t frtos_ioctl_uart0( uint32_t ulRequest, void *pvValue )
{

int16_t xReturn = 0;

	switch( ulRequest )
	{
		case ioctl_UART_CLEAR_TX_BUFFER:
			rBchar_Flush(&TXRB_uart0);
			break;
            
		case ioctl_UART_CLEAR_RX_BUFFER:
			rBchar_Flush(&RXRB_uart0);
			break;
            
		default :
			xReturn = -1;
			break;
	}

	return xReturn;

}
//------------------------------------------------------------------------------
int16_t frtos_ioctl_uart1( uint32_t ulRequest, void *pvValue )
{

int16_t xReturn = 0;

	switch( ulRequest )
	{
		case ioctl_UART_CLEAR_TX_BUFFER:
			rBchar_Flush(&TXRB_uart1);
			break;

        case ioctl_UART_CLEAR_RX_BUFFER:
			rBchar_Flush(&RXRB_uart1);
			break;
            
		default :
			xReturn = -1;
			break;
	}

	return xReturn;

}
//------------------------------------------------------------------------------
int16_t frtos_ioctl_uart2( uint32_t ulRequest, void *pvValue )
{

int16_t xReturn = 0;

	switch( ulRequest )
	{
		case ioctl_UART_CLEAR_TX_BUFFER:
			rBchar_Flush(&TXRB_uart2);
			break;
            
        case ioctl_UART_CLEAR_RX_BUFFER:
			rBchar_Flush(&RXRB_uart2);
			break;

		default :
			xReturn = -1;
			break;
	}

	return xReturn;

}
//------------------------------------------------------------------------------
int16_t frtos_ioctl_uart3( uint32_t ulRequest, void *pvValue )
{

int16_t xReturn = 0;

	switch( ulRequest )
	{
		case ioctl_UART_CLEAR_TX_BUFFER:
			rBchar_Flush(&TXRB_uart3);
			break;

        case ioctl_UART_CLEAR_RX_BUFFER:
			rBchar_Flush(&RXRB_uart3);
			break;
            
		default :
			xReturn = -1;
			break;
	}

	return xReturn;

}
//------------------------------------------------------------------------------
int16_t frtos_ioctl_uart4( uint32_t ulRequest, void *pvValue )
{

int16_t xReturn = 0;

	switch( ulRequest )
	{
		case ioctl_UART_CLEAR_TX_BUFFER:
			rBchar_Flush(&TXRB_uart4);
			break;

        case ioctl_UART_CLEAR_RX_BUFFER:
			rBchar_Flush(&RXRB_uart4);
			break;
            
		default :
			xReturn = -1;
			break;
	}

	return xReturn;

}
//------------------------------------------------------------------------------
int16_t frtos_read_uart0( char *pvBuffer, uint16_t xBytes )
{
	// Lee caracteres de la cola de recepcion y los deja en el buffer.
	// El timeout lo fijo con ioctl.

int16_t xBytesReceived = 0U;
TickType_t xTicksToWait = 10;
TimeOut_t xTimeOut;

     /* Initialize xTimeOut.  This records the time at which this function was
        entered. 
      */
	vTaskSetTimeOutState( &xTimeOut );

	// Are there any more bytes to be received?
	while( xBytesReceived < xBytes )
	{

        if( rBchar_Pop( &RXRB_uart0, &((char *)pvBuffer)[ xBytesReceived ] ) == true ) {
			xBytesReceived++;
            /*
             Recibi un byte. Re-inicio el timeout.
             */
            vTaskSetTimeOutState( &xTimeOut );
			//taskYIELD();
            //vTaskDelay( ( TickType_t)( 1 ) );
		} else {
			// Espero xTicksToWait antes de volver a chequear
			vTaskDelay( ( TickType_t)( 1 ) );

            // Time out has expired ?
            if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) != pdFALSE )
            {
                break;
            }
        }

    }

	return ( xBytesReceived );

}
//------------------------------------------------------------------------------
int16_t frtos_read_uart1( char *pvBuffer, uint16_t xBytes )
{
	// Lee caracteres de la cola de recepcion y los deja en el buffer.
	// El timeout lo fijo con ioctl.

int16_t xBytesReceived = 0U;
TickType_t xTicksToWait = 10;
TimeOut_t xTimeOut;

     /* Initialize xTimeOut.  This records the time at which this function was
        entered. 
      */
	vTaskSetTimeOutState( &xTimeOut );

	// Are there any more bytes to be received?
	while( xBytesReceived < xBytes )
	{

        if( rBchar_Pop( &RXRB_uart1, &((char *)pvBuffer)[ xBytesReceived ] ) == true ) {
			xBytesReceived++;
            /*
             Recibi un byte. Re-inicio el timeout.
             */
            vTaskSetTimeOutState( &xTimeOut );
			//taskYIELD();
            //vTaskDelay( ( TickType_t)( 1 ) );
		} else {
			// Espero xTicksToWait antes de volver a chequear
			vTaskDelay( ( TickType_t)( 1 ) );

            // Time out has expired ?
            if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) != pdFALSE )
            {
                break;
            }
        }

    }

	return ( xBytesReceived );

}
//------------------------------------------------------------------------------
int16_t frtos_read_uart2( char *pvBuffer, uint16_t xBytes )
{
	// Lee caracteres de la cola de recepcion y los deja en el buffer.
	// El timeout lo fijo con ioctl.

int16_t xBytesReceived = 0U;
TickType_t xTicksToWait = 10;
TimeOut_t xTimeOut;

     /* Initialize xTimeOut.  This records the time at which this function was
        entered. 
      */
	vTaskSetTimeOutState( &xTimeOut );

	// Are there any more bytes to be received?
	while( xBytesReceived < xBytes )
	{

        if( rBchar_Pop( &RXRB_uart2, &((char *)pvBuffer)[ xBytesReceived ] ) == true ) {
			xBytesReceived++;
            /*
             Recibi un byte. Re-inicio el timeout.
             */
            vTaskSetTimeOutState( &xTimeOut );
			//taskYIELD();
            //vTaskDelay( ( TickType_t)( 1 ) );
		} else {
			// Espero xTicksToWait antes de volver a chequear
			vTaskDelay( ( TickType_t)( 1 ) );

            // Time out has expired ?
            if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) != pdFALSE )
            {
                break;
            }
        }

    }

	return ( xBytesReceived );

}
//------------------------------------------------------------------------------
int16_t frtos_read_uart3( char *pvBuffer, uint16_t xBytes )
{
	// Lee caracteres de la cola de recepcion y los deja en el buffer.
	// El timeout lo fijo con ioctl.

int16_t xBytesReceived = 0U;
TickType_t xTicksToWait = 10;
TimeOut_t xTimeOut;

     /* Initialize xTimeOut.  This records the time at which this function was
        entered. 
      */
	vTaskSetTimeOutState( &xTimeOut );

	// Are there any more bytes to be received?
	while( xBytesReceived < xBytes )
	{

        if( rBchar_Pop( &RXRB_uart3, &((char *)pvBuffer)[ xBytesReceived ] ) == true ) {
			xBytesReceived++;
            /*
             Recibi un byte. Re-inicio el timeout.
             */
            vTaskSetTimeOutState( &xTimeOut );
			//taskYIELD();
            //vTaskDelay( ( TickType_t)( 1 ) );
		} else {
			// Espero xTicksToWait antes de volver a chequear
			vTaskDelay( ( TickType_t)( 1 ) );

            // Time out has expired ?
            if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) != pdFALSE )
            {
                break;
            }
        }

    }

	return ( xBytesReceived );

}
//------------------------------------------------------------------------------
int16_t frtos_read_uart4( char *pvBuffer, uint16_t xBytes )
{
	// Lee caracteres de la cola de recepcion y los deja en el buffer.
	// El timeout lo fijo con ioctl.

int16_t xBytesReceived = 0U;
TickType_t xTicksToWait = 10;
TimeOut_t xTimeOut;

     /* Initialize xTimeOut.  This records the time at which this function was
        entered. 
      */
	vTaskSetTimeOutState( &xTimeOut );

	// Are there any more bytes to be received?
	while( xBytesReceived < xBytes )
	{

        if( rBchar_Pop( &RXRB_uart4, &((char *)pvBuffer)[ xBytesReceived ] ) == true ) {
			xBytesReceived++;
            /*
             Recibi un byte. Re-inicio el timeout.
             */
            vTaskSetTimeOutState( &xTimeOut );
			//taskYIELD();
            //vTaskDelay( ( TickType_t)( 1 ) );
		} else {
			// Espero xTicksToWait antes de volver a chequear
			vTaskDelay( ( TickType_t)( 1 ) );

            // Time out has expired ?
            if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) != pdFALSE )
            {
                break;
            }
        }

    }

	return ( xBytesReceived );

}
//------------------------------------------------------------------------------
// FUNCIONES ESPECIFICAS DE NVM
//------------------------------------------------------------------------------
int16_t frtos_open_nvm( periferico_nvm_t *xNVM, file_descriptor_t fd, StaticSemaphore_t *nvm_semph, uint32_t flags)
{
	// Asigno las funciones particulares ed write,read,ioctl
	xNVM->fd = fd;
	xNVM->xBusSemaphore = xSemaphoreCreateMutexStatic( nvm_semph );
	xNVM->xBlockTime = (10 / portTICK_PERIOD_MS );
	//
	return(1);
}
//------------------------------------------------------------------------------
int16_t frtos_ioctl_nvm( periferico_nvm_t *xNVM, uint32_t ulRequest, void *pvValue )
{

int16_t xReturn = 0;
    
uint32_t *p = NULL;

	p = pvValue;

	switch( ulRequest )
	{
		case ioctl_OBTAIN_BUS_SEMPH:
			// Espero el semaforo en forma persistente.
			while ( xSemaphoreTake(xNVM->xBusSemaphore, ( TickType_t ) 5 ) != pdTRUE )
				taskYIELD();
			break;
			case ioctl_RELEASE_BUS_SEMPH:
				xSemaphoreGive( xNVM->xBusSemaphore );
				break;
			case ioctl_SET_TIMEOUT:
				xNVM->xBlockTime = *p;
				break;
			case ioctl_NVM_SET_EEADDRESS:
				xNVM->eeAddress = *p;
				break;                
			default :
				xReturn = -1;
				break;
		}

	return xReturn;

}
//------------------------------------------------------------------------------
int16_t frtos_read_nvm( periferico_nvm_t *xNVM, char *pvBuffer, const uint16_t xBytes )
{

int16_t xReturn = xBytes;

    FLASH_0_read_eeprom_block( xNVM->eeAddress, (uint8_t *)pvBuffer, xBytes );
    return(xReturn);
}
//------------------------------------------------------------------------------
int16_t frtos_write_nvm( periferico_nvm_t *xNVM, const char *pvBuffer, const uint16_t xBytes )
{

nvmctrl_status_t nvm_status;

    nvm_status = FLASH_0_write_eeprom_block( xNVM->eeAddress, (uint8_t *)pvBuffer, xBytes);
    if ( nvm_status == NVM_OK) {
        return(xBytes);
    }
    return(-1);

}
//------------------------------------------------------------------------------
// FUNCIONES ESPECIFICAS DEL BUS I2C/TWI
//------------------------------------------------------------------------------
int16_t frtos_open_i2c( volatile TWI_t *twi, periferico_i2c_port_t *xI2c, file_descriptor_t fd, StaticSemaphore_t *i2c_semph, uint32_t flags)
{
	// Asigno las funciones particulares ed write,read,ioctl
	xI2c->fd = fd;
	xI2c->xBusSemaphore = xSemaphoreCreateMutexStatic( i2c_semph );
	xI2c->xBlockTime = (10 / portTICK_PERIOD_MS );
	xI2c->i2c_error_code = I2C_OK;
	//
	// Abro e inicializo el puerto I2C solo la primera vez que soy invocado
	drv_I2C_init(twi, false);
	return(1);
}
//------------------------------------------------------------------------------
int16_t frtos_ioctl_i2c( volatile TWI_t *twi, periferico_i2c_port_t *xI2c, uint32_t ulRequest, void *pvValue )
{

int16_t xReturn = 0;
    
uint32_t *p = NULL;

	p = pvValue;

	switch( ulRequest )
	{
		case ioctl_OBTAIN_BUS_SEMPH:
			// Espero el semaforo en forma persistente.
			while ( xSemaphoreTake(xI2c->xBusSemaphore, ( TickType_t ) 5 ) != pdTRUE )
				taskYIELD();
			break;
			case ioctl_RELEASE_BUS_SEMPH:
				xSemaphoreGive( xI2c->xBusSemaphore );
				break;
			case ioctl_SET_TIMEOUT:
				xI2c->xBlockTime = *p;
				break;
			case ioctl_I2C_SET_DEVADDRESS:
				xI2c->devAddress = (int8_t)(*p);
				break;
			case ioctl_I2C_SET_DATAADDRESS:
				xI2c->dataAddress = (uint16_t)(*p);
				break;
			case ioctl_I2C_SET_DATAADDRESSLENGTH:
				xI2c->dataAddress_length = (int8_t)(*p);
				break;
			case ioctl_I2C_GET_LAST_ERROR:
				xReturn = xI2c->i2c_error_code;
				break;
            case ioctl_I2C_SET_DEBUG:
                xprintf_P(PSTR("frtos_ioctl_i2c: DEBUG_ON\r\n"));
				xI2c->debug_flag = true;
				break;
            case ioctl_I2C_CLEAR_DEBUG:
				xI2c->debug_flag = false;
				break;
            case ioctl_I2C_RESET:
				drv_I2C_reset( twi, xI2c->debug_flag );
				break;                
			default :
				xReturn = -1;
				break;
		}

	return xReturn;

}
//------------------------------------------------------------------------------
int16_t frtos_read_i2c( volatile TWI_t *twi, periferico_i2c_port_t *xI2c, char *pvBuffer, const uint16_t xBytes )
{

int16_t xReturn = 0U;

    if ( xI2c->debug_flag) {
        xprintf_P(PSTR("frtos_read_i2c: DEBUG_ON\r\n"));
    }

	if ( ( xReturn = drv_I2C_master_read( twi, xI2c->devAddress, xI2c->dataAddress, xI2c->dataAddress_length, (char *)pvBuffer, xBytes, xI2c->debug_flag)) > 0 ) {
		xI2c->i2c_error_code = I2C_OK;
	} else {
		// Error de escritura indicado por el driver.
		xI2c->i2c_error_code = I2C_RD_ERROR;
	}
    return(xReturn);
}
//------------------------------------------------------------------------------
int16_t frtos_write_i2c( volatile TWI_t *twi, periferico_i2c_port_t *xI2c, const char *pvBuffer, const uint16_t xBytes )
{

int16_t xReturn = 0U;

    if ( xI2c->debug_flag) {
        xprintf_P(PSTR("frtos_write_i2c DEBUG ON\r\n"));
    }

	if ( ( xReturn = drv_I2C_master_write( twi, xI2c->devAddress, xI2c->dataAddress, xI2c->dataAddress_length, (char *)pvBuffer, xBytes, xI2c->debug_flag) ) > 0 ) {
		xI2c->i2c_error_code = I2C_OK;
	} else {
		// Error de escritura indicado por el driver.
		xI2c->i2c_error_code = I2C_WR_ERROR;
	}

	return(xReturn);

}
//------------------------------------------------------------------------------
/*
int16_t frtos_write_uart4_modbus( const char *pvBuffer, const uint16_t xBytes )
{
    // Hago control de flujo ya que el SP3485 debe operarse HALF-DUPLEX !!
	// Trasmite el buffer sin considerar si tiene NULL 0x00 en el medio.
	// Transmite en forma transparente los xBytes por poleo de modo que controlo exactamente
	// cuando termino de transmitir c/byte.
	//

char cChar = '\0';
char *p = NULL;
int16_t wBytes = 0;
uint16_t i;

	// RTS ON. Habilita el sentido de trasmision del chip.
	SET_RTS_RS485();
	vTaskDelay( ( TickType_t)( 5 ) );
    p = (char *)pvBuffer;  
    
    // Transmision x poleo ( No hablito al INT x DRIE )
    taskENTER_CRITICAL();
       
    for( i = 0; i < xBytes; i++) {
        
        // Espero que el TXDATA reg. este vacio.
        while( (USART4.STATUS & USART_DREIF_bm) == 0 )
            ;
        
        // Voy cargando la cola de a uno.
		cChar = *p;
		// Delay inter chars.(Shinco, Taosonic = 2)
		//vTaskDelay( ( TickType_t)( 2 ) );
        //_delay_us (1750);
        USART_PutChar(&USART4, cChar );
		p++;
		wBytes++;	// Cuento los bytes que voy trasmitiendo

    }
    // Espero que salga el ultimo byte
    while( ( USART4.STATUS &  USART_TXCIF_bm) == 0 )
            ;
    // Borro la flag
    USART4.STATUS |= ( 1 << USART_TXCIF_bp);
    
    vTaskDelay( ( TickType_t)( 1 ) );
    
    taskEXIT_CRITICAL();
    // Para evitar el loopback del puerto RS485
    frtos_ioctl( fdRS485_MODBUS, ioctl_UART_CLEAR_RX_BUFFER, NULL );
    //vTaskDelay( ( TickType_t)( 2 ) );
	// RTS OFF: Habilita la recepcion del chip
	CLEAR_RTS_RS485();
    
	return (wBytes);

}
 */
//-------------------------------------------------------------------------------
