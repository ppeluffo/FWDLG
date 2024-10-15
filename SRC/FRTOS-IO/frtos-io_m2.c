
#include "frtos-io.h"
#include "pines.h"

/*
 * FRTOS-IO_M2:
 * Definicion de funciones para usar en la arquitectura SPX_AVRDA
 *
 */
//------------------------------------------------------------------------------
int16_t frtos_open( file_descriptor_t fd, uint32_t flags)
{
	// Funcion general para abrir el puerto que invoca a una mas
	// especializada para c/periferico.
	// Es la que invoca la aplicacion.
	// Retorna -1 en error o un nro positivo ( fd )

int16_t xRet = -1;

	switch(fd) {
                
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
   
 	default:
		break;
	}

	return(xRet);
}
//------------------------------------------------------------------------------
