#include "project.h"

int main(void)
{
    uint8_t RX_ADDR[5] = { 0 };
    
    CyGlobalIntEnable;

    nRF24_Start();
    UART_Start();
    
    UART_PutString("Hola nRF24\r\n");

    nRF24_ReadLongRegister( NRF_RX_ADDR_P0_REG, RX_ADDR, sizeof(RX_ADDR) );
    
    for( uint8_t i = 0; i < sizeof(RX_ADDR); i++)
    {
        UART_PutHexByte( RX_ADDR[i] );
        UART_PutCRLF();
    }
    
    CyDelay(5000);
    CySoftwareReset();
    
    while( 1 )
    {
    }
}

/* [] END OF FILE */
