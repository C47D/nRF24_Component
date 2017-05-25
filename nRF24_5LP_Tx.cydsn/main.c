#include "project.h"
#include <stdbool.h>

#define PAYLOAD_SIZE 10

volatile uint8_t pressCount;
volatile bool timerFlag;
volatile bool irqFlag;

uint8_t data[PAYLOAD_SIZE] =
    {0x00u, 0x02u, 0x03u, 0x04u, 0x05u,
    0xDEu, 0xADu, 0xBEu, 0xEFu, 0xFFu};
    
uint8_t RXdata[PAYLOAD_SIZE];

CY_ISR_PROTO( IRQ_Handler );
CY_ISR_PROTO( SW_Handler );
CY_ISR_PROTO( Timer_Handler );

int main(void)
{
    const uint8_t RX_ADDR[5] = {0x5E, 0xAF, 0x00, 0xDD, 0xAA};
    const uint8_t TX_ADDR[5] = {0x5E, 0xAF, 0x00, 0xDD, 0xAA};
    
    isr_IRQ_StartEx( IRQ_Handler );
    isr_SW_StartEx( SW_Handler );
    isr_Timer_StartEx( Timer_Handler );
    
    CyGlobalIntEnable;

    nRF24_Start();
    
    nRF24_SetRxAddress(RX_ADDR, sizeof(RX_ADDR));
    nRF24_SetTxAddress(TX_ADDR, sizeof(TX_ADDR));
    nRF24_SetPayloadSize(NRF_DATA_PIPE0, PAYLOAD_SIZE);
    
    UART_Start();
    Timer_Start();

    for(;;)
    {
        if( true == timerFlag ){
            Timer_Stop();
            
            data[0] = pressCount;
            nRF24_TxTransmit(data, sizeof(data));
            timerFlag = false;
            
            Timer_Start();
        }
        
        if( true == irqFlag ){

            if( nRF24_GetStatus() & NRF_STATUS_RX_DR_MASK ){
                RX_STS_Write(~RX_STS_Read());
                do{
                    nRF24_GetRxPayload(RXdata, sizeof(RXdata));
                    nRF24_ClearIRQFlag(NRF_STATUS_RX_DR);
                }while(! ( nRF24_ReadRegister(NRF_FIFO_STATUS_REG) &
                            NRF_STATUS_TX_FIFO_FULL ));
            } else if ( nRF24_GetStatus() & NRF_STATUS_TX_DS_MASK ){
                TX_STS_Write(~TX_STS_Read());
                nRF24_ClearIRQFlag(NRF_STATUS_TX_DS);
            } else if ( nRF24_GetStatus() & NRF_STATUS_MAX_RT_MASK ){
                MAX_Write(~MAX_Read());
                nRF24_ClearIRQFlag(NRF_STATUS_MAX_RT);
            }
            
            irqFlag = false;
        }
    }
}

CY_ISR(IRQ_Handler){
    irqFlag = true;
    IRQ_ClearInterrupt();
}

CY_ISR_PROTO(SW_Handler){
    pressCount++;
    SW_ClearInterrupt();
}

CY_ISR_PROTO(Timer_Handler){
    MAX_Write(~MAX_Read());
    timerFlag = true;
    Timer_ReadStatusRegister();    
}

/* [] END OF FILE */
