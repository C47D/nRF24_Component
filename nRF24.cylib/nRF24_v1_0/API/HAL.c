/****************************************************************************
* This file is part of the nRF24 Custom Component for PSoC Devices.
*
* Copyright (C) 2017 Carlos Diaz <carlos.santiago.diaz@gmail.com>
*
* Permission to use, copy, modify, and/or distribute this software for any
* purpose with or without fee is hereby granted, provided that the above
* copyright notice and this permission notice appear in all copies.
*
* THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
* WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
* ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
* WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
* ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
* OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
****************************************************************************/

#include <`$SPI_INTERFACE`.h>
#include <`$SS_PIN`.h>
#include <`$CE_PIN`.h>

#if defined (CY_SCB_`$SPI_INTERFACE`_H)
	#include <`$SPI_INTERFACE`_SPI_UART.h>
#endif

#include "`$INSTANCE_NAME`_HAL.h"

// SPI Functions

uint8_t `$INSTANCE_NAME`_ReadRegister(const NRF_REGISTER_t reg)
{
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData(NRF_R_REGISTER_CMD | reg);
    `$SPI_INTERFACE`_WriteTxData(NRF_NOP_CMD);
    
    while(0 == (`$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_IDLE));
    `$SS_PIN`_Write(1);
    
    (void)`$SPI_INTERFACE`_ReadRxData(); // This is the STATUS Register
    return `$SPI_INTERFACE`_ReadRxData();
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData(NRF_R_REGISTER_CMD | reg);
    `$SPI_INTERFACE`_SpiUartWriteTxData(NRF_NOP_CMD);
    
    while( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != 2 );
    `$SS_PIN`_Write(1);
    
    (void)`$SPI_INTERFACE`_SpiUartReadRxData(); // This is the STATUS Register
    return `$SPI_INTERFACE`_SpiUartReadRxData();
    
#endif
}

void `$INSTANCE_NAME`_ReadLongRegister(const NRF_REGISTER_t reg, uint8_t* data , const size_t dataSize)
{
    if ( NULL == data )
    {
        return;
    }
    
    uint8_t i, j;
    
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
        
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData(NRF_R_REGISTER_CMD | reg);

    for(i = dataSize; i != 0; i--)
    {
        `$SPI_INTERFACE`_WriteTxData(NRF_NOP_CMD);
    }

    `$SS_PIN`_Write(1);
        
    (void)`$SPI_INTERFACE`_ReadRxData(); // Dummy read, this is the STATUS Register
    for(j = 0; j < dataSize; j++) // TODO: Fix this loop
    {
        *(data + j) = `$SPI_INTERFACE`_ReadRxData();
    }
    
    // TODO: Place this right after the SPI WriteTxData
    while(0 == (`$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_IDLE));
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData(NRF_R_REGISTER_CMD | reg);

    for(i = dataSize; i != 0; i--)
    {
        `$SPI_INTERFACE`_SpiUartWriteTxData(NRF_NOP_CMD);
    }
    
    while( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != dataSize );
    `$SS_PIN`_Write(1);

    (void)`$SPI_INTERFACE`_SpiUartReadRxData(); // Dummy read, this is the STATUS Register

    for(j = 0; j < dataSize; j++) // TODO: Fix this loop
    {
        *(data + j) = `$SPI_INTERFACE`_SpiUartReadRxData();
    }
    
#endif
}

void `$INSTANCE_NAME`_WriteRegister(const NRF_REGISTER_t reg, const uint8_t data)
{
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_INTERFACE`_WriteTxData(data);
    
    while(0 == (`$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_IDLE));
    `$SS_PIN`_Write(1);
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_INTERFACE`_SpiUartWriteTxData(data);

    while( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != 2 );
    `$SS_PIN`_Write(1);
    
#endif
}

void `$INSTANCE_NAME`_WriteLongRegister(const NRF_REGISTER_t reg, const uint8_t *data, const size_t dataSize)
{
    if ( NULL == data )
    {
        return;
    }
    
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
        
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_INTERFACE`_PutArray(data, dataSize);

    while(0 == (`$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_IDLE));
    `$SS_PIN`_Write(1);
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);

    `$SPI_INTERFACE`_SpiUartWriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_INTERFACE`_SpiUartPutArray(data, dataSize);

    // Wait for the RxBuffer to have dataSize + 1 bytes,
    // dataSize + 1 because we need to count the Command + Register byte at the
    // beginning of the transaction
    while( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != ( dataSize + 1 ) );
    `$SS_PIN`_Write(1);
    
#endif
}

uint8_t `$INSTANCE_NAME`_ReadBit(const NRF_REGISTER_t reg, uint8_t bit)
{
    return (`$INSTANCE_NAME`_ReadRegister(reg) & (1 << bit)) != 0;
}

void `$INSTANCE_NAME`_WriteBit(const NRF_REGISTER_t reg, const uint8_t bit, const uint8_t value)
{
	uint8_t temp = `$INSTANCE_NAME`_ReadRegister(reg);
    if( value )
    {
        temp |= (1 << bit);
    } else {
        temp &= ~(1 << bit);
    }
    `$INSTANCE_NAME`_WriteRegister(reg, temp);
}

void `$INSTANCE_NAME`_ClearBit(const NRF_REGISTER_t reg, const uint8_t bit)
{
    `$INSTANCE_NAME`_WriteBit(reg, bit, 0);
}

void `$INSTANCE_NAME`_SetBit(const NRF_REGISTER_t reg, const uint8_t bit)
{
    `$INSTANCE_NAME`_WriteBit(reg, bit, 1);
}

/* [] END OF FILE */
