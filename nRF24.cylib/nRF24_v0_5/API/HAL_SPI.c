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

#include "`$INSTANCE_NAME`_HAL_SPI.h"

uint8_t `$INSTANCE_NAME`_ReadRegister(const NRF_REGISTER_t reg)
{
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData( NRF_R_REGISTER_CMD | reg );
    `$SPI_INTERFACE`_WriteTxData( NRF_NOP_CMD );
    
    while( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
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

void `$INSTANCE_NAME`_ReadLongRegister(const NRF_REGISTER_t reg, uint8_t* data , const size_t size)
{
    if( NULL == data )
    {
        return;
    }
    
    if( 5 < size)
    {
        return;
    }
    
    uint8_t i, j;
    
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
        
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData(NRF_R_REGISTER_CMD | reg);

    for(i = size; i != 0; i--)
    {
        `$SPI_INTERFACE`_WriteTxData( NRF_NOP_CMD );
    }

    while( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_IDLE ) );
    `$SS_PIN`_Write(1);
        
    (void)`$SPI_INTERFACE`_ReadRxData(); // This is the STATUS Register
    for(j = 0; j < size; j++)
    {
        data[j] = `$SPI_INTERFACE`_ReadRxData();
    }
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_R_REGISTER_CMD | reg );

    for(i = size; i != 0; i--)
    {
        `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_NOP_CMD );
    }
    
    while( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != ( 1 + size ) );
    `$SS_PIN`_Write(1);

    (void)`$SPI_INTERFACE`_SpiUartReadRxData(); // This is the STATUS Register

    for(j = 0; j < size; j++)
    {
        data[j] = `$SPI_INTERFACE`_SpiUartReadRxData();
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
    
    while( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
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

void `$INSTANCE_NAME`_WriteLongRegister(const NRF_REGISTER_t reg, const uint8_t* data, size_t size)
{
    if( NULL == data )
    {
        return;
    }
    
    if( 5 < size)
    {
        size = 5;
    }
    
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
        
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_INTERFACE`_PutArray(data, size);

    while( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
    `$SS_PIN`_Write(1);
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);

    `$SPI_INTERFACE`_SpiUartWriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_INTERFACE`_SpiUartPutArray(data, size);

    // Wait for the RxBuffer to have dataSize + 1 bytes,
    // dataSize + 1 because we need to count the Command + Register
    // byte at the beginning of the transaction.
    while( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != ( 1 + size ) );
    `$SS_PIN`_Write(1);
    
#endif
}

uint8_t `$INSTANCE_NAME`_ReadBit(const NRF_REGISTER_t reg, uint8_t bit)
{
    return ( `$INSTANCE_NAME`_ReadRegister( reg ) & ( 1 << bit ) ) != 0;
}

void `$INSTANCE_NAME`_WriteBit(const NRF_REGISTER_t reg, const uint8_t bit, const uint8_t value)
{
    // Get the @reg content
	uint8_t temp = `$INSTANCE_NAME`_ReadRegister( reg );
    
    // Check if @bit is already set
    if( ( temp & ( 1 << bit ) ) != 0 )
    {
        // it is set, return early if we wanted to set it ( @value == 1 ),
        // continue if we wanted to clear it ( @value == 0 )
        if ( value ) return;
    }
    
    // calculate the new value to write back to @reg, if @value is != 0, then
    // we set the bit, if @value == 0, then we clear the bit
    temp = value ? temp | ( 1 << bit ) : temp & ~( 1 << bit );
    
    // Write back to @reg
    `$INSTANCE_NAME`_WriteRegister( reg, temp );
}

void `$INSTANCE_NAME`_ClearBit(const NRF_REGISTER_t reg, const uint8_t bit)
{
    `$INSTANCE_NAME`_WriteBit(reg, bit, 0);
}

void `$INSTANCE_NAME`_SetBit(const NRF_REGISTER_t reg, const uint8_t bit)
{
    `$INSTANCE_NAME`_WriteBit(reg, bit, 1);
}

void `$INSTANCE_NAME`_SendCommand(const NRF_CMD_t cmd)
{
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData(cmd);
    
    while( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
    `$SS_PIN`_Write(1);
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData(cmd);
    
    while( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != 1 );
    `$SS_PIN`_Write(1);
    
#endif
}

// Used for a PTX device
// Reuse last transmitted payload.
// TX payload reuse is active until W_TX_PAYLOAD or FLUSH_TX is executed.
// TX payload reuse must not be activated or deactivated during package transmission.
void `$INSTANCE_NAME`_ReuseTxPayloadCmd(void)
{
    `$INSTANCE_NAME`_SendCommand( NRF_REUSE_TX_PL_CMD );
}

// Used in RX mode
// Flush RX FIFO. Should be not executed during transmission of
// acknowledge, that is, acknowledge package will not be completed
void `$INSTANCE_NAME`_FlushRxCmd(void)
{
    `$INSTANCE_NAME`_SendCommand( NRF_FLUSH_RX_CMD );
}

// Used in TX mode
// Flush TX FIFO
void `$INSTANCE_NAME`_FlushTxCmd(void)
{
    `$INSTANCE_NAME`_SendCommand( NRF_FLUSH_TX_CMD );
}

// Read RX payload: 1 - 32 bytes.
// A read operation always starts at byte 0.
// Payload is deleted from FIFO after it is read. Used in RX mode.
void `$INSTANCE_NAME`_ReadRXPayloadCmd(uint8_t* data, const size_t size)
{
    
    if ( NULL == data )
    {
        return;
    }
    
    uint8_t i, j; // declare in for loops, -std=c11
    
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData( NRF_R_RX_PAYLOAD_CMD );
    
    for(i = size; i != 0; i--)
    {
        `$SPI_INTERFACE`_WriteTxData( NRF_NOP_CMD );
    }

    while( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
    `$SS_PIN`_Write(1);
        
    (void)`$SPI_INTERFACE`_ReadRxData(); // This is the STATUS Register
    for(j = 0; j < size; j++)
    {
        data[j] = `$SPI_INTERFACE`_ReadRxData();
    }
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_R_RX_PAYLOAD_CMD );
    
    for(i = size; i != 0; i--)
    {
        `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_NOP_CMD );
    }
    
    while( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != ( 1 + size ) );
    `$SS_PIN`_Write(1);

    (void)`$SPI_INTERFACE`_SpiUartReadRxData(); // This is the STATUS Register
    for(j = 0; j < size; j++)
    {
        data[j] = `$SPI_INTERFACE`_SpiUartReadRxData();
    }
    
#endif
}

// Write TX payload: 1 - 32 bytes.
// A write operation always starts at byte 0 used in TX payload.
void `$INSTANCE_NAME`_WriteTXPayloadCmd(const uint8_t* data, const size_t size)
{
    if( NULL == data)
    {
        return;
    }
    
    if( 32 < size )
    {
        return;
    }
    
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData( NRF_W_TX_PAYLOAD_CMD );
    `$SPI_INTERFACE`_PutArray(data, size);
    
    while( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
    `$SS_PIN`_Write(1);
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_W_TX_PAYLOAD_CMD );
    `$SPI_INTERFACE`_SpiUartPutArray(data, size);
    
    while( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != ( 1 + size ) );
    `$SS_PIN`_Write(1);
    
#endif
}

// Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO
// Note: Flush RX FIFO if the read value is larger than 32 bytes
uint8_t `$INSTANCE_NAME`_ReadPayloadWidthCmd(void)
{
    uint8_t width;
    
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData( NRF_R_RX_PL_WID_CMD );
    `$SPI_INTERFACE`_WriteTxData( NRF_NOP_CMD );
    
    while( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
    `$SS_PIN`_Write(1);
    
    (void)`$SPI_INTERFACE`_ReadRxData(); // This is the STATUS Register
    width = `$SPI_INTERFACE`_ReadRxData();
    
    if( 32 < width)
    {
        `$INSTANCE_NAME`_FlushRxCmd();
    }
    
    return width;
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_R_RX_PL_WID_CMD );
    `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_NOP_CMD );
    
    while( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != ( 1 + size ) );
    `$SS_PIN`_Write(1);
    
    (void)`$SPI_INTERFACE`_SpiUartReadRxData(); // This is the STATUS Register
    width = `$SPI_INTERFACE`_SpiUartReadRxData();
    
    if( 32 < width)
    {
        `$INSTANCE_NAME`_FlushRxCmd(void);
    }
    
    return width;
    
#endif
}

// Used in RX mode
// Write Payload to be transmitted together with ACK packet
// on PIPE PPP (PPP valid in the range from 000 to 101). Maximum
// three ACK  packet payloads can be pending. Payloads with same PPP
// are handled using first in - first out principle.
// Write payload: 1 - 32 bytes.
// A write operation always starts at byte 0.
void `$INSTANCE_NAME`_WriteACKPayloadCmd(const NRF_DATA_PIPE_t pipe, const uint8_t* data, const size_t size)
{
    if ( NULL == data )
    {
        return;
    }
    
    if( 32 < size)
    {
        return;
    }
    
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
        
    `$SS_PIN`_Write(0);
    
    `$SPI_INTERFACE`_WriteTxData( NRF_W_ACK_PAYLOAD_CMD | pipe );
    `$SPI_INTERFACE`_PutArray(data, size);

    while( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
    `$SS_PIN`_Write(1);
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    
    `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_W_ACK_PAYLOAD_CMD | pipe );
    `$SPI_INTERFACE`_SpiUartPutArray(data, size);
    
    while( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != ( 1 + size ) );
    `$SS_PIN`_Write(1);
    
#endif
}

// Used in TX mode
// Disables AUTOACK on this specific packet.
void `$INSTANCE_NAME`_NoACKPayloadCmd(const uint8_t* data, const size_t size)
{
    if ( NULL == data )
    {
        return;
    }
    
    if( 32 < size )
    {
        return;
    }
    
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
        
    `$SS_PIN`_Write(0);
    
    `$SPI_INTERFACE`_WriteTxData( NRF_W_TX_PAYLOAD_NOACK_CMD );
    `$SPI_INTERFACE`_PutArray(data, size);

    while( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
    `$SS_PIN`_Write(1);
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    
    `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_W_TX_PAYLOAD_NOACK_CMD );
    `$SPI_INTERFACE`_SpiUartPutArray(data, size);
    
    while( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != ( 1 + size ) );
    `$SS_PIN`_Write(1);
    
#endif
}

// No Operation.
// Might be used to read the STATUS register
uint8_t `$INSTANCE_NAME`_NOPCmd(void)
{
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData( NRF_NOP_CMD );
    
    while( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
    CyDelayUs(5); // Let the /ss line go idle
    `$SS_PIN`_Write(1);
    
    return `$SPI_INTERFACE`_ReadRxData();
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_NOP_CMD );
    
    while( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != 1 );
    `$SS_PIN`_Write(1);
    
    return `$SPI_INTERFACE`_SpiUartReadRxData();
    
#endif
}

/* [] END OF FILE */
