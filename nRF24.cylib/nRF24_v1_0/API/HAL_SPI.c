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

/**
 * @brief Read the content of a nRF24 register.
 *
 * @param const NrfRegister reg: Register to be read.
 * 
 * @return uint8_t: Content of the read register (reg).
 *
 */
uint8_t `$INSTANCE_NAME`_ReadRegister(const NrfRegister reg)
{
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData( NRF_R_REGISTER_CMD | reg );
    `$SPI_INTERFACE`_WriteTxData( NRF_NOP_CMD );
    
    while ( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
    `$SS_PIN`_Write(1);
    
    (void)`$SPI_INTERFACE`_ReadRxData(); // This is the STATUS Register
    return `$SPI_INTERFACE`_ReadRxData();
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData(NRF_R_REGISTER_CMD | reg);
    `$SPI_INTERFACE`_SpiUartWriteTxData(NRF_NOP_CMD);
    
    while ( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != 2 );
    `$SS_PIN`_Write(1);
    
    (void)`$SPI_INTERFACE`_SpiUartReadRxData(); // This is the STATUS Register
    return `$SPI_INTERFACE`_SpiUartReadRxData();
    
#endif
}

/**
 * @brief Read the content of a nRF24 register with more than 1byte of data.
 *
 * @param const NrfRegister reg: Register to be read.
 * @param uint8_t* data: Pointer to the array where the content of the register will be
 * stored.
 * @param size_t size: Size of the register, larger register hold 5 bytes of data.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_ReadLongRegister(const NrfRegister reg, uint8_t* data , const size_t size)
{
    if ( NULL == data ) {
        return;
    }
    
    if ( 5 < size) {
        return;
    }

    uint8_t i, j;
    
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
        
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData(NRF_R_REGISTER_CMD | reg);

    for (i = size; i != 0; i--) {
        `$SPI_INTERFACE`_WriteTxData( NRF_NOP_CMD );
    }

    while ( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_IDLE ) );
    `$SS_PIN`_Write(1);
        
    (void)`$SPI_INTERFACE`_ReadRxData(); // This is the STATUS Register
    for (j = 0; j < size; j++) {
        data[j] = `$SPI_INTERFACE`_ReadRxData();
    }
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_R_REGISTER_CMD | reg );

    for (i = size; i != 0; i--) {
        `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_NOP_CMD );
    }
    
    while ( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != ( 1 + size ) );
    `$SS_PIN`_Write(1);

    (void)`$SPI_INTERFACE`_SpiUartReadRxData(); // This is the STATUS Register
    for (j = 0; j < size; j++) {
        data[j] = `$SPI_INTERFACE`_SpiUartReadRxData();
    }
    
#endif
}

/**
 * @brief Write data to a nRF24 Register.
 *
 * @param const NrfRegister reg: Register where the data will be written.
 * @param const uint8_t data: Data to be written into the register.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_WriteRegister(const NrfRegister reg, const uint8_t data)
{
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_INTERFACE`_WriteTxData(data);
    
    while ( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
    `$SS_PIN`_Write(1);
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_INTERFACE`_SpiUartWriteTxData(data);

    while ( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != 2 );
    `$SS_PIN`_Write(1);
    
#endif
}

/**
 * @brief
 *
 *
 * @param const NrfRegister reg:
 * @param const uint8_t* data:
 * @param size_t size:
 * 
 * @return None.
 *
 */
void `$INSTANCE_NAME`_WriteLongRegister(const NrfRegister reg, const uint8_t* data, size_t size)
{
    if ( NULL == data ) {
        return;
    }
    
    if ( 5 < size) {
        size = 5;
    }
    
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
        
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_INTERFACE`_PutArray(data, size);

    while ( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
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
    while ( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != ( 1 + size ) );
    `$SS_PIN`_Write(1);
    
#endif
}

/**
 * @brief
 *
 *
 * @param const NrfRegister reg:
 * @param uint8_t bit:
 * 
 * @return uint8_t:
 *
 */
uint8_t `$INSTANCE_NAME`_ReadBit(const NrfRegister reg, uint8_t bit)
{
    // Return 1 if bit is set (1), return 0 if the bit is clear (0)
    return ( `$INSTANCE_NAME`_ReadRegister( reg ) & ( 1 << bit ) ) != 0;
}

/**
 * @brief
 *
 *
 * @param const NrfRegister reg:
 * @param const uint8_t bit:
 * @param const uint8_t value:
 * 
 * @return None.
 *
 */
void `$INSTANCE_NAME`_WriteBit(const NrfRegister reg, const uint8_t bit, const uint8_t value)
{
    // Get the @reg content
    uint8_t temp = `$INSTANCE_NAME`_ReadRegister( reg );
    
    // Check if @bit is already set
    if ( ( temp & ( 1 << bit ) ) != 0 ) {
        // Return as we wanted to set it ( @value == 1 ),
        // continue if we wanted to clear it ( @value == 0 )
        if ( value ) return;
    }
    
    // calculate the new value to write back to @reg, if @value is != 0, then
    // we set the bit, if @value == 0, then we clear the bit
    temp = value ? temp | ( 1 << bit ) : temp & ~( 1 << bit );
    
    // Write back to @reg
    `$INSTANCE_NAME`_WriteRegister( reg, temp );
}

/**
 * @brief
 *
 *
 * @param const NrfRegister reg:
 * @param const uint8_t bit:
 * 
 * @return None.
 *
 */
void `$INSTANCE_NAME`_ClearBit(const NrfRegister reg, const uint8_t bit)
{
    `$INSTANCE_NAME`_WriteBit(reg, bit, 0);
}

/**
 * @brief
 *
 *
 * @param const NrfRegister reg:
 * @param const uint8_t bit:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_SetBit(const NrfRegister reg, const uint8_t bit)
{
    `$INSTANCE_NAME`_WriteBit(reg, bit, 1);
}

/**
 * @brief
 *
 *
 * @param const NrfCmd cmd:
 * 
 * @return None.
 *
 */
void `$INSTANCE_NAME`_SendCommand(const NrfCmd cmd)
{
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData(cmd);
    
    while ( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
    `$SS_PIN`_Write(1);
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData(cmd);
    
    while ( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != 1 );
    `$SS_PIN`_Write(1);
    
#endif
}

/**
 * @brief Reuse last transmitted payload.
 *
 * Used for a PTX device.
 * Reuse last transmitted payload. TX payload reuse is active until
 * W_TX_PAYLOAD or FLUSH_TX is executed.
 * TX payload reuse must not be activated or deactivated during package
 * transmission.
 *
 * @param None.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_TX_ReuseTxPayloadCmd(void)
{
    `$INSTANCE_NAME`_SendCommand( NRF_REUSE_TX_PL_CMD );
}

/**
 * @brief Flush RX FIFO.
 *
 * Used in RX mode.
 * Flush RX FIFO. Should be not executed during transmission of acknowledge,
 * that is, acknowledge package will not be completed.
 *
 * @param None.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_FlushRxCmd(void)
{
    `$INSTANCE_NAME`_SendCommand( NRF_FLUSH_RX_CMD );
}

/**
 * @brief Flush TX FIFO.
 *
 * Used in TX mode. Flush TX FIFO.
 *
 * @param None.
 * 
 * @return None.
 *
 */
void `$INSTANCE_NAME`_FlushTxCmd(void)
{
    `$INSTANCE_NAME`_SendCommand( NRF_FLUSH_TX_CMD );
}

/**
 * @brief Read RX payload.
 *
 * Used in RX mode.
 * Read RX payload: 1 - 32 bytes.
 * A read operation always starts at byte 0.
 * Payload is deleted from FIFO after it is read.
 *
 * @param uint8_t* data:
 * @param const size_t size:
 * 
 * @return None.
 *
 */
void `$INSTANCE_NAME`_RX_ReadRXPayloadCmd(uint8_t* data, const size_t size)
{
    
    if ( NULL == data ) {
        return;
    }
    
    uint8_t i, j; // declare in for loops, -std=c11
    
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData( NRF_R_RX_PAYLOAD_CMD );
    
    for (i = size; i != 0; i--) {
        `$SPI_INTERFACE`_WriteTxData( NRF_NOP_CMD );
    }

    while ( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
    `$SS_PIN`_Write(1);
        
    (void)`$SPI_INTERFACE`_ReadRxData(); // This is the STATUS Register
    for (j = 0; j < size; j++) {
        data[j] = `$SPI_INTERFACE`_ReadRxData();
    }
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_R_RX_PAYLOAD_CMD );
    
    for (i = size; i != 0; i--) {
        `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_NOP_CMD );
    }
    
    while ( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != ( 1 + size ) );
    `$SS_PIN`_Write(1);

    (void)`$SPI_INTERFACE`_SpiUartReadRxData(); // This is the STATUS Register
    for (j = 0; j < size; j++) {
        data[j] = `$SPI_INTERFACE`_SpiUartReadRxData();
    }
    
#endif
}

/**
 * @brief Write TX payload: 1 - 32 bytes.
 *
 * Write TX payload: 1 - 32 bytes.
 * A write operation always starts at byte 0 used in TX payload.
 *
 * @param const uint8_t* data:
 * @param const size_t size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_WriteTXPayloadCmd(const uint8_t* data, const size_t size)
{
    if ( NULL == data) {
        return;
    }
    
    if ( 32 < size ) {
        return;
    }
    
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData( NRF_W_TX_PAYLOAD_CMD );
    `$SPI_INTERFACE`_PutArray(data, size);
    
    while ( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
    `$SS_PIN`_Write(1);
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_W_TX_PAYLOAD_CMD );
    `$SPI_INTERFACE`_SpiUartPutArray(data, size);
    
    while ( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != ( 1 + size ) );
    `$SS_PIN`_Write(1);
    
#endif
}

/**
 * @brief Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO.
 *
 * Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO.
 * Note: Flush RX FIFO if the read value is larger than 32 bytes.
 * 
 * @param None.
 *
 * @return uint8_t: width of the payload on the top of the RX FIFO.
 *
 */
uint8_t `$INSTANCE_NAME`_ReadPayloadWidthCmd(void)
{
    uint8_t width;
    
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData( NRF_R_RX_PL_WID_CMD );
    `$SPI_INTERFACE`_WriteTxData( NRF_NOP_CMD );
    
    while ( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
    `$SS_PIN`_Write(1);
    
    (void)`$SPI_INTERFACE`_ReadRxData(); // This is the STATUS Register
    width = `$SPI_INTERFACE`_ReadRxData();
    
    if ( 32 < width) {
        `$INSTANCE_NAME`_FlushRxCmd();
    }
    
    return width;
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_R_RX_PL_WID_CMD );
    `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_NOP_CMD );
    
    while ( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != ( 1 + size ) );
    `$SS_PIN`_Write(1);
    
    (void)`$SPI_INTERFACE`_SpiUartReadRxData(); // This is the STATUS Register
    width = `$SPI_INTERFACE`_SpiUartReadRxData();
    
    if ( 32 < width) {
        `$INSTANCE_NAME`_FlushRxCmd(void);
    }
    
    return width;
    
#endif
}

/**
 * @brief Write Payload to be transmitted together with ACK packet.
 *
 * Used in RX mode.
 * Write Payload to be transmitted together with ACK packet on PIPE PPP
 * (PPP valid in the range from 000 to 101). Maximum three ACK  packet
 * payloads can be pending. Payloads with same PPP are handled using
 * first in - first out principle.
 * Write payload: 1 - 32 bytes.
 * A write operation always starts at byte 0.
 *
 * @param const NrfDataPipe pipe:
 * @param const uint8_t* data:
 * @param const size_t size
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_RX_WriteACKPayloadCmd(const NrfDataPipe pipe,
                                const uint8_t* data, const size_t size)
{
    if ( NULL == data ) {
        return;
    }
    
    if ( 32 < size) {
        return;
    }
    
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
        
    `$SS_PIN`_Write(0);
    
    `$SPI_INTERFACE`_WriteTxData( NRF_W_ACK_PAYLOAD_CMD | pipe );
    `$SPI_INTERFACE`_PutArray(data, size);

    while ( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
    `$SS_PIN`_Write(1);
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    
    `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_W_ACK_PAYLOAD_CMD | pipe );
    `$SPI_INTERFACE`_SpiUartPutArray(data, size);
    
    while ( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != ( 1 + size ) );
    `$SS_PIN`_Write(1);
    
#endif
}

/**
 * @brief Disables AUTOACK on this specific packet.
 *
 * Used in TX mode.
 * Disables AUTOACK on this specific packet.
 *
 * @param const uint8_t* data:
 * @param const size_t size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_TX_NoACKPayloadCmd(const uint8_t* data, const size_t size)
{
    if ( NULL == data ) {
        return;
    }
    
    if ( 32 < size ) {
        return;
    }
    
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
        
    `$SS_PIN`_Write(0);
    
    `$SPI_INTERFACE`_WriteTxData( NRF_W_TX_PAYLOAD_NOACK_CMD );
    `$SPI_INTERFACE`_PutArray(data, size);

    while ( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
    `$SS_PIN`_Write(1);
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    
    `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_W_TX_PAYLOAD_NOACK_CMD );
    `$SPI_INTERFACE`_SpiUartPutArray(data, size);
    
    while ( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != ( 1 + size ) );
    `$SS_PIN`_Write(1);
    
#endif
}

/**
 * @brief Send NOP (No OPeration) Command.
 * 
 * Send NOP (No OPeration) Command. Might be used to read the STATUS register.
 *
 * @param None.
 *
 * @return uint8_t: Content of the nRF24 STATUS register.
 *
 */
uint8_t `$INSTANCE_NAME`_NOPCmd(void)
{
#if !defined(CY_SCB_`$SPI_INTERFACE`_H) // UDB Block
    
    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData( NRF_NOP_CMD );
    
    while ( 0 == ( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_DONE ) );
    `$SS_PIN`_Write(1);
    
    return `$SPI_INTERFACE`_ReadRxData();
    
#else // SCB Block
    
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();
    
    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData( NRF_NOP_CMD );
    
    while ( `$SPI_INTERFACE`_SpiUartGetRxBufferSize() != 1 );
    `$SS_PIN`_Write(1);
    
    return `$SPI_INTERFACE`_SpiUartReadRxData();
    
#endif
}

/* [] END OF FILE */
