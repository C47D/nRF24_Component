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

/**
* @file `$INSTANCE_NAME`_COMMANDS.c
*
* @brief The nRF24 radio is controlled via commands, this file implement all
* the available commands.
*/

#include "`$INSTANCE_NAME`_CONFIG.h"
#include "`$INSTANCE_NAME`_COMMANDS.h"
#include "`$INSTANCE_NAME`_REGS.h"

/**
 * Send the specified command to the nRF24 radio.
 *
 * @param const nrf_cmd cmd: Command to send to the nRF24 radio.
 */
void `$INSTANCE_NAME`_send_command(const nrf_cmd cmd)
{
#if defined (_PSOC6)
    Cy_SCB_ClearRxFifo(`$SPI_MASTER`_HW);
    Cy_SCB_ClearTxFifo(`$SPI_MASTER`_HW);

    Cy_GPIO_Clr(`$SS_PIN`_PORT, `$SS_PIN`_NUM);

    Cy_SCB_WriteArrayBlocking(`$SPI_MASTER`_HW, (void *) &cmd, 1);

    while (Cy_SCB_IsTxComplete(`$SPI_MASTER`_HW) == false) {
    }
    CyDelayUs(1);

    Cy_GPIO_Set(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
#elif defined (_PSOC4_SCB)
    `$SPI_MASTER`_SpiUartClearRxBuffer();
    `$SPI_MASTER`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_SpiUartWriteTxData(cmd);

    while (`$SPI_MASTER`_SpiUartGetRxBufferSize() != 1) {
    }
    `$SS_PIN`_Write(1);
#else // _PSOC_UDB
    `$SPI_MASTER`_ClearFIFO();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_WriteTxData(cmd);

    while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_BYTE_COMPLETE)) {
    }
    `$SS_PIN`_Write(1);
#endif
}

/**
 * @brief Reuse last transmitted payload.
 *
 * Used for a PTX device.
 * Reuse last transmitted payload. TX payload reuse is active until
 * W_TX_PAYLOAD or FLUSH_TX is executed, TX payload reuse must not be
 * activated or deactivated during package transmission.
 */
void `$INSTANCE_NAME`_reuse_tx_payload_cmd(void)
{
    `$INSTANCE_NAME`_send_command(NRF_REUSE_TX_PL_CMD);
}

/**
 * @brief Flush RX FIFO.
 *
 * Used in RX mode.
 * Flush RX FIFO. Should be not executed during transmission of acknowledge,
 * that is, acknowledge package will not be completed.
 */
void `$INSTANCE_NAME`_flush_rx_cmd(void)
{
    `$INSTANCE_NAME`_send_command(NRF_FLUSH_RX_CMD);
}

/**
 * Used in TX mode.
 * Flush TX FIFO.
 */
void `$INSTANCE_NAME`_flush_tx_cmd(void)
{
    `$INSTANCE_NAME`_send_command(NRF_FLUSH_TX_CMD);
}

/**
 * Used in RX mode.
 * Read RX payload: 1 - 32 bytes. A read operation always starts at byte 0.
 * Payload is deleted from FIFO after it is read.
 *
 * @param uint8_t* data: Data to be read.
 * @param const size_t size: Bytes of data to be read (max 32).
 */
void `$INSTANCE_NAME`_read_rx_payload_cmd(uint8_t* data, const size_t size)
{
#if defined (_PSOC6)
    Cy_SCB_ClearRxFifo(`$SPI_MASTER`_HW);
    Cy_SCB_ClearTxFifo(`$SPI_MASTER`_HW);

    Cy_GPIO_Clr(`$SS_PIN`_PORT, `$SS_PIN`_NUM);

    Cy_SCB_Write(`$SPI_MASTER`_HW, NRF_R_RX_PAYLOAD_CMD);
    while (Cy_SCB_GetNumInRxFifo(`$SPI_MASTER`_HW) != 0) {
    }

    (void)Cy_SCB_ReadRxFifo(`$SPI_MASTER`_HW);

    for (size_t i = 0; i < size; i++) {
        Cy_SCB_Write(`$SPI_MASTER`_HW, NRF_NOP_CMD);
        while (Cy_SCB_GetNumInRxFifo(`$SPI_MASTER`_HW) != 0) {
        }
        data[i] = Cy_SCB_ReadRxFifo(`$SPI_MASTER`_HW);
    }
    Cy_GPIO_Set(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
#elif defined (_PSOC4_SCB)
    `$SPI_MASTER`_SpiUartClearRxBuffer();
    `$SPI_MASTER`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_SpiUartWriteTxData(NRF_R_RX_PAYLOAD_CMD);
    while (`$SPI_MASTER`_SpiUartGetRxBufferSize() == 0){
    }
    // Read the status register, just to clear the rx fifo
    `$SPI_MASTER`_SpiUartReadRxData();

    for (size_t i = 0; i < size; i++) {
        `$SPI_MASTER`_SpiUartWriteTxData(NRF_NOP_CMD);
        while (`$SPI_MASTER`_SpiUartGetRxBufferSize() == 0){
        }
        data[i] = `$SPI_MASTER`_SpiUartReadRxData();
    }
    `$SS_PIN`_Write(1);
#else // _PSOC_UDB
    `$SPI_MASTER`_ClearFIFO();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_WriteTxData(NRF_R_RX_PAYLOAD_CMD);

    // Wait for the byte to be sent
    while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_BYTE_COMPLETE)) {
    }

    // Read the status register, just to clear the rx fifo
    `$SPI_MASTER`_ReadRxData();

    for (size_t i = 0; i < size; i++) {
        `$SPI_MASTER`_WriteTxData(NRF_NOP_CMD);
        while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_BYTE_COMPLETE)) {
        }
        data[i] = `$SPI_MASTER`_ReadRxData();
    }
    `$SS_PIN`_Write(1);
#endif
}

/**
 * Write TX payload: 1 - 32 bytes.
 * A write operation always starts at byte 0 used in TX payload.
 *
 * @param const uint8_t* data: Data to be sent.
 * @param const size_t size: Bytes of data to be sent (max 32).
 */
void `$INSTANCE_NAME`_write_tx_payload_cmd(const uint8_t* data, const size_t size)
{
#if defined (_PSOC6)
    Cy_SCB_ClearRxFifo(`$SPI_MASTER`_HW);
    Cy_SCB_ClearTxFifo(`$SPI_MASTER`_HW);

    Cy_GPIO_Clr(`$SS_PIN`_PORT, `$SS_PIN`_NUM);

    Cy_SCB_Write(`$SPI_MASTER`_HW, NRF_W_TX_PAYLOAD_CMD);
    Cy_SCB_WriteArrayBlocking(`$SPI_MASTER`_HW, (void *) data, size);

    while (Cy_SCB_IsTxComplete(`$SPI_MASTER`_HW) == false) {
    }
    CyDelayUs(1);

    Cy_GPIO_Set(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
#elif defined (_PSOC4_SCB)
    `$SPI_MASTER`_SpiUartClearRxBuffer();
    `$SPI_MASTER`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_SpiUartWriteTxData(NRF_W_TX_PAYLOAD_CMD);
    `$SPI_MASTER`_SpiUartPutArray(data, size);

    while (`$SPI_MASTER`_SpiUartGetRxBufferSize() != (1 + size)) {
    }
    `$SS_PIN`_Write(1);
#else // _PSOC_UDB
    `$SPI_MASTER`_ClearFIFO();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_WriteTxData(NRF_W_TX_PAYLOAD_CMD);
    for (size_t i = 0; i < size; i++) {
        SPI_WriteTxData(data[i]);
        while (!(SPI_ReadTxStatus() & SPI_STS_BYTE_COMPLETE)) {
        }
    }

    while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_SPI_IDLE)) {
    }
    `$SS_PIN`_Write(1);
#endif
}

/**
 * Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO.
 * Note: Flush RX FIFO if the read value is larger than 32 bytes.
 *
 * @return uint8_t: width of the payload on the top of the RX FIFO.
 */
uint8_t `$INSTANCE_NAME`_read_payload_width_cmd(void)
{
    uint8_t width = 0;
#if defined (_PSOC6)
    Cy_SCB_ClearRxFifo(`$SPI_MASTER`_HW);
    Cy_SCB_ClearTxFifo(`$SPI_MASTER`_HW);

    Cy_GPIO_Clr(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
#if 0
    Cy_SCB_Write(`$SPI_MASTER`_HW, NRF_R_RX_PL_WID_CMD);
    Cy_SCB_Write(`$SPI_MASTER`_HW, NRF_NOP_CMD);
#else
    Cy_SCB_WriteArrayBlocking(`$SPI_MASTER`_HW,
        (void *)(uint8_t []){NRF_R_RX_PL_WID_CMD, NRF_NOP_CMD}, 2);
#endif
    while (Cy_SCB_GetNumInRxFifo(`$SPI_MASTER`_HW) != 2) {
    }
    Cy_GPIO_Set(`$SS_PIN`_PORT, `$SS_PIN`_NUM);

    (void)Cy_SCB_ReadRxFifo(`$SPI_MASTER`_HW);
    width = Cy_SCB_ReadRxFifo(`$SPI_MASTER`_HW);
#elif defined (_PSOC4_SCB)
    `$SPI_MASTER`_SpiUartClearRxBuffer();
    `$SPI_MASTER`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_SpiUartWriteTxData(NRF_R_RX_PL_WID_CMD);
    `$SPI_MASTER`_SpiUartWriteTxData(NRF_NOP_CMD);

    while (`$SPI_MASTER`_SpiUartGetRxBufferSize() != 2) {
    }
    `$SS_PIN`_Write(1);

    // This is the STATUS Register
    (void)`$SPI_MASTER`_SpiUartReadRxData();
    // This is the data we want
    width = `$SPI_MASTER`_SpiUartReadRxData();
#else // _PSOC_UDB
    `$SPI_MASTER`_ClearFIFO();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_WriteTxData(NRF_R_RX_PL_WID_CMD);
    `$SPI_MASTER`_WriteTxData(NRF_NOP_CMD);

    while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_BYTE_COMPLETE)) {
    }
    `$SS_PIN`_Write(1);

    (void)`$SPI_MASTER`_ReadRxData();
    width = `$SPI_MASTER`_ReadRxData();
#endif

    // If width is greater than 32 then is garbage, we must flush the RX FIFO
    if (32 < width) {
        `$INSTANCE_NAME`_flush_rx_cmd();
    }

    return width;
}

/**
 * @brief Write Payload to be transmitted together with ACK packet.
 *
 * Used in RX mode.
 * Write Payload to be transmitted together with ACK packet on PIPE PPP
 * (PPP valid in the range from 000 to 101). Maximum three ACK packet
 * payloads can be pending. Payloads with same PPP are handled using
 * first in - first out principle.
 * Write payload: 1 - 32 bytes.
 * A write operation always starts at byte 0.
 *
 * @param const nrf_pipe pipe: Pipe to use.
 * @param const uint8_t* data: Data to be sent.
 * @param const size_t size: Bytes of data to be sent (max 32)
 */
void `$INSTANCE_NAME`_write_ack_payload_cmd(const nrf_pipe pipe, const uint8_t* data,
                                        const size_t size)
{
#if defined (_PSOC6)
    Cy_SCB_ClearRxFifo(`$SPI_MASTER`_HW);
    Cy_SCB_ClearTxFifo(`$SPI_MASTER`_HW);

    Cy_GPIO_Clr(`$SS_PIN`_PORT, `$SS_PIN`_NUM);

    Cy_SCB_Write(`$SPI_MASTER`_HW, NRF_W_ACK_PAYLOAD_CMD | pipe);
    Cy_SCB_WriteArrayBlocking(`$SPI_MASTER`_HW, (void *) data, size);

    while (Cy_SCB_GetNumInRxFifo(`$SPI_MASTER`_HW) != (1 + size)) {
    }
    Cy_GPIO_Set(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
#elif defined (_PSOC4_SCB)
    `$SPI_MASTER`_SpiUartClearRxBuffer();
    `$SPI_MASTER`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);

    `$SPI_MASTER`_SpiUartWriteTxData(NRF_W_ACK_PAYLOAD_CMD | pipe);
    `$SPI_MASTER`_SpiUartPutArray(data, size);

    while (`$SPI_MASTER`_SpiUartGetRxBufferSize() != (1 + size)) {
    }
    `$SS_PIN`_Write(1);
#else // _PSOC_UDB
    `$SPI_MASTER`_ClearFIFO();

    `$SS_PIN`_Write(0);

    `$SPI_MASTER`_WriteTxData(NRF_W_ACK_PAYLOAD_CMD | pipe);
    for (size_t i = 0; i < size; i++) {
        `$SPI_MASTER`_WriteTxData(data[i]);
        while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_BYTE_COMPLETE)){
        }
    }

    while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_SPI_IDLE)) {
    }
    `$SS_PIN`_Write(1);
#endif
}

/**
 * Used in TX mode.
 * Disable AUTOACK on this packet.
 *
 * @param const uint8_t* data: Data to be sent.
 * @param const size_t size: Bytes of data to be sent (max 32).
 */
void `$INSTANCE_NAME`_no_ack_payload_cmd(const uint8_t* data, const size_t size)
{
#if defined (_PSOC6)
    Cy_SCB_ClearRxFifo(`$SPI_MASTER`_HW);
    Cy_SCB_ClearTxFifo(`$SPI_MASTER`_HW);

    Cy_GPIO_Clr(`$SS_PIN`_PORT, `$SS_PIN`_NUM);

    Cy_SCB_Write(`$SPI_MASTER`_HW, NRF_W_TX_PAYLOAD_NOACK_CMD);
    Cy_SCB_WriteArrayBlocking(`$SPI_MASTER`_HW, (void *) data, size);

    while (Cy_SCB_GetNumInRxFifo(`$SPI_MASTER`_HW) != (1 + size)) {
    }
    Cy_GPIO_Set(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
#elif defined (_PSOC4_SCB)
    `$SPI_MASTER`_SpiUartClearRxBuffer();
    `$SPI_MASTER`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);

    `$SPI_MASTER`_SpiUartWriteTxData(NRF_W_TX_PAYLOAD_NOACK_CMD);
    `$SPI_MASTER`_SpiUartPutArray(data, size);

    while (`$SPI_MASTER`_SpiUartGetRxBufferSize() != (1 + size)) {
    }
    `$SS_PIN`_Write(1);
#else // _PSOC_UDB
    `$SPI_MASTER`_ClearFIFO();

    `$SS_PIN`_Write(0);

    `$SPI_MASTER`_WriteTxData(NRF_W_TX_PAYLOAD_NOACK_CMD);
    for (size_t i = 0; i < size; i++) {
        `$SPI_MASTER`_WriteTxData(data[i]);
        while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_BYTE_COMPLETE)){
        }
    }

    while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_SPI_IDLE)) {
    }
    `$SS_PIN`_Write(1);
#endif
}

/**
 * NOP (No OPeration) Command. Might be used to read the STATUS register.
 *
 * @return uint8_t: STATUS register.
 */
uint8_t `$INSTANCE_NAME`_nop_cmd(void)
{
    uint8_t status = 0;
#if defined (_PSOC6)
    Cy_SCB_ClearRxFifo(`$SPI_MASTER`_HW);
    Cy_SCB_ClearTxFifo(`$SPI_MASTER`_HW);

    Cy_GPIO_Clr(`$SS_PIN`_PORT, `$SS_PIN`_NUM);

    Cy_SCB_Write(`$SPI_MASTER`_HW, NRF_NOP_CMD);

    while (Cy_SCB_GetNumInRxFifo(`$SPI_MASTER`_HW) != 1) {
    }
    Cy_GPIO_Set(`$SS_PIN`_PORT, `$SS_PIN`_NUM);

    status = Cy_SCB_ReadRxFifo(`$SPI_MASTER`_HW);
#elif defined (_PSOC4_SCB)
    `$SPI_MASTER`_SpiUartClearRxBuffer();
    `$SPI_MASTER`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_SpiUartWriteTxData(NRF_NOP_CMD);

    while (`$SPI_MASTER`_SpiUartGetRxBufferSize() != 1) {
    }
    `$SS_PIN`_Write(1);

    status = `$SPI_MASTER`_SpiUartReadRxData();
#else // _PSOC_UDB
    `$SPI_MASTER`_ClearFIFO();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_WriteTxData(NRF_NOP_CMD);

    while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_BYTE_COMPLETE)) {
    }
    `$SS_PIN`_Write(1);

    status = `$SPI_MASTER`_ReadRxData();
#endif
    return status;
}

/* [] END OF FILE */
