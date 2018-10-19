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
#include "`$INSTANCE_NAME`_HAL.h"
#include "`$INSTANCE_NAME`_COMMANDS.h"
#include "`$INSTANCE_NAME`_DEFS.h"

/**
 * Send the specified command to the nRF24 radio.
 *
 * @param const nrf_cmd cmd: Command to send to the nRF24 radio.
 */
void `$INSTANCE_NAME`_send_command(const nrf_cmd cmd)
{
    uint8_t status = 0;
    
    `$INSTANCE_NAME`_spi_xfer(&cmd, &status, 1);
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
    `$INSTANCE_NAME`_send_command(NRF_CMD_REUSE_TX_PL);
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
    `$INSTANCE_NAME`_send_command(NRF_CMD_FLUSH_RX);
}

/**
 * Used in TX mode.
 * Flush TX FIFO.
 */
void `$INSTANCE_NAME`_flush_tx_cmd(void)
{
    `$INSTANCE_NAME`_send_command(NRF_CMD_FLUSH_TX);
}

/**
 * Used in RX mode.
 * Read RX payload: 1 - 32 bytes. A read operation always starts at byte 0.
 * Payload is deleted from FIFO after it is read.
 *
 * @param data: Data to be read.
 * @param data_size: Bytes of data to be read (max 32).
 */
void `$INSTANCE_NAME`_read_rx_payload_cmd(uint8_t *data, const size_t data_size)
{
    uint8_t nrf_data_out[data_size + 1];
    // the nrf_data_in array is to keep the spi sending dummy bytes so the
    // radio can send us the payload, there's no need to set the array to
    // any specific value, apart of the first element being the command
    uint8_t nrf_data_in[data_size + 1];
    
    nrf_data_in[0] = NRF_CMD_R_RX_PAYLOAD;
    
    `$INSTANCE_NAME`_spi_xfer(nrf_data_in, nrf_data_out, sizeof(nrf_data_out));
    
    memcpy(data, &nrf_data_out[1], data_size);
}

/**
 * Write TX payload: 1 - 32 bytes.
 * A write operation always starts at byte 0 used in TX payload.
 *
 * @param const uint8_t* data: Data to be sent.
 * @param const size_t size: Bytes of data to be sent (max 32).
 */
void `$INSTANCE_NAME`_write_tx_payload_cmd(const uint8_t *data, const size_t data_size)
{
    uint8_t nrf_data_in[data_size + 1];
    uint8_t nrf_data_out[data_size + 1];
    
    nrf_data_in[0] = NRF_CMD_W_TX_PAYLOAD;
    memcpy(&nrf_data_in[1], data, data_size);
    
    `$INSTANCE_NAME`_spi_xfer(nrf_data_in, nrf_data_out, sizeof(nrf_data_in));
}

/**
 * Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO.
 * Note: Flush RX FIFO if the read value is larger than 32 bytes.
 *
 * @return uint8_t: width of the payload on the top of the RX FIFO.
 */
uint8_t `$INSTANCE_NAME`_read_payload_width_cmd(void)
{
    uint8_t nrf_data_in[2] = {
        NRF_CMD_R_RX_PL_WID, NRF_CMD_NOP
    };
    // nrf_data_out[0] = STATUS_REGISTER, nrf_data_out[1] = payload_width
    uint8_t nrf_data_out[2] = {0};
    
    `$INSTANCE_NAME`_spi_xfer(nrf_data_in, nrf_data_out, sizeof(nrf_data_in));

    // If width is greater than 32 then is garbage, we must flush the RX FIFO
    if (32 < nrf_data_out[1]) {
        `$INSTANCE_NAME`_flush_rx_cmd();
    }

    return nrf_data_out[1];
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
                                        const size_t data_size)
{
    uint8_t nrf_data_in[data_size + 1];
    uint8_t nrf_data_out[data_size + 1];
    
    nrf_data_in[0] = NRF_CMD_W_ACK_PAYLOAD | pipe;
    memcpy(&nrf_data_in[1], data, data_size);
    
    `$INSTANCE_NAME`_spi_xfer(nrf_data_in, nrf_data_out, sizeof(nrf_data_in));
}

/**
 * Used in TX mode.
 * Disable AUTOACK on this packet.
 *
 * @param const uint8_t* data: Data to be sent.
 * @param const size_t size: Bytes of data to be sent (max 32).
 */
void `$INSTANCE_NAME`_no_ack_payload_cmd(const uint8_t* data, const size_t data_size)
{
    uint8_t nrf_data_in[data_size + 1];
    uint8_t nrf_data_out[data_size + 1];
    
    nrf_data_in[0] = NRF_CMD_W_TX_PAYLOAD_NOACK;
    memcpy(&nrf_data_in[1], data, data_size);
    
    `$INSTANCE_NAME`_spi_xfer(nrf_data_in, nrf_data_out, sizeof(nrf_data_in));
}

/**
 * NOP (No OPeration) Command. Useful to read the STATUS register.
 *
 * @return uint8_t: STATUS register.
 */
uint8_t `$INSTANCE_NAME`_nop_cmd(void)
{
    uint8_t dummy = 0xFF;
    uint8_t status = 0;
    
    `$INSTANCE_NAME`_spi_xfer(&dummy, &status, 1);
    
    return status;
}

/* [] END OF FILE */
