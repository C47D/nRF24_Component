/**
* @file     `$INSTANCE_NAME`_COMMANDS.c
* @version  3
* @brief    nRF24 commands.
*/
#include <string.h>

#include "`$INSTANCE_NAME`_CONFIG.h"
#include "`$INSTANCE_NAME`_HAL.h"
#include "`$INSTANCE_NAME`_COMMANDS.h"
#include "`$INSTANCE_NAME`_DEFS.h"

/**
 * @brief Send the @c cmd to the nRF24.
 *
 * @param[in]   nrf_cmd cmd: Command to be sent.
 */
static uint8_t `$INSTANCE_NAME`_send_cmd(const nrf_cmd cmd);

/**
 * @brief Reuse the last transmitted payload.
 *
 * Reuse last transmitted payload. TX payload reuse is active until
 * NRF_CMD_W_TX_PAYLOAD or NRF_CMD_FLUSH_TX commands are executed.
 *
 * @note Used in TX mode.
 *
 * @warning TX payload reuse must not be activated or deactivated during
 * package transmission.
 */
void `$INSTANCE_NAME`_cmd_reuse_tx_payload(void)
{
    `$INSTANCE_NAME`_send_cmd(NRF_CMD_REUSE_TX_PL);
}

/**
 * @brief Flush the RX FIFO.
 *
 * @note Used in RX mode.
 *
 * @warning Should be not executed during transmission of acknowledge,
 * that is, acknowledge package will not be completed.
 */
void `$INSTANCE_NAME`_cmd_flush_rx(void)
{
    `$INSTANCE_NAME`_send_cmd(NRF_CMD_FLUSH_RX);
}

/**
 * @brief Flush the TX FIFO.
 * 
 * @note Used in TX mode.
 */
void `$INSTANCE_NAME`_cmd_flush_tx(void)
{
    `$INSTANCE_NAME`_send_cmd(NRF_CMD_FLUSH_TX);
}

/**
 * @brief
 * Read RX payload: 1 - 32 bytes. A read operation always starts at byte 0.
 * Payload is deleted from FIFO after it is read.
 *
 * @note Used in RX mode.
 *
 * @param payload: payload to be read.
 * @param data_size: Bytes of payload to be read (max 32).
 */
void `$INSTANCE_NAME`_cmd_read_rx_payload(uint8_t *payload, const size_t payload_size)
{
    uint8_t nrf_data_out[payload_size + 1];
    // the nrf_data_in array is to keep the spi sending dummy bytes so the
    // radio can send us the payload, there's no need to set the array to
    // any specific value, apart of the first element being the command
    uint8_t nrf_data_in[payload_size + 1];
    
    nrf_data_in[0] = NRF_CMD_R_RX_PAYLOAD;
    
    `$INSTANCE_NAME`_spi_xfer(nrf_data_in, nrf_data_out, sizeof(nrf_data_out));
    
    memcpy(payload, &nrf_data_out[1], payload_size);
}

/**
 * @brief Write TX payload: 1 - 32 bytes.
 *
 * A write operation always starts at byte 0 used in TX payload.
 *
 * @param const uint8_t* data: Data to be sent.
 * @param const size_t size: Bytes of data to be sent (max 32).
 */
void `$INSTANCE_NAME`_cmd_write_tx_payload(const uint8_t *payload, const size_t payload_size)
{
    uint8_t nrf_data_in[payload_size + 1];
    uint8_t nrf_data_out[payload_size + 1];
    
    nrf_data_in[0] = NRF_CMD_W_TX_PAYLOAD;
    memcpy(&nrf_data_in[1], payload, payload_size);
    
    `$INSTANCE_NAME`_spi_xfer(nrf_data_in, nrf_data_out, sizeof(nrf_data_in));
}

/**
 * @brief Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO.
 * 
 * @note Flush RX FIFO if the read value is larger than 32 bytes.
 *
 * @return uint8_t: width of the payload on the top of the RX FIFO.
 */
uint8_t `$INSTANCE_NAME`_cmd_read_payload_width(void)
{
    uint8_t nrf_data_in[2] = {
        NRF_CMD_R_RX_PL_WID, NRF_CMD_NOP
    };
    // nrf_data_out[0] = STATUS_REGISTER, nrf_data_out[1] = payload_width
    uint8_t nrf_data_out[2] = {0};
    
    `$INSTANCE_NAME`_spi_xfer(nrf_data_in, nrf_data_out, sizeof(nrf_data_in));

    return nrf_data_out[1];
}

/**
 * @brief Write Payload to be transmitted together with ACK packet.
 *
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
 *
 * @note Used in RX mode.
 */
void `$INSTANCE_NAME`_cmd_write_ack_payload(const nrf_pipe pipe,
                                            const uint8_t* payload, const size_t payload_size)
{
    uint8_t nrf_data_in[payload_size + 1];
    uint8_t nrf_data_out[payload_size + 1];
    
    nrf_data_in[0] = NRF_CMD_W_ACK_PAYLOAD | pipe;
    memcpy(&nrf_data_in[1], payload, payload_size);
    
    `$INSTANCE_NAME`_spi_xfer(nrf_data_in, nrf_data_out, sizeof(nrf_data_in));
}

/**
 * @brief Disable AUTOACK on this packet.
 *
 * @param const uint8_t* data: Data to be sent.
 * @param const size_t size: Bytes of data to be sent (max 32).
 *
 * @note Used in TX mode.
 */
void `$INSTANCE_NAME`_cmd_no_ack_payload(const uint8_t* payload, const size_t payload_size)
{
    uint8_t nrf_data_in[payload_size + 1];
    uint8_t nrf_data_out[payload_size + 1];
    
    nrf_data_in[0] = NRF_CMD_W_TX_PAYLOAD_NO_ACK;
    memcpy(&nrf_data_in[1], payload, payload_size);
    
    `$INSTANCE_NAME`_spi_xfer(nrf_data_in, nrf_data_out, sizeof(nrf_data_in));
}

/**
 * @brief NOP (No OPeration) Command. Useful to read the STATUS register.
 *
 * @return uint8_t: STATUS register.
 */
uint8_t `$INSTANCE_NAME`_cmd_nop(void)
{
    return `$INSTANCE_NAME`_send_cmd(NRF_CMD_NOP);
}

static uint8_t `$INSTANCE_NAME`_send_cmd(const nrf_cmd cmd)
{
    uint8_t status = 0;
    `$INSTANCE_NAME`_spi_xfer(&cmd, &status, 1);
    return status;
}

/* [] END OF FILE */
