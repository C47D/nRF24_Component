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
* @file `$INSTANCE_NAME`_NRF_FUNCS.c
*
* @brief This file define all the functions available to the user.
*/

#include "`$INSTANCE_NAME`_CONFIG.h"
#include "`$INSTANCE_NAME`_LL_SPI.h"
#include "`$INSTANCE_NAME`_COMMANDS.h"
#include "`$INSTANCE_NAME`_FUNCS.h"
#include "`$INSTANCE_NAME`_REGS.h"

/* "Helper" global variables */
/* _nrf_addr_width expresed in bytes, declared with the default value */
static nrf_pipe_address_width _nrf_addr_width = NRF_PIPE_ADDR_WIDTH_5BYTES;

static void
_addr_width_conf_to_bytes(nrf_setup_address_width addr_width)
{
    switch(addr_width)
    {
    case NRF_SETUP_AW_3BYTES:
        _nrf_addr_width = NRF_PIPE_ADDR_WIDTH_3BYTES;
        break;
    case NRF_SETUP_AW_4BYTES:
        _nrf_addr_width = NRF_PIPE_ADDR_WIDTH_4BYTES;
        break;
    case NRF_SETUP_AW_5BYTES:
        _nrf_addr_width = NRF_PIPE_ADDR_WIDTH_5BYTES;
        break;
    }
}

/**
 * @brief Configure the radio and clear IRQs, TX and RX FIFOs.
 */
void `$INSTANCE_NAME`_start(void)
{
    // Recommended delay before start using the nRF24
    CyDelay(NRF_POWER_UP_DELAY);

    // Now the radio is in Power Down mode

    // Start the `$SPI_MASTER` and set CE and SS to a known value
#if defined (_PSOC6)
    /* We are using the low level driver, so pass NULL to the context */
    (void) Cy_SCB_`$SPI_MASTER`_Init(`$SPI_MASTER`_HW, &`$SPI_MASTER`_config, NULL);
    Cy_SCB_`$SPI_MASTER`_Enable(`$SPI_MASTER`_HW);
    Cy_GPIO_Clr(`$CE_PIN`_PORT, `$CE_PIN`_NUM);
    Cy_GPIO_Set(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
#else // PSoC 4 and 5LP
    `$SPI_MASTER`_Start();
    `$CE_PIN`_Write(0);
    `$SS_PIN`_Write(1);
#endif

    // Flush both nRF24 FIFOs
    `$INSTANCE_NAME`_flush_rx_cmd();
    `$INSTANCE_NAME`_flush_tx_cmd();
    // Clear IRQ flags
    `$INSTANCE_NAME`_clear_all_irqs();

    // Configure the nRF24 with the data from the customizer
    `$INSTANCE_NAME`_init();

    // After PWR_UP = 1 the radio is in Standby-I mode, 130us of delay for settling
    CyDelayUs(150);
}

/**
 * Configure the nRF radio with the data from the customizer.
 */
void `$INSTANCE_NAME`_init(void)
{
    `$INSTANCE_NAME`_write_register(NRF_EN_AA_REG, CUSTOMIZER_EN_AA);
    `$INSTANCE_NAME`_write_register(NRF_EN_RXADDR_REG, CUSTOMIZER_EN_RXADDR);
    `$INSTANCE_NAME`_write_register(NRF_SETUP_AW_REG, CUSTOMIZER_SETUP_AW);
    `$INSTANCE_NAME`_write_register(NRF_SETUP_RETR_REG, CUSTOMIZER_SETUP_RETR);
    `$INSTANCE_NAME`_write_register(NRF_RF_CH_REG, CUSTOMIZER_RF_CH);
    `$INSTANCE_NAME`_write_register(NRF_RF_SETUP_REG, CUSTOMIZER_RF_SETUP);
    `$INSTANCE_NAME`_write_register(NRF_DYNPD_REG, CUSTOMIZER_DYNPD);
    `$INSTANCE_NAME`_write_register(NRF_FEATURE_REG, CUSTOMIZER_FEATURE);

// Configuring data pipes
#if (ENABLE_PIPE0 == 1)
    `$INSTANCE_NAME`_write_register(NRF_RX_PW_P0_REG, CUSTOMIZER_RX_PW_P0);
#endif
#if (ENABLE_PIPE1 == 1)
    `$INSTANCE_NAME`_write_register(NRF_RX_PW_P1_REG, CUSTOMIZER_RX_PW_P1);
#endif
#if (ENABLE_PIPE2 == 1)
    `$INSTANCE_NAME`_write_register(NRF_RX_ADDR_P2_REG, CUSTOMIZER_RX_ADDR_P2);
    `$INSTANCE_NAME`_write_register(NRF_RX_PW_P2_REG, CUSTOMIZER_RX_PW_P2);
#endif
#if (ENABLE_PIPE3 == 1)
    `$INSTANCE_NAME`_write_register(NRF_RX_ADDR_P3_REG, CUSTOMIZER_RX_ADDR_P3);
    `$INSTANCE_NAME`_write_register(NRF_RX_PW_P3_REG, CUSTOMIZER_RX_PW_P3);
#endif
#if (ENABLE_PIPE4 == 1)
    `$INSTANCE_NAME`_write_register(NRF_RX_ADDR_P4_REG, CUSTOMIZER_RX_ADDR_P4);
    `$INSTANCE_NAME`_write_register(NRF_RX_PW_P4_REG, CUSTOMIZER_RX_PW_P4);
#endif
#if (ENABLE_PIPE5 == 1)
    `$INSTANCE_NAME`_write_register(NRF_RX_ADDR_P5_REG, CUSTOMIZER_RX_ADDR_P5);
    `$INSTANCE_NAME`_write_register(NRF_RX_PW_P5_REG, CUSTOMIZER_RX_PW_P5);
#endif

    `$INSTANCE_NAME`_write_register(NRF_CONFIG_REG, CUSTOMIZER_CONFIG);

    // Set the _nrf_addr_width variable
    _addr_width_conf_to_bytes(CUSTOMIZER_SETUP_AW);
}

/**
 * Enable the nRF24 radio.
 *
 * @todo Implement this function
 */
void `$INSTANCE_NAME`_enable(void)
{
}

/**
 * Stop the nRF24 radio.
 *
 * @todo Implement this function
 */
void `$INSTANCE_NAME`_stop(void)
{
}

/**
 * Put the nRF24 radio on Standby-I mode.
 * This function doesn't put the SPI, MCU nor pins to sleep
 */
void `$INSTANCE_NAME`_sleep(void)
{
    // is standby.I the lowest power mode?
    `$INSTANCE_NAME`_set_standby_i_mode();
}

/**
 * Wakeup the nRF24 radio, and restore the configuration.
 *
 * @todo Implement this function.
 */
void `$INSTANCE_NAME`_wakeup(void)
{
    // after leaving standby-I mode the radio need a time to
    // return to TX or RX Mode
    `$INSTANCE_NAME`_set_bit(NRF_CONFIG_REG, NRF_CONFIG_PWR_UP);
    CyDelay(5);
}

/**
 * Save the nRF24 radio configuration.
 *
 * @todo Implement this function.
 */
void `$INSTANCE_NAME`_save_config(void)
{
}

/**
 * Restore the nRF24 radio configuration.
 *
 * @todo Implement this function.
 */
void `$INSTANCE_NAME`_restore_config(void)
{
}

/**
 * Configure the radio as Receiver or Transmitter.
 *
 * @param const nrf_mode mode: The radio can be configured as Receiver (PRX)
 * or Transmitter (PTX).
 */
void `$INSTANCE_NAME`_set_mode(const nrf_mode mode)
{
    mode ? `$INSTANCE_NAME`_set_rx_mode() : `$INSTANCE_NAME`_set_tx_mode();
}

/**
 * @brief Set radio in Power Down Mode.
 *
 * In power down mode nRF24 is disabled using minimal current consumption.
 * All register values available are maintained and the SPI is kept active,
 * enabling change of configuration and the uploading/downloading of data
 * registers.
 * Power down mode is entered by setting the PWR_UP bit (CONFIG register) to 0.
 */
void `$INSTANCE_NAME`_set_power_down_mode(void)
{
    `$INSTANCE_NAME`_clear_bit(NRF_CONFIG_REG, NRF_CONFIG_PWR_UP);
}

/**
 * @brief Set the radio in Standby I Mode.
 *
 * By setting the PWR_UP bit in the CONFIG register to 1, the device enters
 * standby-I mode. Standby-I mode is used to minimize average current
 * consumption while maintaining short start up times.
 * In this mode only part of the crystal oscillator is active. Change to
 * active modes only happens if CE is set high and when CE is set low,
 * the nRF24 returns to standby-I mode from both the TX and RX modes.
 */
void `$INSTANCE_NAME`_set_standby_i_mode(void)
{
    `$INSTANCE_NAME`_stop_listening();
    `$INSTANCE_NAME`_set_bit(NRF_CONFIG_REG, NRF_CONFIG_PWR_UP);
}

/**
 * @brief Set the radio in Standby II Mode.
 *
 * In standby-II mode extra clock buffers are active and more current is used
 * compared to standby-I mode. nRF24 enters standby-II mode if CE is held high
 * on a TX device with an empty TX FIFO.
 * If a new packet is uploaded to the TX FIFO, the PLL immediately starts and
 * the packet is transmitted after the normal PLL settling  delay (130us).
 */
void `$INSTANCE_NAME`_set_standby_ii_mode(void)
{
    `$INSTANCE_NAME`_start_listening();
    `$INSTANCE_NAME`_set_bit(NRF_CONFIG_REG, NRF_CONFIG_PWR_UP);
}

/**
 * @brief Set the radio in Receiver (Rx) Mode.
 *
 * RX mode is an active mode where the nRF24 radio is used as receiver. To
 * enter this mode, the nRF24 must have the PWR_UP bit, PRIM_RX bit and the CE
 * pin set high.
 */
void `$INSTANCE_NAME`_set_rx_mode(void)
{
    `$INSTANCE_NAME`_set_bit(NRF_CONFIG_REG, NRF_CONFIG_PRIM_RX);
}

/**
 * @brief Set the radio in Transmitted (Tx) Mode.
 *
 * TX mode is an active mode for transmitting packets. To enter this mode,
 * the nRF24 must have the PWR_UP bit set high, PRIM_RX bit set low, a payload
 * in the TX FIFO and a high pulse on the CE for more than 10us.
 */
void `$INSTANCE_NAME`_set_tx_mode(void)
{
    `$INSTANCE_NAME`_clear_bit(NRF_CONFIG_REG, NRF_CONFIG_PRIM_RX);
}

/**
 * @brief Enable AutoACK in the given pipe.
 *
 * @param pipe: Enable AutoACK in the given pipe.
 */
void `$INSTANCE_NAME`_enable_auto_ack(const nrf_pipe pipe)
{
    `$INSTANCE_NAME`_set_bit(NRF_EN_AA_REG, pipe);
}

/**
 * @brief Disable AutoACK in the given pipe.
 *
 * @param pipe: Disable AutoACK in the given pipe.
 */
void `$INSTANCE_NAME`_disable_auto_ack(const nrf_pipe pipe)
{
    `$INSTANCE_NAME`_clear_bit(NRF_EN_AA_REG, pipe);
}

/**
 * @brief Set the channel where the radio will work.
 *
 * @param uint8_t channel: Channel where the radio will work.
 */
void `$INSTANCE_NAME`_set_channel(uint8_t channel)
{
    if (NRF_MAX_RF_CHANNEL < channel) {
        channel = NRF_MAX_RF_CHANNEL;
    }

    `$INSTANCE_NAME`_write_register(NRF_RF_CH_REG, channel);

    // Flush both nRF24 FIFOs, from mcuoneclipse nrf24 component
    `$INSTANCE_NAME`_flush_rx_cmd();
    `$INSTANCE_NAME`_flush_tx_cmd();
}

/**
 * @brief Set the data pipes address width.
 *
 * @param const nrf_setup_address_width addr_width:
 */
void `$INSTANCE_NAME`_set_address_width(const nrf_setup_address_width addr_width)
{
    `$INSTANCE_NAME`_write_register(NRF_SETUP_AW_REG, (uint8_t)addr_width);
    // update the _nrf_addr_width variable
    _addr_width_conf_to_bytes(addr_width);
}

/**
 * @brief Get the address width of the data pipes.
 *
 * @return uint8_t: Address width in bytes.
 */
uint8_t `$INSTANCE_NAME`_get_address_width(void)
{
    return _nrf_addr_width;
}

/**
 * @brief Set the address of the RX Pipe 0 in the radio.
 *
 * This function configure the address of the Rx Pipe 0 of the radio.
 *
 * @param const uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_set_rx_pipe_0_address(const uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (_nrf_addr_width < size) {
        size = _nrf_addr_width;
    }

    `$INSTANCE_NAME`_write_long_register(NRF_RX_ADDR_P0_REG, addr, size);
}

/**
 * @brief Get the address of the RX Pipe 0 in the radio.
 *
 * @param uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_get_rx_pipe_0_address(uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (_nrf_addr_width < size) {
        size = _nrf_addr_width;
    }

    `$INSTANCE_NAME`_read_long_register(NRF_RX_ADDR_P0_REG, addr, size);
}

/**
 * @brief Set the address of the RX Pipe 1 in the radio.
 *
 * This function configure the address of the Rx Pipe 1 of the radio.
 *
 * @param const uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_set_rx_pipe_1_address(const uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (_nrf_addr_width < size) {
        size = _nrf_addr_width;
    }

    `$INSTANCE_NAME`_write_long_register(NRF_RX_ADDR_P1_REG, addr, size);
}

/**
 * @brief Get the address of the RX Pipe 1 in the radio.
 *
 * @param uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_get_rx_pipe_1_address(uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (_nrf_addr_width < size) {
        size = _nrf_addr_width;
    }

    `$INSTANCE_NAME`_read_long_register(NRF_RX_ADDR_P1_REG, addr, size);
}

/**
 * Set the address of the least significant byte of the Rx Pipe 2 in the radio.
 *
 * @param const uint8_t addr_lsb:
 */
void `$INSTANCE_NAME`_set_rx_pipe_2_address(const uint8_t addr_lsb)
{
    `$INSTANCE_NAME`_write_register(NRF_RX_ADDR_P2_REG, addr_lsb);
}

/**
 * Get the address of the Rx Pipe 2.
 *
 * @param uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_get_rx_pipe_2_address(uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (_nrf_addr_width < size) {
        size = _nrf_addr_width;
    }

    // The pipe2 address is the same as the pipe1 address except the LSB
    `$INSTANCE_NAME`_read_long_register(NRF_RX_ADDR_P1_REG, addr, size - 1);
    addr[size - 1] = `$INSTANCE_NAME`_read_register(NRF_RX_ADDR_P2_REG);
}

/**
 * Set the address of the least significant byte of the Rx Pipe 3 in the radio.
 *
 * @param const uint8_t addr_lsb:
 */
void `$INSTANCE_NAME`_set_rx_pipe_3_address(const uint8_t addr_lsb)
{
    `$INSTANCE_NAME`_write_register(NRF_RX_ADDR_P3_REG, addr_lsb);
}

/**
 * Get the address of the Rx Pipe 3.
 *
 * @param uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_get_rx_pipe_3_address(uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (_nrf_addr_width < size) {
        size = _nrf_addr_width;
    }

    // The pipe3 address is the same as the pipe1 address except the LSB
    `$INSTANCE_NAME`_read_long_register(NRF_RX_ADDR_P1_REG, addr, size - 1);
    addr[size - 1] = `$INSTANCE_NAME`_read_register(NRF_RX_ADDR_P3_REG);
}

/**
 * Set the address of the least significant byte of the Rx Pipe 4 in the radio.
 *
 * @param const uint8_t addr_lsb:
 */
void `$INSTANCE_NAME`_set_rx_pipe_4_address(const uint8_t addr_lsb)
{
    `$INSTANCE_NAME`_write_register(NRF_RX_ADDR_P4_REG, addr_lsb);
}

/**
 * Get the address of the Rx Pipe 4.
 *
 * @param uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_get_rx_pipe_4_address(uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (_nrf_addr_width < size) {
        size = _nrf_addr_width;
    }

    // The pipe4 address is the same as the pipe1 address except the LSB
    `$INSTANCE_NAME`_read_long_register(NRF_RX_ADDR_P1_REG, addr, size - 1);
    addr[size - 1] = `$INSTANCE_NAME`_read_register(NRF_RX_ADDR_P4_REG);
}

/**
 * Set the address of the least significant byte of the Rx Pipe 5 in the radio.
 *
 * @param const uint8_t addr_lsb:
 */
void `$INSTANCE_NAME`_set_rx_pipe_5_address(const uint8_t addr_lsb)
{
    `$INSTANCE_NAME`_write_register(NRF_RX_ADDR_P5_REG, addr_lsb);
}

/**
 * Get the address of the Rx Pipe 5.
 *
 * @param uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_get_rx_pipe_5_address(uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (_nrf_addr_width < size) {
        size = _nrf_addr_width;
    }

    // The pipe5 address is the same as the pipe1 address except the LSB
    `$INSTANCE_NAME`_read_long_register(NRF_RX_ADDR_P1_REG, addr, size - 1);
    addr[size - 1] = `$INSTANCE_NAME`_read_register(NRF_RX_ADDR_P5_REG);
}

/**
 * @brief Set the TX Address of the radio.
 *
 * @param const uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_set_tx_address(const uint8_t *const addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (_nrf_addr_width < size) {
        size = _nrf_addr_width;
    }

    `$INSTANCE_NAME`_write_long_register(NRF_TX_ADDR_REG, addr, size);
}

/**
 * @brief Get the TX Address of the radio.
 *
 * @param const uint8_t* addr:
 * @param const size_t size:
 */
void `$INSTANCE_NAME`_get_tx_address(uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (_nrf_addr_width < size) {
        size = _nrf_addr_width;
    }

    `$INSTANCE_NAME`_read_long_register(NRF_TX_ADDR_REG, addr, size);
}

/**
 * @brief Set the payload size of the given pipe.
 *
 * Configure the payload size of the given pipe.
 *
 * @param const nrf_pipe pipe:
 * @param uint8_t size:
 */
void `$INSTANCE_NAME`_set_payload_size(const nrf_pipe_payload_size pipe, uint8_t size)
{
    if (NRF_MAX_PAYLOAD_SIZE < size) {
        size = NRF_MAX_PAYLOAD_SIZE;
    }

    `$INSTANCE_NAME`_write_register(pipe, size);
}

/**
 * Get the payload size of the given pipe.
 *
 * @param const nrf_pipe pipe: Pipe to be read.
 *
 * @return uint8_t: Configured payload size of the given pipe.
 */
uint8_t `$INSTANCE_NAME`_get_payload_size(const nrf_pipe_payload_size pipe)
{
    return `$INSTANCE_NAME`_read_register(pipe);
}

/**
 * @brief Reuse last transmitted payload.
 *
 * This function issue the command ReuseTxPayload and then toggle the CE pin
 * to transmit the last transmitted payload.
 */
void `$INSTANCE_NAME`_reuse_last_transmitted_payload(void)
{
    `$INSTANCE_NAME`_reuse_tx_payload_cmd();
    `$INSTANCE_NAME`_transmit_pulse();
}

/**
 * @brief Enable dynamic payload on the given pipe.
 *
 * @param const nrf_pipe pipe:
 */
void `$INSTANCE_NAME`_enable_dynamic_payload(void)
{
    // TODO: Read the FEATURE register, check the needed values and set
    // the bits if not set already
    `$INSTANCE_NAME`_set_bit(NRF_FEATURE_REG, NRF_FEATURE_EN_ACK_PAY);
    `$INSTANCE_NAME`_set_bit(NRF_FEATURE_REG, NRF_FEATURE_EN_DPL);
}

/**
 * @brief Enable dynamic payload on the given pipe.
 *
 * @param const nrf_pipe pipe:
 */
void `$INSTANCE_NAME`_enable_dynamic_payload_on_pipe(const nrf_pipe pipe)
{
    `$INSTANCE_NAME`_set_bit(NRF_EN_AA_REG, pipe);
    `$INSTANCE_NAME`_set_bit(NRF_DYNPD_REG, pipe);
}

/**
 * @brief Disable dynamic payload on the given pipe.
 *
 * @param const nrf_pipe pipe:
 */
void `$INSTANCE_NAME`_disable_dynamic_payload()
{
    // TODO: Read the FEATURE register, check the needed values and clear
    // the bits if not clear already
    `$INSTANCE_NAME`_clear_bit(NRF_FEATURE_REG, NRF_FEATURE_EN_ACK_PAY);
    `$INSTANCE_NAME`_clear_bit(NRF_FEATURE_REG, NRF_FEATURE_EN_DPL);
}

/**
 * @brief Disable dynamic payload on the given pipe.
 *
 * @param const nrf_pipe pipe:
 */
void `$INSTANCE_NAME`_disable_dynamic_payload_on_pipe(const nrf_pipe pipe)
{
    `$INSTANCE_NAME`_clear_bit(NRF_EN_AA_REG, pipe);
    `$INSTANCE_NAME`_clear_bit(NRF_DYNPD_REG, pipe);
}

/**
 * @brief Enable dinamic payload length.
 */
void `$INSTANCE_NAME`_enable_dynamic_payload_length(void)
{
    `$INSTANCE_NAME`_set_bit(NRF_FEATURE_REG, NRF_FEATURE_EN_DPL);
}

/**
 * @brief Disable dynamic payload length.
 */
void `$INSTANCE_NAME`_disable_dynamic_payload_length(void)
{
    `$INSTANCE_NAME`_clear_bit(NRF_FEATURE_REG, NRF_FEATURE_EN_DPL);
}

/**
 * @brief Enable payload with ACK.
 */
void `$INSTANCE_NAME`_enable_payload_with_ack(void)
{
    `$INSTANCE_NAME`_set_bit(NRF_FEATURE_REG, NRF_FEATURE_EN_ACK_PAY);
}

/**
 * @brief Disable Payload with ACK.
 */
void `$INSTANCE_NAME`_disable_payload_with_ack(void)
{
    `$INSTANCE_NAME`_clear_bit(NRF_FEATURE_REG, NRF_FEATURE_EN_ACK_PAY);
}

/**
 * @brief Enable dynamic payload length.
 */
void `$INSTANCE_NAME`_enable_payload_with_no_ack_cmd(void)
{
    `$INSTANCE_NAME`_set_bit(NRF_FEATURE_REG, NRF_FEATURE_EN_DYN_ACK);
}

/**
 * @brief Disable Payload with no ACK.
 */
void `$INSTANCE_NAME`_disable_payload_with_no_ack_cmd(void)
{
    `$INSTANCE_NAME`_clear_bit(NRF_FEATURE_REG, NRF_FEATURE_EN_DYN_ACK);
}

/**
 * @brief The nRF24 radio will start listening.
 *
 * This function set the pin CE to logic high, this enable the radio for
 * listening.
 */
void `$INSTANCE_NAME`_start_listening(void)
{
#if defined (_PSOC6)
    Cy_GPIO_Set(`$CE_PIN`_PORT, `$CE_PIN`_NUM);
#else
    `$CE_PIN`_Write(1);
#endif
}

/**
 * @brief The nRF24 radio will stop listening.
 *
 * This function the pin CE of the nRF24 radio will be set to logic low,
 * this disable the radio for listening.
 */
void `$INSTANCE_NAME`_stop_listening(void)
{
#if defined (_PSOC6)
    Cy_GPIO_Clr(`$CE_PIN`_PORT, `$CE_PIN`_NUM);
#else
    `$CE_PIN`_Write(0);
#endif
}

/**
 * @brief Transmit pulse on the CE pin.
 *
 * With this function the CE pin of the nRF24 radio will have a pulse of 15us,
 * this pulse trigger a transmission of the content of the TX FIFO.
 */
void `$INSTANCE_NAME`_transmit_pulse(void)
{
#if defined (_PSOC6)
    Cy_GPIO_Set(`$CE_PIN`_PORT, `$CE_PIN`_NUM);
    CyDelayUs(NRF_CE_PULSE_WIDTH);
    Cy_GPIO_Clr(`$CE_PIN`_PORT, `$CE_PIN`_NUM);
#else
    `$CE_PIN`_Write(1);
    CyDelayUs(NRF_CE_PULSE_WIDTH);
    `$CE_PIN`_Write(0);
#endif
}

/**
 * @brief Get the STATUS register of the nRF24.
 *
 * @return uint8_t: STATUS register of the nRF24.
 */
uint8_t `$INSTANCE_NAME`_get_status(void)
{
    return `$INSTANCE_NAME`_nop_cmd();
}

uint8_t `$INSTANCE_NAME`_get_fifo_status(void)
{
    return `$INSTANCE_NAME`_read_register(NRF_FIFO_STATUS_REG);
}

/**
 * @brief Get the number of retransmissions.
 *
 * @return uint8_t: Retransmissions count.
 */
uint8_t `$INSTANCE_NAME`_get_retransmissions_count(void)
{
    uint8_t count = `$INSTANCE_NAME`_read_register(NRF_OBSERVE_TX_REG);
    return count & NRF_OBSERVE_TX_ARC_CNT_MASK;
}

/**
 * @brief Get the number of lost packets.
 *
 * @return uint8_t: Lost packets.
 */
uint8_t `$INSTANCE_NAME`_get_lost_packets_count(void)
{
    uint8_t lostPackets = `$INSTANCE_NAME`_read_register(NRF_OBSERVE_TX_REG);
    lostPackets = lostPackets & NRF_OBSERVE_TX_PLOS_CNT_MASK;
    return lostPackets >> NRF_OBSERVE_TX_PLOS_CNT;
}

/**
 * Put data into the TX FIFO wihout sending it.
 *
 * @param const uint8_t* data:
 * @param const size_t size:
 */
void `$INSTANCE_NAME`_put_in_tx_fifo(const uint8_t* data, size_t size)
{
    if (NULL == data) {
        return;
    }

    if (NRF_MAX_PAYLOAD_SIZE < size) {
        size = NRF_MAX_PAYLOAD_SIZE;
    }

    `$INSTANCE_NAME`_write_tx_payload_cmd(data, size);
}

/**
 * Put data in TX FIFO and transmit it.
 *
 * @param const uint8_t* data:
 * @param const size_t size:
 */
void `$INSTANCE_NAME`_transmit(const uint8_t* data, size_t size)
{
    if (NULL == data) {
        return;
    }

    if (NRF_MAX_PAYLOAD_SIZE < size) {
        size = NRF_MAX_PAYLOAD_SIZE;
    }

    `$INSTANCE_NAME`_put_in_tx_fifo(data, size);
    `$INSTANCE_NAME`_transmit_pulse();
}

/**
 * @brief
 *
 * @return bool: True if there's data ready.
 */
bool `$INSTANCE_NAME`_is_data_ready(void)
{
    return NRF_STATUS_RX_DR_MASK & `$INSTANCE_NAME`_get_status();
}

/**
 * @brief
 *
 * @param uint8_t* data:
 * @param const size_t size:
 */
void `$INSTANCE_NAME`_get_rx_payload(uint8_t *data, const size_t size)
{
    if (NULL == data) {
        return;
    }
#if defined (_PSOC6)
    Cy_GPIO_Clr(`$CE_PIN`_PORT, `$CE_PIN`_NUM);
    `$INSTANCE_NAME`_read_rx_payload_cmd(data, size);
    Cy_GPIO_Set(`$CE_PIN`_PORT, `$CE_PIN`_NUM);
#else
    `$CE_PIN`_Write(0);
    `$INSTANCE_NAME`_read_rx_payload_cmd(data, size);
    `$CE_PIN`_Write(1);
#endif
}

/**
 * @brief
 *
 * @param const uint8_t* data:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_tx_transmit_wait_no_ack(const uint8_t *data, size_t size)
{
    if (NULL == data) {
        return;
    }

    if (NRF_MAX_PAYLOAD_SIZE < size) {
        return;
    }

    `$INSTANCE_NAME`_no_ack_payload_cmd(data, size);
    `$INSTANCE_NAME`_transmit_pulse();
}

/**
 * @brief
 *
 * @param const nrf_pipe pipe:
 * @param const uint8_t* data:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_rx_write_payload(const nrf_pipe pipe,
                                        const uint8_t *data,
                                        size_t size)
{
    if (NULL == data) {
        return;
    }

    if (NRF_MAX_PAYLOAD_SIZE < size) {
        size = NRF_MAX_PAYLOAD_SIZE;
    }

    `$INSTANCE_NAME`_write_ack_payload_cmd(pipe, data, size);
}

/**
 * Return the pipe number with data.
 *
 * @return uint8_t:
 */
uint8_t `$INSTANCE_NAME`_get_data_pipe_with_payload(void)
{
    uint8_t pipe = `$INSTANCE_NAME`_read_register(NRF_STATUS_REG);
    return (pipe & NRF_STATUS_PIPES_MASK) >> NRF_STATUS_PIPES_SHIFT;
}

/**
 * @brief
 *
 * @return uint8_t:
 */
uint8_t `$INSTANCE_NAME`_received_power_detector(void)
{
    return `$INSTANCE_NAME`_read_bit(NRF_RPD_REG, NRF_RPD_RPD);
}

/**
 * @return bool: true if the TX FIFO is full, false if it have available
 * locations.
 */
bool `$INSTANCE_NAME`_is_tx_fifo_full(void)
{
    return `$INSTANCE_NAME`_read_bit(NRF_FIFO_STATUS_REG, NRF_FIFO_STATUS_TX_FULL);
}

/**
 * @return bool: true if the RX FIFO es empty, false if any pipe have data.
 */
bool `$INSTANCE_NAME`_is_rx_fifo_empty(void)
{
    return `$INSTANCE_NAME`_read_bit(NRF_FIFO_STATUS_REG, NRF_FIFO_STATUS_RX_EMPTY);
}

/**
 * Received Power Detector triggers at received power levels above -64dBm that
 * are present in the RF channel you receive on. If the received power is less
 * than -64dBm, RDP = 0.
 * The RPD can be read out at any time while nRF24L01+ is in receive mode.
 */
bool `$INSTANCE_NAME`_test_carrier(void)
{
    return `$INSTANCE_NAME`_read_bit(NRF_RPD_REG, NRF_RPD_RPD);
}

/**
 * This function will write 1 to all the three IRQ "flag" bits on the
 * STATUS register.
 */
void `$INSTANCE_NAME`_clear_all_irqs(void)
{
    `$INSTANCE_NAME`_write_register(NRF_STATUS_REG, NRF_ALL_IRQ_MASK);
}

/**
 * @brief Clears the specific IRQ flag.
 *
 * Clear the flag by writing 1 to the interrupt flag bit.
 *
 * @param nrf_irq irq_flag: Interrupt flag to clear.
 */
void `$INSTANCE_NAME`_clear_irq_flag(const nrf_irq irq_flag)
{
    `$INSTANCE_NAME`_write_register(NRF_STATUS_REG, (1 << irq_flag));
}

/**
 * This method is used to get if any of the IRQ "flag" bits on the STATUS
 * register is set to 1. This function returns 0 is none of the flag bits
 * is not set to 1.
 *
 * @return nrf_irq: Asserted bit of the interrupt flags.
 */
nrf_irq `$INSTANCE_NAME`_get_irq_flag(void)
{
    nrf_irq irq = NRF_NONE_IRQ;

    // Get the STATUS register
    uint8_t sts = `$INSTANCE_NAME`_nop_cmd();

    // We only care if bits 4, 5 or 6 are set, so we mask the STATUS with 0x0111_0000
    switch (sts & NRF_ALL_IRQ_MASK) {
    case NRF_STATUS_RX_DR_MASK:
        irq = NRF_RX_DR_IRQ;
        break;
    case NRF_STATUS_TX_DS_MASK:
        irq = NRF_TX_DS_IRQ;
        break;
    case NRF_STATUS_MAX_RT_MASK:
        irq = NRF_MAX_RT_IRQ;
        break;
    default:
        irq = NRF_ALL_IRQ_MASK;
        break;
    }

    return irq;
}

void `$INSTANCE_NAME`_poll_interrupt(void)
{

}

uint8_t `$INSTANCE_NAME`_get_status_clear_irq(void)
{
    uint8_t sts = `$INSTANCE_NAME`_nop_cmd();
    `$INSTANCE_NAME`_write_register(NRF_STATUS_REG, NRF_ALL_IRQ_MASK);

    return sts;
}

// command wrappers
void `$INSTANCE_NAME`_flush_rx(void)
{
    `$INSTANCE_NAME`_flush_rx_cmd();
}

void `$INSTANCE_NAME`_flush_tx(void)
{
    `$INSTANCE_NAME`_flush_tx_cmd();
}

/* [] END OF FILE */
