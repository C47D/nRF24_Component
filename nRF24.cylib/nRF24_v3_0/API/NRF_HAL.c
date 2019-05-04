/**
* @file     `$INSTANCE_NAME`_HAL.c
* @version  3
* @brief    The nRF24 radio is controlled via SPI, this file have all the SPI
* communication between the PSoC and the nRF24 radio.
*/

#include <stdint.h>
#include <string.h>

#include "`$INSTANCE_NAME`_CONFIG.h"
#include "`$INSTANCE_NAME`_HAL.h"

#if defined (_PSOC6)
# include "gpio/cy_gpio.h"
#else // (_PSOC_UDB) || (_PSOC4)
# if defined (_PSOC4_SCB)
#  include "`$SPI_MASTER`_SPI_UART.h" // may be no longer necessary
# endif
# include "`$CE_PIN`.h"
# include "`$SS_PIN`.h"
#endif

static void `$INSTANCE_NAME`_spi_clear_fifo(void);

nrf_spi_xfer `$INSTANCE_NAME`_spi_xfer = `$INSTANCE_NAME`_default_spi_xfer;

void `$INSTANCE_NAME`_register_spi_xfer(nrf_spi_xfer new_spi_xfer)
{
    `$INSTANCE_NAME`_spi_xfer = new_spi_xfer;
}

// xfer xfer_size bytes to the nrf24 radio and get back xfer_size bytes back
// from it.
// in[0] is always the NRF_COMMAND
// out[0] is always the STATUS_REGISTER
//
// Example of use:
// - To read the radio status register and save it in status_reg variable:
//
// uint8_t NRF_DUMMY = 0xFF;
// uint8_t status_reg = 0;
// nrf24_spi_xfer(&NRF_DUMMY, &status_reg, 1);
void `$INSTANCE_NAME`_default_spi_xfer(const uint8_t *in, uint8_t *out, const size_t xfer_size)
{
    `$INSTANCE_NAME`_spi_clear_fifo();

    `$INSTANCE_NAME`_ss_write(GPIO_CLEAR);

#if defined (_PSOC6)
    for(size_t i = 0; i < xfer_size; i++){
        while(Cy_SCB_Write(`$SPI_MASTER`_HW, in[i]) == 0);
        while (Cy_SCB_GetNumInRxFifo(`$SPI_MASTER`_HW) == 0) {}
        out[i] = Cy_SCB_ReadRxFifo(`$SPI_MASTER`_HW);
    }
#elif defined (_PSOC4_SCB)
    for (size_t i = 0; i < xfer_size; i++) {
        `$SPI_MASTER`_SpiUartWriteTxData(in[i]);
        while (`$SPI_MASTER`_SpiUartGetRxBufferSize() == 0){}
        out[i] = `$SPI_MASTER`_SpiUartReadRxData();
    }
#elif defined (_PSOC_UDB)
    for (size_t i = 0; i < xfer_size; i++) {
        `$SPI_MASTER`_WriteTxData(in[i]);
        while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_BYTE_COMPLETE)) {
        }
        out[i] = `$SPI_MASTER`_ReadRxData();
    }
#else
    #error "Non valid PSoC device identified."
#endif

    `$INSTANCE_NAME`_ss_write(GPIO_SET);
}

/**
 * Read the specified nRF24 register (bigger than 1 byte).
 *
 * @param[in]   reg: Register to be read, see @nrf_register.
 * @param[out]  data: Where the content of the register will be stored.
 * @param[int]  data_size: Size (in bytes) of the register and data.
 *
 * @return      status register.
 */
uint8_t `$INSTANCE_NAME`_read_reg(const nrf_register reg,
    uint8_t *data, const size_t data_size)
{
    uint8_t data_out[data_size + 1];
    uint8_t data_in[data_size + 1];
    
    data_in[0] = NRF_CMD_R_REGISTER | reg;
    `$INSTANCE_NAME`_spi_xfer(data_in, data_out, sizeof data_in/sizeof *data_in);
    memcpy(data, &data_out[1], data_size);
    
    // Return the NRF_STATUS register.
    return data_out[0];
}

/**
 * Read the specified nRF24 register (bigger than 1 byte).
 *
 * @param[in]   reg: Register to be read, see @nrf_register.
 * @param[in]   data: Data to be written to the register.
 * @param[int]  data_size: Size (in bytes) of the register and data.
 *
 * @return      status register.
 */
uint8_t `$INSTANCE_NAME`_write_reg(const nrf_register reg,
    const uint8_t *data, const size_t data_size)
{
    uint8_t data_out[data_size + 1];
    uint8_t data_in[data_size + 1];
    
    data_in[0] = NRF_CMD_W_REGISTER | reg;
    memcpy(&data_in[1], data, data_size);

    `$INSTANCE_NAME`_spi_xfer(data_in, data_out, sizeof data_in/sizeof *data_in);
    
    // Return the NRF_STATUS register.
    return data_out[0];
}

/**
 * Read the specified bit.
 *
 * @param[in] reg: Register to be read, see @c nrf_register.
 * @param[in] bit_pos: Bit position to be read.
 *
 * @return bool: Bit value.
 */
bool `$INSTANCE_NAME`_read_bit(const nrf_register reg, const uint8_t bit_pos)
{
    uint8_t reg_val;
    nRF24_read_reg(reg, &reg_val, 1);
    return (reg_val & (1 << bit_pos)) != 0;
}

/**
 * Set (1) or clear (0) the specified bit of the specified nRF24 register.
 *
 * First we read the specified register and check the content of the specified
 * bit, exit early if the bit already is the value we wanted, otherwise we set
 * the bit to the specified value.
 *
 * @param[in] reg: Register to be written, see @c nrf_register.
 * @param[in] bit_pos: Position of the bit to be written.
 * @param[in] value: Value (1 or 0) to write into the bit.
 */
static void `$INSTANCE_NAME`_write_bit(const nrf_register reg,
                                  const uint8_t bit_pos, const bool value)
{
    uint8_t temp;
    `$INSTANCE_NAME`_read_reg(reg, &temp, 1);
    
    const uint8_t bit_mask = 1 << bit_pos;
    
    // Read the bit value before writing to it.
    // Check if the bit is 1
    if ((temp & bit_mask) != 0) {
        // it is 1, return if we wanted to set it to 1
        if (value) {
            return;
        }
    } else { // the bit is 0
        // it is 0, return if we wanted to set it to 0
        if (!value) {
            return;
        }
    }

    // Calculate the new value to be written into the register
    temp = value ? temp | bit_mask : temp & ~bit_mask;

    `$INSTANCE_NAME`_write_reg(reg, &temp, 1);
}

/**
 * Set to 0 the specified bit of the specified nRF24 register.
 *
 * @param[in]   reg: Register to be written, see @c nrf_register.
 * @param[in]   bit_pos: Position of the bit to be written.
 */
void `$INSTANCE_NAME`_clear_bit(const nrf_register reg, const uint8_t bit_pos)
{
    if (8 < bit_pos) {
        return;
    }

    `$INSTANCE_NAME`_write_bit(reg, bit_pos, 0);
}

/**
 * Set to 1 the specified bit of the specified nRF24 register.
 *
 * @param[in]   reg: Register to be written, see @c nrf_register.
 * @param[in]   bit_pos: Position of the bit to be written.
 */
void `$INSTANCE_NAME`_set_bit(const nrf_register reg, const uint8_t bit_pos)
{
    if (8 < bit_pos) {
        return;
    }

    `$INSTANCE_NAME`_write_bit(reg, bit_pos, 1);
}

/* GPIO Control */

/**
 * GPIO control in PSoC6 and PSoC4/PSoC5LP is a little bit different,
 * so here we try to unify them.
 *
 * @param[in] nrf_gpio state 
 */
void `$INSTANCE_NAME`_ss_write(gpio_state state)
{
    if (GPIO_CLEAR == state) {
#if defined (_PSOC6)
    Cy_GPIO_Clr(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
#else // _PSoC4_SCB | _PSOC_UDB
    `$SS_PIN`_Write(0);
#endif
    } else { // GPIO_SET
#if defined (_PSOC6)
    Cy_GPIO_Set(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
#else // _PSoC4_SCB | _PSOC_UDB
    `$SS_PIN`_Write(1);
#endif
    }
}

void `$INSTANCE_NAME`_ce_write(gpio_state state)
{
    if (GPIO_CLEAR == state) {
#if defined (_PSOC6)
    Cy_GPIO_Clr(`$CE_PIN`_PORT, `$CE_PIN`_NUM);
#else // _PSoC4_SCB | _PSOC_UDB
    `$CE_PIN`_Write(0);
#endif
    } else { // GPIO_SET
#if defined (_PSOC6)
    Cy_GPIO_Set(`$CE_PIN`_PORT, `$CE_PIN`_NUM);
#else // _PSoC4_SCB | _PSOC_UDB
    `$CE_PIN`_Write(1);
#endif
    }
}

static void `$INSTANCE_NAME`_spi_clear_fifo(void)
{
#if defined (_PSOC6)
    Cy_SCB_ClearRxFifo(`$SPI_MASTER`_HW);
    Cy_SCB_ClearTxFifo(`$SPI_MASTER`_HW);
#elif defined (_PSOC4_SCB)
    `$SPI_MASTER`_SpiUartClearRxBuffer();
    `$SPI_MASTER`_SpiUartClearTxBuffer();
#elif defined (_PSOC_UDB)
    `$SPI_MASTER`_ClearFIFO();
#else
    #error "Non valid PSoC device identified."
#endif
}

/* [] END OF FILE */
