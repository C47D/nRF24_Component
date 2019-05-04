/**
* @file     `$INSTANCE_NAME`_HAL.h
* @version  3
*/

#ifndef `$INSTANCE_NAME`_HAL_H
#define `$INSTANCE_NAME`_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "`$INSTANCE_NAME`_DEFS.h"

/* SPI Control */

// Function pointer to send and receive data from the nrf24 radio
typedef void (*nrf_spi_xfer)(const uint8_t *in, uint8_t *out, const size_t xfer_size);

extern nrf_spi_xfer `$INSTANCE_NAME`_spi_xfer;

// Default function, spi sends byte by byte
void `$INSTANCE_NAME`_default_spi_xfer(const uint8_t *in, uint8_t *out, const size_t xfer_size);
// Register your own spi xfer function
void `$INSTANCE_NAME`_register_spi_xfer(nrf_spi_xfer new_spi_xfer);

uint8_t `$INSTANCE_NAME`_read_reg(const nrf_register reg, uint8_t *data, const size_t data_size);
uint8_t `$INSTANCE_NAME`_write_reg(const nrf_register reg, const uint8_t *data, const size_t data_size);
bool `$INSTANCE_NAME`_read_bit(const nrf_register reg, const uint8_t bit_pos);
void `$INSTANCE_NAME`_clear_bit(const nrf_register reg, const uint8_t bit_pos);
void `$INSTANCE_NAME`_set_bit(const nrf_register reg, const uint8_t bit_pos);

/* IO Control */
typedef enum {
    GPIO_CLEAR,
    GPIO_SET
} gpio_state;

void `$INSTANCE_NAME`_ss_write(gpio_state state);
void `$INSTANCE_NAME`_ce_write(gpio_state state);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* `$INSTANCE_NAME`_HAL_H */

/* [] END OF FILE */
