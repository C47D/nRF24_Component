/**
* @file     `$INSTANCE_NAME`_HAL.h
* @version  3
*/

#ifndef `$INSTANCE_NAME`_HAL_H
#define `$INSTANCE_NAME`_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cytypes.h"

#if (CY_PSOC3)
#include <STDDEF.h>
#define uint8_t uint8
#define bool    uint8
#define size_t  uint32
#define true    1
#define false   0
#else
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#endif

#include "`$INSTANCE_NAME`_DEFS.h"

/* SPI */
void `$INSTANCE_NAME`_spi_xfer(const uint8_t *in, uint8_t *out, const size_t xfer_size);

bool `$INSTANCE_NAME`_read_bit(const nrf_register reg, const uint8_t bit);
void `$INSTANCE_NAME`_clear_bit(const nrf_register reg, const uint8_t bit_pos);
void `$INSTANCE_NAME`_set_bit(const nrf_register reg, const uint8_t bit_pos);

// 3.0
uint8_t `$INSTANCE_NAME`_read_reg(const nrf_register reg, uint8_t *data, const size_t data_size);
uint8_t `$INSTANCE_NAME`_write_reg(const nrf_register reg, const uint8_t *data, const size_t data_size);

/* GPIO Control */

typedef enum {
    GPIO_CLEAR,
    GPIO_SET
} nrf_gpio;

void `$INSTANCE_NAME`_ss_write(nrf_gpio state);
void `$INSTANCE_NAME`_ce_write(nrf_gpio state);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* `$INSTANCE_NAME`_HAL_H */

/* [] END OF FILE */
