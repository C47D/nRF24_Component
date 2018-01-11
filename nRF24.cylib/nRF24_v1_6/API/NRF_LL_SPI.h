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

#ifndef `$INSTANCE_NAME`_LL_SPI_H
#define `$INSTANCE_NAME`_LL_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "`$INSTANCE_NAME`_REGS.h"

uint8_t `$INSTANCE_NAME`_read_register(const nrf_register reg);
void `$INSTANCE_NAME`_read_long_register(const nrf_register reg, uint8_t* data , const size_t size);
void `$INSTANCE_NAME`_write_register(const nrf_register reg, const uint8_t data);
void `$INSTANCE_NAME`_write_long_register(const nrf_register reg, const uint8_t* data, const size_t size);
bool `$INSTANCE_NAME`_read_bit(const nrf_register reg, const uint8_t bit);
void `$INSTANCE_NAME`_clear_bit(const nrf_register reg, const uint8_t bit);
void `$INSTANCE_NAME`_set_bit(const nrf_register reg, const uint8_t bit);

#endif /* `$INSTANCE_NAME`_LL_SPI_H */

/* [] END OF FILE */
