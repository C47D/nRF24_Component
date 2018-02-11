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
* @file `$INSTANCE_NAME`_COMMANDS.h
*
* @brief The nRF24 radio is controlled via commands, this file implement all
* the available commands.
*/

#ifndef `$INSTANCE_NAME`_COMMANDS_H
#define `$INSTANCE_NAME`_COMMANDS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "`$INSTANCE_NAME`_DEFS.h"

void `$INSTANCE_NAME`_send_command(const nrf_cmd cmd);
void `$INSTANCE_NAME`_reuse_tx_payload_cmd(void);
void `$INSTANCE_NAME`_read_rx_payload_cmd(uint8_t* data, const size_t size);
void `$INSTANCE_NAME`_write_tx_payload_cmd(const uint8_t* data, const size_t size);
void `$INSTANCE_NAME`_flush_rx_cmd(void);
void `$INSTANCE_NAME`_flush_tx_cmd(void);
uint8_t `$INSTANCE_NAME`_read_payload_width_cmd(void);
void `$INSTANCE_NAME`_write_ack_payload_cmd(const nrf_pipe pipe, const uint8_t* data, const size_t size);
void `$INSTANCE_NAME`_no_ack_payload_cmd(const uint8_t* data, const size_t size);
uint8_t `$INSTANCE_NAME`_nop_cmd(void);

#endif /* `$INSTANCE_NAME`_NRF_COMMANDS_H */

/* [] END OF FILE */
