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
 * @file `$INSTANCE_NAME`_FUNCS.h
 *
 * @brief This file define all the functions available to the user.
 */

#ifndef `$INSTANCE_NAME`_FUNCS_H
#define `$INSTANCE_NAME`_FUNCS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "`$INSTANCE_NAME`_DEFS.h"

// PSoC Component Functions
void `$INSTANCE_NAME`_start(void);
void `$INSTANCE_NAME`_init(void);
void `$INSTANCE_NAME`_enable(void);
void `$INSTANCE_NAME`_stop(void);
void `$INSTANCE_NAME`_sleep(void);
void `$INSTANCE_NAME`_wakeup(void);
void `$INSTANCE_NAME`_save_config(void);
void `$INSTANCE_NAME`_restore_config(void);

// nRF24 Functions

// Configuration functions
void `$INSTANCE_NAME`_set_mode(const nrf_mode mode);
void `$INSTANCE_NAME`_set_power_down_mode(void);
void `$INSTANCE_NAME`_set_standby_i_mode(void);
void `$INSTANCE_NAME`_set_standby_ii_mode(void);
void `$INSTANCE_NAME`_set_rx_mode(void);
void `$INSTANCE_NAME`_set_tx_mode(void);

// Functions related to registers
void `$INSTANCE_NAME`_enable_auto_ack(const nrf_pipe pipe);
void `$INSTANCE_NAME`_disable_auto_ack(const nrf_pipe pipe);
void `$INSTANCE_NAME`_set_channel(uint8_t channel);
uint8_t `$INSTANCE_NAME`_get_channel(void);
void `$INSTANCE_NAME`_set_address_width(const nrf_addr_width addr_width);
uint8_t `$INSTANCE_NAME`_get_address_width(void);
void `$INSTANCE_NAME`_set_tx_address(const uint8_t *const addr, size_t size);
void `$INSTANCE_NAME`_get_tx_address(uint8_t *addr, size_t size);
void `$INSTANCE_NAME`_set_payload_size(const nrf_pld_size pipe,
                                          uint8_t size);
uint8_t `$INSTANCE_NAME`_get_payload_size(const nrf_pld_size pipe);
void `$INSTANCE_NAME`_enable_dynamic_payload(void);
void `$INSTANCE_NAME`_disable_dynamic_payload(void);
void `$INSTANCE_NAME`_enable_dynamic_payload_on_pipe(const nrf_pipe pipe);
void `$INSTANCE_NAME`_disable_dynamic_payload_on_pipe(const nrf_pipe pipe);
void `$INSTANCE_NAME`_enable_dynamic_payload_length(void);
void `$INSTANCE_NAME`_disable_dynamic_payload_length(void);
void `$INSTANCE_NAME`_enable_payload_with_ack(void);
void `$INSTANCE_NAME`_disable_payload_with_ack(void);
void `$INSTANCE_NAME`_enable_payload_with_no_ack(void);
void `$INSTANCE_NAME`_disable_payload_with_no_ack(void);

/* General purpose functions */
void `$INSTANCE_NAME`_start_listening(void);
void `$INSTANCE_NAME`_stop_listening(void);
void `$INSTANCE_NAME`_transmit_pulse(void);
uint8_t `$INSTANCE_NAME`_get_status(void);
uint8_t `$INSTANCE_NAME`_get_fifo_status(void);
uint8_t `$INSTANCE_NAME`_get_retransmissions_count(void);
uint8_t `$INSTANCE_NAME`_get_lost_packets_count(void);
void `$INSTANCE_NAME`_put_in_tx_fifo(const uint8_t *data, size_t size);
void `$INSTANCE_NAME`_transmit(const uint8_t *data, size_t size);
bool `$INSTANCE_NAME`_is_data_ready(void);
void `$INSTANCE_NAME`_get_rx_payload(uint8_t *payload, const size_t size);
void `$INSTANCE_NAME`_tx_transmit_no_ack(const uint8_t *data, size_t size);
void `$INSTANCE_NAME`_rx_write_payload(const nrf_pipe pipe, const uint8_t *data,
                                         size_t size);
uint8_t `$INSTANCE_NAME`_get_data_pipe_with_payload(void);
uint8_t `$INSTANCE_NAME`_received_power_detector(void);
bool `$INSTANCE_NAME`_is_tx_fifo_full(void);
bool `$INSTANCE_NAME`_is_rx_fifo_empty(void);
bool `$INSTANCE_NAME`_test_carrier(void);

// v2.0
void `$INSTANCE_NAME`_set_rx_pipe_address(const nrf_addr_rx_pipe pipe,
                                            const uint8_t *addr, size_t size);
void `$INSTANCE_NAME`_get_rx_pipe_address(const nrf_addr_rx_pipe pipe,
                                            uint8_t *addr, size_t size);

// IRQ Handle functions
void `$INSTANCE_NAME`_clear_all_irqs(void);
void `$INSTANCE_NAME`_clear_irq_flag(const nrf_irq irq_flag);
nrf_irq `$INSTANCE_NAME`_get_irq_flag(void);
void `$INSTANCE_NAME`_poll_interrupt(void);
uint8_t `$INSTANCE_NAME`_get_status_clear_irq(void);

// command wrappers
void `$INSTANCE_NAME`_flush_rx(void);
void `$INSTANCE_NAME`_flush_tx(void);
void `$INSTANCE_NAME`_reuse_last_transmitted_payload(void);

#endif /* `$INSTANCE_NAME`_FUNCS_H */

/* [] END OF FILE */
