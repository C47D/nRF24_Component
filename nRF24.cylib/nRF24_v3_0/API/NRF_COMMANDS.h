/**
* @file     `$INSTANCE_NAME`_COMMANDS.h
* @version  3
* @brief    The nRF24 radio is controlled via commands, this file implement all
* the available commands.
*/

#ifndef `$INSTANCE_NAME`_COMMANDS_H
#define `$INSTANCE_NAME`_COMMANDS_H
    
#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "`$INSTANCE_NAME`_DEFS.h"

void `$INSTANCE_NAME`_cmd_reuse_tx_payload(void);
void `$INSTANCE_NAME`_cmd_read_rx_payload(uint8_t* payload, const size_t payload_size);
void `$INSTANCE_NAME`_cmd_write_tx_payload(const uint8_t* payload, const size_t payload_size);
void `$INSTANCE_NAME`_cmd_flush_rx(void);
void `$INSTANCE_NAME`_cmd_flush_tx(void);
uint8_t `$INSTANCE_NAME`_cmd_read_payload_width(void);
void `$INSTANCE_NAME`_cmd_write_ack_payload(const nrf_pipe pipe, const uint8_t* payload, const size_t payload_size);
void `$INSTANCE_NAME`_cmd_no_ack_payload(const uint8_t* payload, const size_t payload_size);
uint8_t `$INSTANCE_NAME`_cmd_nop(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* `$INSTANCE_NAME`_NRF_COMMANDS_H */

/* [] END OF FILE */
