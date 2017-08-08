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

#ifndef `$INSTANCE_NAME`_FUNCS_H
#define `$INSTANCE_NAME`_FUNCS_H

#include <cytypes.h>
#include <cyfitter.h>

#include <stdbool.h>
#include <stddef.h>

#include "`$INSTANCE_NAME`_REGS.h"

// PSoC Component Functions
void `$INSTANCE_NAME`_start(void);
void `$INSTANCE_NAME`_init(void);
void `$INSTANCE_NAME`_enable(void);
void `$INSTANCE_NAME`_stop(void);
void `$INSTANCE_NAME`_sleep(void);
void `$INSTANCE_NAME`_wakeup(void);
void `$INSTANCE_NAME`_saveConfig(void);
void `$INSTANCE_NAME`_restoreConfig(void);

// nRF24 Functions

// Configuration functions
void `$INSTANCE_NAME`_setMode(const NrfMode mode);
void `$INSTANCE_NAME`_setPowerDownMode(void);
void `$INSTANCE_NAME`_setStandbyIMode(void);
void `$INSTANCE_NAME`_setStandbyIIMode(void);
void `$INSTANCE_NAME`_setRxMode(void);
void `$INSTANCE_NAME`_setTxMode(void);

// CONFIG Reg
// EN_AA_Reg
void `$INSTANCE_NAME`_enableAutoACK(const NrfDataPipe pipe);
void `$INSTANCE_NAME`_disableAutoACK(const NrfDataPipe pipe);
// EN_RXADDR Reg
// SETUP_AW Reg
// SETUP_RETR Reg
// RF_CH Reg
void `$INSTANCE_NAME`_setChannel(uint8_t channel);
// RF_SETUP Reg
// STATUS Reg

// RX ADDR P0 Reg
void `$INSTANCE_NAME`_setRxAddress(const uint8_t* addr, size_t size);
void `$INSTANCE_NAME`_getRxAddress(const uint8_t* addr, size_t size);
void `$INSTANCE_NAME`_setRxPipe0Address(const uint8_t* addr, size_t size);
void `$INSTANCE_NAME`_getRxPipe0Address(const uint8_t* addr, size_t size);
// RX ADDR P1 Reg
void `$INSTANCE_NAME`_setRxPipe1Address(const uint8_t* addr, size_t size);
void `$INSTANCE_NAME`_getRxPipe1Address(const uint8_t* addr, size_t size);
// RX ADDR P2 Reg
void `$INSTANCE_NAME`_setRxPipe2Address(const uint8_t addr);
void `$INSTANCE_NAME`_getRxPipe2Address(const uint8_t* addr, size_t size);
// RX ADDR P3 Reg
void `$INSTANCE_NAME`_setRxPipe3Address(const uint8_t addr);
void `$INSTANCE_NAME`_getRxPipe3Address(const uint8_t* addr, size_t size);
// RX ADDR P4 Reg
void `$INSTANCE_NAME`_setRxPipe4Address(const uint8_t addr);
void `$INSTANCE_NAME`_getRxPipe4Address(const uint8_t* addr, size_t size);
// RX ADDR P5 Reg
void `$INSTANCE_NAME`_setRxPipe5Address(const uint8_t addr);
void `$INSTANCE_NAME`_getRxPipe5Address(const uint8_t* addr, size_t size);
// TX_ADDR Reg
void `$INSTANCE_NAME`_setTxAddress(const uint8_t* addr, size_t size);
void `$INSTANCE_NAME`_getTxAddress(const uint8_t* addr, size_t size);
// RX_PW_P0 Reg
// RX_PW_P1 Reg
// RX_PW_P2 Reg
// RX_PW_P3 Reg
// RX_PW_P4 Reg
// RX_PW_P5 Reg
void `$INSTANCE_NAME`_setPayloadSize(const NrfDataPipe pipe, uint8_t size);
uint8_t `$INSTANCE_NAME`_getPayloadSize(const NrfDataPipe pipe);
// FIFO_STATUS Reg
void `$INSTANCE_NAME`_PTX_reuseLastTransmittedPayload(void);
// DYNPD Reg
void `$INSTANCE_NAME`_enableDynamicPayload(const NrfDataPipe pipe);
void `$INSTANCE_NAME`_disableDynamicPayload(const NrfDataPipe pipe);
// FEATURE Reg
void `$INSTANCE_NAME`_enableDynamicPayloadLength(void);
void `$INSTANCE_NAME`_enablePayloadWithACK(void);
void `$INSTANCE_NAME`_enablePayloadWithNoACKCmd(void);
void `$INSTANCE_NAME`_disableDynamicPayloadLength(void);
void `$INSTANCE_NAME`_disablePayloadWithACK(void);
void `$INSTANCE_NAME`_disablePayloadWithNoACKCmd(void);

// Data transfer functions
// TODO:
// We need to somehow distinguish what functions belong to 
// receiver and transmitter radios.
// `$INSTANCE_NAME`_PRX_x for receiver radio
// `$INSTANCE_NAME`_PTX_x for transmitter radio
// Functions that can be used from both modes will not have that distinction.
// Function naming must be clear enough to make documentation almost redundant,
// anyways the functions will be documented on the component datasheet.

void `$INSTANCE_NAME`_startListening(void);
void `$INSTANCE_NAME`_stopListening(void);
void `$INSTANCE_NAME`_transmitPulse(void);
void `$INSTANCE_NAME`_listen(const bool listen);
uint8_t `$INSTANCE_NAME`_getStatus(void);
uint8_t `$INSTANCE_NAME`_getRetransmissionsCount(void);
uint8_t `$INSTANCE_NAME`_getLostPacketsCount(void);
void `$INSTANCE_NAME`_fillTxFIFO(const uint8_t* data, size_t size);
void `$INSTANCE_NAME`_txTransmit(const uint8_t* data, size_t size);
bool `$INSTANCE_NAME`_isDataReady(void);
void `$INSTANCE_NAME`_getRxPayload(uint8_t* payload, const size_t size);
void `$INSTANCE_NAME`_txTransmitWaitNoACK(const uint8_t* data, size_t size);
void `$INSTANCE_NAME`_rxWritePayload(const NrfDataPipe pipe, uint8_t* data, size_t size);
uint8_t `$INSTANCE_NAME`_getDataPipeWithPayload(void);
uint8_t `$INSTANCE_NAME`_PRX_receivedPowerDetector(void);
uint8_t `$INSTANCE_NAME`_isTXFIFOFull(void);
uint8_t `$INSTANCE_NAME`_isTXFIFOEmpty(void);
uint8_t `$INSTANCE_NAME`_isRXFIFOFull(void);
uint8_t `$INSTANCE_NAME`_isRXFIFOEmpty(void);

// IRQ Handle functions
void `$INSTANCE_NAME`_clearIRQs(void);
void `$INSTANCE_NAME`_clearIRQFlag(const NrfIRQ irq_flag);
NrfIRQ `$INSTANCE_NAME`_getIRQFlag(void);

#endif /* `$INSTANCE_NAME`_FUNCS_H */

/* [] END OF FILE */
