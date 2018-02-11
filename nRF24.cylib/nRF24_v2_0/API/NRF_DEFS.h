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
* @file `$INSTANCE_NAME`_DEFS.h
*
* @brief This file contains all the nRF24 radio registers, commands and
* usefull enum types.
*/

#ifndef `$INSTANCE_NAME`_DEFS_H
#define `$INSTANCE_NAME`_DEFS_H

// nRF24L01P Registers
typedef enum {
    NRF_REG_CONFIG      = 0x00,
    NRF_REG_EN_AA       = 0x01,
    NRF_REG_EN_RXADDR   = 0x02,
    NRF_REG_SETUP_AW    = 0x03,
    NRF_REG_SETUP_RETR  = 0x04,
    NRF_REG_RF_CH       = 0x05,
    NRF_REG_RF_SETUP    = 0x06,
    NRF_REG_STATUS      = 0x07,
    NRF_REG_OBSERVE_TX  = 0x08,
    NRF_REG_RPD         = 0x09,
    NRF_REG_RX_ADDR_P0  = 0x0A,
    NRF_REG_RX_ADDR_P1  = 0x0B,
    NRF_REG_RX_ADDR_P2  = 0x0C,
    NRF_REG_RX_ADDR_P3  = 0x0D,
    NRF_REG_RX_ADDR_P4  = 0x0E,
    NRF_REG_RX_ADDR_P5  = 0x0F,
    NRF_REG_TX_ADDR     = 0x10,
    NRF_REG_RX_PW_P0    = 0x11,
    NRF_REG_RX_PW_P1    = 0x12,
    NRF_REG_RX_PW_P2    = 0x13,
    NRF_REG_RX_PW_P3    = 0x14,
    NRF_REG_RX_PW_P4    = 0x15,
    NRF_REG_RX_PW_P5    = 0x16,
    NRF_REG_FIFO_STATUS = 0x17,
    NRF_REG_DYNPD       = 0x1C,
    NRF_REG_FEATURE     = 0x1D,
} nrf_register;

// Command Name Mnemonics (Instructions)
typedef enum {
    NRF_CMD_R_REGISTER          = 0x00, // Read command
    NRF_CMD_W_REGISTER          = 0x20, // Write command
    NRF_CMD_R_RX_PAYLOAD        = 0x61, // Read Rx payload
    NRF_CMD_W_TX_PAYLOAD        = 0xA0, // Write Tx payload
    NRF_CMD_FLUSH_TX            = 0xE1, // Flush Tx FIFO
    NRF_CMD_FLUSH_RX            = 0xE2, // Flush Rx FIFO
    NRF_CMD_REUSE_TX_PL         = 0xE3, // Reuse the last transmitted payload
    NRF_CMD_R_RX_PL_WID         = 0x60, // Read Rx payload width for the top R_RX_PAYLOAD
    NRF_CMD_W_ACK_PAYLOAD       = 0xA8, // Write payload to be transmitted together with ACK packet
    NRF_CMD_W_TX_PAYLOAD_NOACK  = 0xB0, // Disable AUTOACK on this specific packet
    NRF_CMD_NOP                 = 0xFF, // No operation, read STATUS register
} nrf_cmd;

// Data pipes
typedef enum {
    NRF_PIPE0   = 0x00,
    NRF_PIPE1   = 0x01,
    NRF_PIPE2   = 0x02,
    NRF_PIPE3   = 0x03,
    NRF_PIPE4   = 0x04,
    NRF_PIPE5   = 0x05,
} nrf_pipe;

// Rx data pipes addresses
typedef enum {
    NRF_ADDR_PIPE0 = NRF_REG_RX_ADDR_P0,
    NRF_ADDR_PIPE1 = NRF_REG_RX_ADDR_P1,
    NRF_ADDR_PIPE2 = NRF_REG_RX_ADDR_P2,
    NRF_ADDR_PIPE3 = NRF_REG_RX_ADDR_P3,
    NRF_ADDR_PIPE4 = NRF_REG_RX_ADDR_P4,
    NRF_ADDR_PIPE5 = NRF_REG_RX_ADDR_P5,
} nrf_addr_rx_pipe;

typedef enum {
    NRF_PLD_SIZE_PIPE0  = NRF_REG_RX_PW_P0,
    NRF_PLD_SIZE_PIPE1  = NRF_REG_RX_PW_P1,
    NRF_PLD_SIZE_PIPE2  = NRF_REG_RX_PW_P2,
    NRF_PLD_SIZE_PIPE3  = NRF_REG_RX_PW_P3,
    NRF_PLD_SIZE_PIPE4  = NRF_REG_RX_PW_P4,
    NRF_PLD_SIZE_PIPE5  = NRF_REG_RX_PW_P5,
} nrf_pld_size;

// nRF24 Modes
typedef enum{
    NRF_MODE_TX = 0,
    NRF_MODE_RX = 1,
} nrf_mode;

typedef enum{
    NRF_NONE_IRQ    = 0, // 0x00
    NRF_MAX_RT_IRQ  = 4, // 0x10
    NRF_TX_DS_IRQ   = 5, // 0x20
    NRF_RX_DR_IRQ   = 6, // 0x40
} nrf_irq;

// CONFIG: Configuration Register
enum {
    NRF_CONFIG_BIT_PRIM_RX      = 0,
    NRF_CONFIG_BIT_PWR_UP       = 1,
    NRF_CONFIG_BIT_CRCO         = 2,
    NRF_CONFIG_BIT_EN_CRC       = 3,
    NRF_CONFIG_BIT_MASK_MAX_RT  = 4,
    NRF_CONFIG_BIT_MASK_TX_DS   = 5,
    NRF_CONFIG_BIT_MASK_RX_DR   = 6,
};

enum {
    NRF_CONFIG_TRANSMITTER      = (0 << NRF_CONFIG_BIT_PRIM_RX),
    NRF_CONFIG_RECEIVER         = (1 << NRF_CONFIG_BIT_PRIM_RX),
    NRF_CONFIG_PWR_DOWN         = (0 << NRF_CONFIG_BIT_PWR_UP),
    NRF_CONFIG_PWR_UP           = (1 << NRF_CONFIG_BIT_PWR_UP),
    NRF_CONFIG_1_BYTE_CRC       = (0 << NRF_CONFIG_BIT_CRCO),
    NRF_CONFIG_2_BYTE_CRC       = (1 << NRF_CONFIG_BIT_CRCO),
    NRF_CONFIG_DISABLE_CRC      = (0 << NRF_CONFIG_BIT_EN_CRC),
    NRF_CONFIG_ENABLE_CRC       = (1 << NRF_CONFIG_BIT_EN_CRC),
    NRF_CONFIG_ENABLE_MAX_RT    = (0 << NRF_CONFIG_BIT_MASK_MAX_RT),
    NRF_CONFIG_DISABLE_MAX_RT   = (1 << NRF_CONFIG_BIT_MASK_MAX_RT),
    NRF_CONFIG_ENABLE_TX_DS     = (0 << NRF_CONFIG_BIT_MASK_TX_DS),
    NRF_CONFIG_DISABLE_TX_DS    = (1 << NRF_CONFIG_BIT_MASK_TX_DS),
    NRF_CONFIG_ENABLE_RX_DR     = (0 << NRF_CONFIG_BIT_MASK_RX_DR),
    NRF_CONFIG_DISABLE_RX_DR    = (1 << NRF_CONFIG_BIT_MASK_RX_DR),
};

// EN_AA: Enable Enhanced ShockBurst
enum {
    NRF_EN_AA_BIT_ENAA_P0   = 0,
    NRF_EN_AA_BIT_ENAA_P1   = 1,
    NRF_EN_AA_BIT_ENAA_P2   = 2,
    NRF_EN_AA_BIT_ENAA_P3   = 3,
    NRF_EN_AA_BIT_ENAA_P4   = 4,
    NRF_EN_AA_BIT_ENAA_P5   = 5,
};

enum {
    NRF_DISABLE_AUTO_ACK_PIPE0  = (0 << NRF_EN_AA_BIT_ENAA_P0),
    NRF_ENABLE_AUTO_ACK_PIPE0   = (1 << NRF_EN_AA_BIT_ENAA_P0),
    NRF_DISABLE_AUTO_ACK_PIPE1  = (0 << NRF_EN_AA_BIT_ENAA_P1),
    NRF_ENABLE_AUTO_ACK_PIPE1   = (1 << NRF_EN_AA_BIT_ENAA_P1),
    NRF_DISABLE_AUTO_ACK_PIPE2  = (0 << NRF_EN_AA_BIT_ENAA_P2),
    NRF_ENABLE_AUTO_ACK_PIPE2   = (1 << NRF_EN_AA_BIT_ENAA_P2),
    NRF_DISABLE_AUTO_ACK_PIPE3  = (0 << NRF_EN_AA_BIT_ENAA_P3),
    NRF_ENABLE_AUTO_ACK_PIPE3   = (1 << NRF_EN_AA_BIT_ENAA_P3),
    NRF_DISABLE_AUTO_ACK_PIPE4  = (0 << NRF_EN_AA_BIT_ENAA_P4),
    NRF_ENABLE_AUTO_ACK_PIPE4   = (1 << NRF_EN_AA_BIT_ENAA_P4),
    NRF_DISABLE_AUTO_ACK_PIPE5  = (0 << NRF_EN_AA_BIT_ENAA_P5),
    NRF_ENABLE_AUTO_ACK_PIPE5   = (1 << NRF_EN_AA_BIT_ENAA_P5),
};

// EN_RXADDR: Enabled RX Addresses
enum {
    NRF_EN_RXADDR_BIT_ERX_P0    = 0,
    NRF_EN_RXADDR_BIT_ERX_P1    = 1,
    NRF_EN_RXADDR_BIT_ERX_P2    = 2,
    NRF_EN_RXADDR_BIT_ERX_P3    = 3,
    NRF_EN_RXADDR_BIT_ERX_P4    = 4,
    NRF_EN_RXADDR_BIT_ERX_P5    = 5,
};

enum {
    NRF_DISABLE_PIPE0   = (0 << NRF_EN_RXADDR_BIT_ERX_P0),
    NRF_ENABLE_PIPE0    = (1 << NRF_EN_RXADDR_BIT_ERX_P0),
    NRF_DISABLE_PIPE1   = (0 << NRF_EN_RXADDR_BIT_ERX_P1),
    NRF_ENABLE_PIPE1    = (1 << NRF_EN_RXADDR_BIT_ERX_P1),
    NRF_DISABLE_PIPE2   = (0 << NRF_EN_RXADDR_BIT_ERX_P2),
    NRF_ENABLE_PIPE2    = (1 << NRF_EN_RXADDR_BIT_ERX_P2),
    NRF_DISABLE_PIPE3   = (0 << NRF_EN_RXADDR_BIT_ERX_P3),
    NRF_ENABLE_PIPE3    = (1 << NRF_EN_RXADDR_BIT_ERX_P3),
    NRF_DISABLE_PIPE4   = (0 << NRF_EN_RXADDR_BIT_ERX_P4),
    NRF_ENABLE_PIPE4    = (1 << NRF_EN_RXADDR_BIT_ERX_P4),
    NRF_DISABLE_PIPE5   = (0 << NRF_EN_RXADDR_BIT_ERX_P5),
    NRF_ENABLE_PIPE5    = (1 << NRF_EN_RXADDR_BIT_ERX_P5),
};

// SETUP_AW: Setup of Address Width (common for all data pipes)
typedef enum {
    NRF_SETUP_AW_3BYTES = 1,
    NRF_SETUP_AW_4BYTES = 2,
    NRF_SETUP_AW_5BYTES = 3,
} nrf_addr_width;

// helper to use set and get pipe addresses
typedef enum {
    NRF_PIPE_ADDR_WIDTH_3BYTES = 3,
    NRF_PIPE_ADDR_WIDTH_4BYTES = 4,
    NRF_PIPE_ADDR_WIDTH_5BYTES = 5,
} nrf_pipe_addr_width;

// SETUP_RETR: Setup of Automatic Retransmission
enum {
    NRF_SETUP_RETR_BIT_ARC  = 0,
    NRF_SETUP_RETR_BIT_ARD  = 4,
};

enum {
    NRF_AUTO_RETRANSMIT_CNT_0   = (0x00 << NRF_SETUP_RETR_BIT_ARC),
    NRF_AUTO_RETRANSMIT_CNT_1   = (0x01 << NRF_SETUP_RETR_BIT_ARC),
    NRF_AUTO_RETRANSMIT_CNT_2   = (0x02 << NRF_SETUP_RETR_BIT_ARC),
    NRF_AUTO_RETRANSMIT_CNT_3   = (0x03 << NRF_SETUP_RETR_BIT_ARC),
    NRF_AUTO_RETRANSMIT_CNT_4   = (0x04 << NRF_SETUP_RETR_BIT_ARC),
    NRF_AUTO_RETRANSMIT_CNT_5   = (0x05 << NRF_SETUP_RETR_BIT_ARC),
    NRF_AUTO_RETRANSMIT_CNT_6   = (0x06 << NRF_SETUP_RETR_BIT_ARC),
    NRF_AUTO_RETRANSMIT_CNT_7   = (0x07 << NRF_SETUP_RETR_BIT_ARC),
    NRF_AUTO_RETRANSMIT_CNT_8   = (0x08 << NRF_SETUP_RETR_BIT_ARC),
    NRF_AUTO_RETRANSMIT_CNT_9   = (0x09 << NRF_SETUP_RETR_BIT_ARC),
    NRF_AUTO_RETRANSMIT_CNT_10  = (0x0A << NRF_SETUP_RETR_BIT_ARC),
    NRF_AUTO_RETRANSMIT_CNT_11  = (0x0B << NRF_SETUP_RETR_BIT_ARC),
    NRF_AUTO_RETRANSMIT_CNT_12  = (0x0C << NRF_SETUP_RETR_BIT_ARC),
    NRF_AUTO_RETRANSMIT_CNT_13  = (0x0D << NRF_SETUP_RETR_BIT_ARC),
    NRF_AUTO_RETRANSMIT_CNT_14  = (0x0E << NRF_SETUP_RETR_BIT_ARC),
    NRF_AUTO_RETRANSMIT_CNT_15  = (0x0F << NRF_SETUP_RETR_BIT_ARC),
};

enum {
    NRF_AUTO_RETRANSMIT_DELAY_250_US    = (0x00 << NRF_SETUP_RETR_BIT_ARD),
    NRF_AUTO_RETRANSMIT_DELAY_500_US    = (0x01 << NRF_SETUP_RETR_BIT_ARD),
    NRF_AUTO_RETRANSMIT_DELAY_750_US    = (0x02 << NRF_SETUP_RETR_BIT_ARD),
    NRF_AUTO_RETRANSMIT_DELAY_1000_US   = (0x03 << NRF_SETUP_RETR_BIT_ARD),
    NRF_AUTO_RETRANSMIT_DELAY_1250_US   = (0x04 << NRF_SETUP_RETR_BIT_ARD),
    NRF_AUTO_RETRANSMIT_DELAY_1500_US   = (0x05 << NRF_SETUP_RETR_BIT_ARD),
    NRF_AUTO_RETRANSMIT_DELAY_1750_US   = (0x06 << NRF_SETUP_RETR_BIT_ARD),
    NRF_AUTO_RETRANSMIT_DELAY_2000_US   = (0x07 << NRF_SETUP_RETR_BIT_ARD),
    NRF_AUTO_RETRANSMIT_DELAY_2250_US   = (0x08 << NRF_SETUP_RETR_BIT_ARD),
    NRF_AUTO_RETRANSMIT_DELAY_2500_US   = (0x09 << NRF_SETUP_RETR_BIT_ARD),
    NRF_AUTO_RETRANSMIT_DELAY_2750_US   = (0x0A << NRF_SETUP_RETR_BIT_ARD),
    NRF_AUTO_RETRANSMIT_DELAY_3000_US   = (0x0B << NRF_SETUP_RETR_BIT_ARD),
    NRF_AUTO_RETRANSMIT_DELAY_3250_US   = (0x0C << NRF_SETUP_RETR_BIT_ARD),
    NRF_AUTO_RETRANSMIT_DELAY_3500_US   = (0x0D << NRF_SETUP_RETR_BIT_ARD),
    NRF_AUTO_RETRANSMIT_DELAY_3750_US   = (0x0E << NRF_SETUP_RETR_BIT_ARD),
    NRF_AUTO_RETRANSMIT_DELAY_4000_US   = (0x0F << NRF_SETUP_RETR_BIT_ARD),
};

// RF_CH: RF Channel the radio will work on, there are 125 available channels.

// RF_SETUP: RF Setup Register
enum {
    NRF_RF_SETUP_BIT_RF_PWR     = 1,
    NRF_RF_SETUP_BIT_RF_DR      = 3,
    NRF_RF_SETUP_BIT_RF_DR_HIGH = 3,
    NRF_RF_SETUP_BIT_PLL_LOCK   = 4,
    NRF_RF_SETUP_BIT_RF_DR_LOW  = 5,
    NRF_RF_SETUP_BIT_CONT_WAVE  = 7,
};

enum {
    NRF_RF_SETUP_RF_DR_1000 = 0,
    NRF_RF_SETUP_RF_DR_2000 = 8,
    NRF_RF_SETUP_RF_DR_250  = 0x20,
};

enum {
    NRF_RF_SETUP_RF_PWR_18  = 0,
    NRF_RF_SETUP_RF_PWR_12  = 2,
    NRF_RF_SETUP_RF_PWR_6   = 4,
    NRF_RF_SETUP_RF_PWR_0   = 6,
};

// STATUS: Status Register
enum {
    NRF_STATUS_BIT_TX_FULL  = 0,
    NRF_STATUS_BIT_RX_P_NO  = 1,
    NRF_STATUS_BIT_MAX_RT   = 4,
    NRF_STATUS_BIT_TX_DS    = 5,
    NRF_STATUS_BIT_RX_DR    = 6,
};

enum {
    NRF_STATUS_MAX_RT_MASK  = 0x10,
    NRF_STATUS_TX_DS_MASK   = 0x20,
    NRF_STATUS_RX_DR_MASK   = 0x40,
};

enum {
    NRF_STATUS_TXFIFO_NOTFULL   = 0,
    NRF_STATUS_TXFIFO_FULL      = 1,
};

enum {
    NRF_STATUS_RX_P_NO_0 = 0,
    NRF_STATUS_RX_P_NO_1 = 1,
    NRF_STATUS_RX_P_NO_2 = 2,
    NRF_STATUS_RX_P_NO_3 = 3,
    NRF_STATUS_RX_P_NO_4 = 4,
    NRF_STATUS_RX_P_NO_5 = 5,
    NRF_STATUS_RX_P_NO_UNUSED   = 0x0C,
    NRF_STATUS_RX_P_NO_EMPTY    = 0x0E,
};

// OBSERVE TX: Transmit observe register
enum {
    NRF_OBSERVE_TX_BIT_ARC_CNT  = 0,
    NRF_OBSERVE_TX_BIT_PLOS_CNT = 4,
};

enum {
    NRF_OBSERVE_TX_ARC_CNT_MASK     = 0x0F,
    NRF_OBSERVE_TX_PLOS_CNT_MASK    = 0xF0,
};

// RPD: Received Power Detector
enum {
    NRF_RPD_BIT_RPD = 0,
};

// SETUP_AW: Setup of Address Widths(common for all data pipes)
enum {
    NRF_RX_PW_MASK  = 0x3F,
};

// FIFO_STATUS: FIFO Status Register
enum {
    NRF_FIFO_STATUS_BIT_RX_EMPTY    = 0,
    NRF_FIFO_STATUS_BIT_RX_FULL     = 1,
    NRF_FIFO_STATUS_BIT_TX_EMPTY    = 4,
    NRF_FIFO_STATUS_BIT_TX_FULL     = 5,
    NRF_FIFO_STATUS_BIT_TX_REUSE    = 6,
};

enum {
    NRF_FIFO_STATUS_RX_EMPTY    = (1 << NRF_FIFO_STATUS_BIT_RX_EMPTY),
    NRF_FIFO_STATUS_RX_FULL     = (1 << NRF_FIFO_STATUS_BIT_RX_FULL),
    NRF_FIFO_STATUS_TX_EMPTY    = (1 << NRF_FIFO_STATUS_BIT_TX_EMPTY),
    NRF_FIFO_STATUS_TX_FULL     = (1 << NRF_FIFO_STATUS_BIT_TX_FULL),
    NRF_FIFO_STATUS_TX_REUSE    = (1 << NRF_FIFO_STATUS_BIT_TX_REUSE),
};

// DYNPD: Enable dynamic payload length
enum {
    NRF_DYNPD_BIT_DPL_P0    = 0,
    NRF_DYNPD_BIT_DPL_P1    = 1,
    NRF_DYNPD_BIT_DPL_P2    = 2,
    NRF_DYNPD_BIT_DPL_P3    = 3,
    NRF_DYNPD_BIT_DPL_P4    = 4,
    NRF_DYNPD_BIT_DPL_P5    = 5,
};

enum {
    NRF_DISABLE_DYN_PAYLOAD_LEN_P0  = (0 << NRF_DYNPD_BIT_DPL_P0),
    NRF_ENABLE_DYN_PAYLOAD_LEN_P0   = (1 << NRF_DYNPD_BIT_DPL_P0),
    NRF_DISABLE_DYN_PAYLOAD_LEN_P1  = (0 << NRF_DYNPD_BIT_DPL_P1),
    NRF_ENABLE_DYN_PAYLOAD_LEN_P1   = (1 << NRF_DYNPD_BIT_DPL_P1),
    NRF_DISABLE_DYN_PAYLOAD_LEN_P2  = (0 << NRF_DYNPD_BIT_DPL_P2),
    NRF_ENABLE_DYN_PAYLOAD_LEN_P2   = (1 << NRF_DYNPD_BIT_DPL_P2),
    NRF_DISABLE_DYN_PAYLOAD_LEN_P3  = (0 << NRF_DYNPD_BIT_DPL_P3),
    NRF_ENABLE_DYN_PAYLOAD_LEN_P3   = (1 << NRF_DYNPD_BIT_DPL_P3),
    NRF_DISABLE_DYN_PAYLOAD_LEN_P4  = (0 << NRF_DYNPD_BIT_DPL_P4),
    NRF_ENABLE_DYN_PAYLOAD_LEN_P4   = (1 << NRF_DYNPD_BIT_DPL_P4),
    NRF_DISABLE_DYN_PAYLOAD_LEN_P5  = (0 << NRF_DYNPD_BIT_DPL_P5),
    NRF_ENABLE_DYN_PAYLOAD_LEN_P5   = (1 << NRF_DYNPD_BIT_DPL_P5),
};

// FEATURE: Feature Register
enum {
    NRF_FEATURE_BIT_EN_DYN_ACK  = 0,
    NRF_FEATURE_BIT_EN_ACK_PAY  = 1,
    NRF_FEATURE_BIT_EN_DPL      = 2,
};

enum {
    NRF_DISABLE_W_TX_PAYLOAD_CMD    = (0 << NRF_FEATURE_BIT_EN_DYN_ACK),
    NRF_ENABLE_W_TX_PAYLOAD_CMD     = (1 << NRF_FEATURE_BIT_EN_DYN_ACK),
    NRF_DISABLE_PAYLOAD_WITH_ACK    = (0 << NRF_FEATURE_BIT_EN_ACK_PAY),
    NRF_ENABLE_PAYLOAD_WITH_ACK     = (1 << NRF_FEATURE_BIT_EN_ACK_PAY),
    NRF_DISABLE_DYN_PAYLOAD_LEN     = (0 << NRF_FEATURE_BIT_EN_DPL),
    NRF_ENABLE_DYN_PAYLOAD_LEN      = (1 << NRF_FEATURE_BIT_EN_DPL),
};

// return status, based in cyret
typedef enum {
    STATUS_SUCCESS         = 0x00, // Successful
    STATUS_BAD_PARAM       = 0x01, // One or more invalid parameters
    STATUS_STARTED         = 0x02, // Operation started, but not necessarily completed yet
    STATUS_FINISHED        = 0x03, // Operation completed
    STATUS_CANCELED        = 0x04, // Operation canceled
    STATUS_TIMEOUT         = 0x05, // Operation timed out
    STATUS_NULL_PTR        = 0x06, // NULL pointer was used as an parameter.
} status;

// MISC enums
enum {
    NRF_STATUS_PIPES_SHIFT  = 1,
    NRF_CE_PULSE_WIDTH_US   = 15,
    NRF_PAYLOAD_SIZE_MAX    = 32,
    NRF_POWER_UP_DELAY_MS   = 100,
    NRF_MAX_RF_CHANNEL      = 125,
    NRF_ALL_IRQ_MASK        = 0x70,
    NRF_STATUS_PIPES_MASK   = 0x0E,
};

#endif /* `$INSTANCE_NAME`_DEFS_H */

/* [] END OF FILE */
