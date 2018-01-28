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

#ifndef `$INSTANCE_NAME`_CONFIG_H
#define `$INSTANCE_NAME`_CONFIG_H

#include "`$SPI_MASTER`.h"

#if defined (CY_SCB_`$SPI_MASTER`_H)
# if defined (`$SPI_MASTER`_CY_SCB_SPI_PDL_H)
#  define _PSOC6
# else
#  define _PSOC4_SCB
# endif
#else
# define _PSOC_UDB
#endif

#if defined (_PSOC6)
# include "gpio/cy_gpio.h"
#else // (_PSOC_UDB==1) || (_PSOC4_SCB==1)
# if defined (_PSOC4_SCB)
#  include "`$SPI_MASTER`_SPI_UART.h"
# endif
# include "CE.h"
# include "SS.h"
#endif

// Component version info
#define `$INSTANCE_NAME`_VERSION_MAJOR  `=$CY_MAJOR_VERSION`
#define `$INSTANCE_NAME`_VERSION_MINOR  `=$CY_MINOR_VERSION`
#define `$INSTANCE_NAME`_VERSION        `=$CY_MAJOR_VERSION`.`=$CY_MINOR_VERSION`

// Data from customizer
#define ENABLE_PIPE0    `@ERX_P0`
#define ENABLE_PIPE1    `@ERX_P1`
#define ENABLE_PIPE2    `@ERX_P2`
#define ENABLE_PIPE3    `@ERX_P3`
#define ENABLE_PIPE4    `@ERX_P4`
#define ENABLE_PIPE5    `@ERX_P5`

#define CUSTOMIZER_EN_AA        ((`$ENAA_P5` << NRF_EN_AA_ENAA_P5) | (`$ENAA_P4` << NRF_EN_AA_ENAA_P4) | \
                                (`$ENAA_P3` << NRF_EN_AA_ENAA_P3) | (`$ENAA_P2` << NRF_EN_AA_ENAA_P2) | \
                                (`$ENAA_P1` << NRF_EN_AA_ENAA_P1) | (`$ENAA_P0` << NRF_EN_AA_ENAA_P0))
#define CUSTOMIZER_EN_RXADDR    ((`$ERX_P5` << NRF_EN_RXADDR_ERX_P5) | (`$ERX_P4` << NRF_EN_RXADDR_ERX_P4) | \
                                (`$ERX_P3` << NRF_EN_RXADDR_ERX_P3) | (`$ERX_P2` << NRF_EN_RXADDR_ERX_P2) | \
                                (`$ERX_P1` << NRF_EN_RXADDR_ERX_P1) | (`$ERX_P0` << NRF_EN_RXADDR_ERX_P0))
#define CUSTOMIZER_SETUP_AW     (`$AW`)
#define CUSTOMIZER_SETUP_RETR   ((`$ARD` << NRF_SETUP_RETR_ARD) | (`$ARC` << NRF_SETUP_RETR_ARC))
#define CUSTOMIZER_RF_CH        (`$RF_CH`)
#define CUSTOMIZER_RF_SETUP     ((`$CONT_WAVE` << NRF_RF_SETUP_CONT_WAVE) | (`$RF_DATA_RATE` << NRF_RF_SETUP_RF_DR) | \
                                (`$RF_PWR` << NRF_RF_SETUP_RF_PWR))
#define CUSTOMIZER_DYNPD        ((`$DPL_P5` << NRF_DYNPD_DPL_P5) | (`$DPL_P4` << NRF_DYNPD_DPL_P4) | \
                                (`$DPL_P3` << NRF_DYNPD_DPL_P3) | (`$DPL_P2` << NRF_DYNPD_DPL_P2) | \
                                (`$DPL_P1` << NRF_DYNPD_DPL_P1) | (`$DPL_P0` << NRF_DYNPD_DPL_P0))
#define CUSTOMIZER_FEATURE      ((`$EN_DPL` << NRF_FEATURE_EN_DPL) | (`$EN_ACK_PAY` << NRF_FEATURE_EN_ACK_PAY) | \
                                (`$EN_DYN_ACK` << NRF_FEATURE_EN_DYN_ACK))
#define CUSTOMIZER_CONFIG       ((`$MASK_RX_DR` << NRF_CONFIG_MASK_RX_DR) | (`$MASK_TX_DS` << NRF_CONFIG_MASK_TX_DS) | \
                                (`$MASK_MAX_RT` << NRF_CONFIG_MASK_MAX_RT) | (`$EN_CRC` << NRF_CONFIG_EN_CRC) | \
                                (`$CRCO` << NRF_CONFIG_CRCO) | (`$PWR_UP`<< NRF_CONFIG_PWR_UP) | (`$PRIM_RX` << NRF_CONFIG_PRIM_RX))
#define CUSTOMIZER_RX_PW_P0     (`@RX_PW_P0`)
#define CUSTOMIZER_RX_PW_P1     (`@RX_PW_P1`)
#define CUSTOMIZER_RX_PW_P2     (`@RX_PW_P2`)
#define CUSTOMIZER_RX_ADDR_P2   (`@RX_ADDR_P2`)
#define CUSTOMIZER_RX_PW_P3     (`@RX_PW_P3`)
#define CUSTOMIZER_RX_ADDR_P3   (`@RX_ADDR_P3`)
#define CUSTOMIZER_RX_PW_P4     (`@RX_PW_P4`)
#define CUSTOMIZER_RX_ADDR_P4   (`@RX_ADDR_P4`)
#define CUSTOMIZER_RX_PW_P5     (`@RX_PW_P5`)
#define CUSTOMIZER_RX_ADDR_P5   (`@RX_ADDR_P5`)

#endif /* `$INSTANCE_NAME`_CONFIG_H */

/* [] END OF FILE */
