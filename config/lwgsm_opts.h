/**
 * \file            lwgsm_opts_template.h
 * \brief           Template config file
 */

/*
 * Copyright (c) 2020 Tilen MAJERLE
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of LwGSM - Lightweight GSM-AT library.
 *
 * Author:          Tilen MAJERLE <tilen@majerle.eu>
 * Version:         v0.1.0
 */
#ifndef LWGSM_HDR_OPTS_H
#define LWGSM_HDR_OPTS_H

/* Rename this file to "lwgsm_opts.h" for your application */

/*
 * Open "include/lwgsm/lwgsm_opt.h" and
 * copy & replace here settings you want to change values
 */

#define LWGSM_CFG_AT_ECHO                     0
#define LWGSM_CFG_INPUT_USE_PROCESS           1
#define LWGSM_CFG_MEM_CUSTOM                  1

/* Enable network, conn and netconn APIs */
#define LWGSM_CFG_NETWORK                     1
#define LWGSM_CFG_CONN                        1
#define LWGSM_CFG_NETCONN                     1

/* Enable hardware reset */
// #define LWGSM_RESET_PIN             19

/* LWGSM DEBUGGING LEVEL */
#define LWGSM_CFG_DBG               LWGSM_DBG_OFF
#define LWGSM_CFG_DBG_LVL_MIN       LWGSM_DBG_LVL_WARNING
#define LWGSM_CFG_DBG_TYPES_ON      LWGSM_DBG_TYPE_ALL

#define LWGSM_CFG_DBG_INIT          LWGSM_DBG_OFF
#define LWGSM_CFG_DBG_THREAD        LWGSM_DBG_OFF
#define LWGSM_CFG_DBG_INPUT         LWGSM_DBG_OFF
#define LWGSM_CFG_DBG_ASSERT        LWGSM_DBG_OFF
#define LWGSM_CFG_DBG_NETCONN       LWGSM_DBG_OFF
#define LWGSM_CFG_DBG_CONN          LWGSM_DBG_OFF
#define LWGSM_CFG_DBG_IPD           LWGSM_DBG_OFF
#define LWGSM_CFG_DBG_PBUF          LWGSM_DBG_OFF
#define LWGSM_CFG_DBG_LL_SEND       LWGSM_DBG_OFF
#define LWGSM_CFG_DBG_LL_RECV       LWGSM_DBG_OFF


#include "esp_log.h"
#define LWGSM_CFG_DBG_OUT(fmt, ...)         do {                                      \
                                                ESP_LOGW("LWGSM_DBG_OUT", fmt, ##__VA_ARGS__);      \
                                            } while (0)

/* Time in milliseconds for sending first AT command after reset */
#define LWGSM_CFG_RESET_DELAY_DEFAULT       1000
#define LWGSM_CFG_RESET_ON_INIT             1
#define LWGSM_CFG_WAIT_AFTER_RESET          CONFIG_ESP_LWGSM_WAIT_AFTER_RESET_MODULE

/*
*   RDLR: Custom definitions
*/
#define LWGSM_UART_NUM              CONFIG_ESP_LWGSM_UART_PORT_NUM
#define LWGSM_UART_TX               CONFIG_ESP_LWGSM_UART_TX_PIN
#define LWGSM_UART_RX               CONFIG_ESP_LWGSM_UART_RX_PIN
#define LWGSM_UART_TX_BUF_SIZE      CONFIG_ESP_LWGSM_UART_TX_BUFFER_SIZE
#define LWGSM_UART_RX_BUF_SIZE      CONFIG_ESP_LWGSM_UART_RX_BUFFER_SIZE
#define LWGSM_UART_QUEUE_SIZE       10

#define LWGSM_SIM7080                   1
#define LWGSM_SIM7080_TCP_RECV_MANUAL   1

#define LWGSM_PREFERRED_NETWORK_TYPE    LWGSM_NET_TYPE_CAT_M
#define LWGSM_PREFERRED_MODE_SELECTION  LWGSM_MODE_SELECTION_AUTO

#define LWGSM_PRIMARY_DNS_SERVER        "8.8.8.8"           // Google Primary DNS server
#define LWGSM_SECONDARY_DNS_SERVER      "8.8.4.4"           // Google Secondary DNS server

#define LWGSM_SSL_STACK                 1
#define CA_ROOT_LABEL                   "ca_root.pem"
#define CLIENT_CERT_LABEL               "client.pem"
#define CLIENT_KEY_LABEL                "client.key"

#define LWGSM_SNTP_UPDATE_INTERVAL      43200   // 12-hours

#endif /* LWGSM_HDR_OPTS_H */
