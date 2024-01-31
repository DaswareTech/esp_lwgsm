/**
 * @file esp_lwgsm.h
 * @author Rafael de la Rosa (derosa.rafael@gmail.com)
 * @brief API functions over LWGSM library adapted to SIM7080G
 * @version 0.1
 * @date 2023-01-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef ESP_LWGSM_H
#define ESP_LWGSM_H

#define ESP_LWGSM_PDP_INDEX       1

#include "lwgsm/lwgsm.h"
#include "esp_err.h"

esp_err_t esp_lwgsm_init(lwgsm_evt_fn evt_func);
esp_err_t esp_lwgsm_reinit();
esp_err_t esp_lwgsm_reset_sw();
esp_err_t esp_lwgsm_connect(int* fd, const char* host, int port, uint8_t block);
esp_err_t esp_lwgsm_close(int fd);
int esp_lwgsm_send(int fd, const char* data, size_t datalen, int flags);
int esp_lwgsm_recv(int fd, char* data, size_t datalen, int flags);
int esp_lwgsm_mbedtls_send(void* ctx, const unsigned char* data, size_t datalen);
int esp_lwgsm_mbedtls_recv(void* ctx, unsigned char* data, size_t datalen);
uint8_t esp_lwgsm_is_connected(int fd, uint32_t timeout);
esp_err_t esp_lwgsm_oper_scan();
int esp_lwgsm_set_recv_timeout(int fd, uint32_t timeout);

#endif /* ESP_LWGSM_H */