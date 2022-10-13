#ifndef ESP_LWGSM_H
#define ESP_LWGSM_H

#define ESP_LWGSM_PDP_INDEX       1
#define ESP_LWGSM_APN_NAME        "orangeworld"
#define ESP_LWGSM_PDP_ADDRESS     "0.0.0.0"

#include "esp_err.h"

esp_err_t esp_lwgsm_init();
esp_err_t esp_lwgsm_reinit();
esp_err_t esp_lwgsm_reset();
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