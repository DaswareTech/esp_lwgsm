/**
 * @file esp_lwgsm_info.h
 * @author Rafael de la Rosa (derosa.rafael@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef ESP_LWGSM_INFO_H
#define ESP_LWGSM_INFO_H

/*******************************************************************************
 * 
 * Public function export
 * 
*******************************************************************************/

esp_err_t esp_lwgsm_get_rssi(int16_t* rssi);
esp_err_t esp_lwgsm_get_device_revision_cached(char** rev);

#endif /* ESP_LWGSM_INFO_H */