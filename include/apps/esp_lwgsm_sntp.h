/**
 * @file esp_lwgsm_sntp.h
 * @author Rafael de la Rosa Vidal (derosa.rafael@gmail.com)
 * @brief SNTP protocol implementation over LWGSM
 * @version 0.1
 * @date 2022-12-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef ESP_LWGSM_SNTP_H
#define ESP_LWGSM_SNTP_H

/* Time includes */
#include <sys/time.h>

/*******************************************************************************
 * 
 * Typedefs, enums and structs
 * 
*******************************************************************************/

/**
 * @brief The synchronization mode
 * 
 */
typedef enum {
    ESP_LWGSM_SNTP_SYNC_MODE_IMMED,   /*!< Update system time immediately when receiving a response from the SNTP server. */
    ESP_LWGSM_SNTP_SYNC_MODE_SMOOTH,  /*!< Smooth time updating. Time error is gradually reduced using adjtime function. If the difference between SNTP response time and system time is large (more than 35 minutes) then update immediately. */
} esp_lwgsm_sntp_sync_mode_t;

/**
 * @brief The SNTP service synchronization status
 * 
 */
typedef enum {
    ESP_LWGSM_SNTP_SYNC_STATUS_RESET,         // Reset status.
    ESP_LWGSM_SNTP_SYNC_STATUS_COMPLETED,     // Time is synchronized.
    ESP_LWGSM_SNTP_SYNC_STATUS_IN_PROGRESS,   // Smooth time sync in progress.
} esp_lwgsm_sntp_sync_status_t;

/**
 * @brief SNTP callback function for notifying about time sync event
 *
 * @param tv Time received from SNTP server.
 */
typedef void (*esp_lwgsm_sntp_sync_time_cb_t) (struct timeval *tv);

/*******************************************************************************
 * 
 * Public function export
 * 
*******************************************************************************/

esp_err_t esp_lwgsm_sntp_init(void);
esp_err_t esp_lwgsm_sntp_deinit(void);
esp_err_t esp_lwgsm_sntp_start(void);
esp_err_t esp_lwgsm_sntp_stop(void);
void esp_lwgsm_sntp_set_system_time(uint32_t sec, uint32_t us);
void esp_lwgsm_sntp_sync_time(struct timeval *tv);
void esp_lwgsm_sntp_set_sync_mode(esp_lwgsm_sntp_sync_mode_t sync_mode);
esp_lwgsm_sntp_sync_mode_t esp_lwgsm_sntp_get_sync_mode(void);
void esp_lwgsm_sntp_set_notification_cb(esp_lwgsm_sntp_sync_time_cb_t callback);
esp_lwgsm_sntp_sync_status_t esp_lwgsm_sntp_get_sync_status(void);

#endif /* ESP_LWGSM_SNTP_H */