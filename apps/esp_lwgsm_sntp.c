/**
 * @file esp_lwgsm_sntp.c
 * @author Rafael de la Rosa Vidal (derosa.rafael@gmail.com)
 * @brief SNTP protocol implementation over LWGSM
 * @version 0.1
 * @date 2022-12-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/* LWGSM includes */
#include "lwgsm_opts.h"
#include "lwgsm/lwgsm_netconn.h"

/* ESP IDF includes */
#include "esp_log.h"

/* AWS provisioning */
#include "aws_provisioning.h"

/* FreeRTOS includes */
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/timers.h"

/* Own header */
#include "apps/esp_lwgsm_sntp.h"
#include "apps/esp_lwgsm_sntp_policy.h"

/*******************************************************************************
 * 
 * Defines
 * 
*******************************************************************************/

#define ESP_LWGSM_SNTP_UPDATE_PERIOD_MS         1000   // 1 seconds
#define ESP_LWGSM_SNTP_UPDATE_PERIOD_TICKS      ESP_LWGSM_SNTP_UPDATE_PERIOD_MS/portTICK_PERIOD_MS
#define ESP_LWGSM_SNTP_RETRY_PERIOD_TICKS       5000/portTICK_PERIOD_MS    // 500 milliseconds

/* SNTP protocol defines */
#define ESP_LWGSM_SNTP_MSG_LEN                48

#define ESP_LWGSM_SNTP_OFFSET_LI_VN_MODE      0
#define ESP_LWGSM_SNTP_LI_MASK                0xC0
#define ESP_LWGSM_SNTP_LI_NO_WARNING          (0x00 << 6)
#define ESP_LWGSM_SNTP_LI_LAST_MINUTE_61_SEC  (0x01 << 6)
#define ESP_LWGSM_SNTP_LI_LAST_MINUTE_59_SEC  (0x02 << 6)
#define ESP_LWGSM_SNTP_LI_ALARM_CONDITION     (0x03 << 6) /* (clock not synchronized) */

#define ESP_LWGSM_SNTP_VERSION_MASK           0x38
#define ESP_LWGSM_SNTP_VERSION                (4/* NTP Version 4*/<<3)

#define ESP_LWGSM_SNTP_MODE_MASK              0x07
#define ESP_LWGSM_SNTP_MODE_CLIENT            0x03
#define ESP_LWGSM_SNTP_MODE_SERVER            0x04
#define ESP_LWGSM_SNTP_MODE_BROADCAST         0x05

#define ESP_LWGSM_SNTP_STRATUM_KOD            0x00

#ifndef ESP_LWGSM_SNTP_SET_SYSTEM_TIME_US
#define ESP_LWGSM_SNTP_SET_SYSTEM_TIME_US(s, f) \
        esp_lwgsm_sntp_set_system_time(s, f)
#endif /* ESP_LWGSM_SNTP_SET_SYSTEM_TIME_US */

/* Number of seconds between 1970 and Feb 7, 2036 06:28:16 UTC (epoch 1) */
#define DIFF_SEC_1970_2036          ((uint32_t)2085978496L)

/** Convert NTP timestamp fraction to microseconds.
 */
#ifndef ESP_LWGSM_SNTP_FRAC_TO_US
#define ESP_LWGSM_SNTP_FRAC_TO_US(f)        ((uint32_t)(f) / 4295)
#endif /* !ESP_LWGSM_SNTP_FRAC_TO_US */

/* Treat NTP timestamps as signed two's-complement integers. This way,
 * timestamps that have the MSB set simply become negative offsets from
 * the epoch (Feb 7, 2036 06:28:16 UTC). Representable dates range from
 * 1968 to 2104.
 */
#ifndef ESP_LWGSM_SNTP_SET_SYSTEM_TIME_NTP
#  define ESP_LWGSM_SNTP_SET_SYSTEM_TIME_NTP(s, f) \
    ESP_LWGSM_SNTP_SET_SYSTEM_TIME_US((uint32_t)((s) + DIFF_SEC_1970_2036), ESP_LWGSM_SNTP_FRAC_TO_US(f))
#endif /* !ESP_LWGSM_SNTP_SET_SYSTEM_TIME_NTP */

/* Get the system time either natively as NTP timestamp or convert from
 * Unix time in seconds and microseconds. Take care to avoid overflow if the
 * microsecond value is at the maximum of 999999. Also add 0.5 us fudge to
 * avoid special values like 0, and to mask round-off errors that would
 * otherwise break round-trip conversion identity.
 */
#ifndef ESP_LWGSM_SNTP_GET_SYSTEM_TIME_NTP
# define ESP_LWGSM_SNTP_GET_SYSTEM_TIME_NTP(s, f) do { \
    uint32_t sec_, usec_; \
    ESP_LWGSM_SNTP_GET_SYSTEM_TIME(sec_, usec_); \
    (s) = (int32_t)(sec_ - DIFF_SEC_1970_2036); \
    (f) = usec_ * 4295 - ((usec_ * 2143) >> 16) + 2147; \
  } while (0)
#endif /* !ESP_LWGSM_SNTP_GET_SYSTEM_TIME_NTP */

/*******************************************************************************
 * 
 * Typedefs, enums and structs
 * 
*******************************************************************************/

typedef struct{
    uint8_t li_vn_mode;      // Eight bits. li, vn, and mode.
                             // li.   Two bits.   Leap indicator.
                             // vn.   Three bits. Version number of the protocol.
                             // mode. Three bits. Client will pick mode 3 for client.
    uint8_t stratum;         // Eight bits. Stratum level of the local clock.
    uint8_t poll;            // Eight bits. Maximum interval between successive messages.
    uint8_t precision;       // Eight bits. Precision of the local clock.

    uint32_t rootDelay;      // 32 bits. Total round trip delay time.
    uint32_t rootDispersion; // 32 bits. Max error aloud from primary clock source.
    uint32_t refId;          // 32 bits. Reference clock identifier.

    uint32_t refTm_s;        // 32 bits. Reference time-stamp seconds.
    uint32_t refTm_f;        // 32 bits. Reference time-stamp fraction of a second.

    uint32_t origTm_s;       // 32 bits. Originate time-stamp seconds.
    uint32_t origTm_f;       // 32 bits. Originate time-stamp fraction of a second.

    uint32_t rxTm_s;         // 32 bits. Received time-stamp seconds.
    uint32_t rxTm_f;         // 32 bits. Received time-stamp fraction of a second.

    uint32_t txTm_s;         // 32 bits and the most important field the client cares about. Transmit time-stamp seconds.
    uint32_t txTm_f;         // 32 bits. Transmit time-stamp fraction of a second.
} esp_lwgsm_sntp_packet_t;

/* SNTP worker event types.
 * The Tmr Svc daemon posts SNTP_EVT_REQUEST and returns immediately;
 * the worker drains the queue and calls esp_lwgsm_sntp_request, which
 * blocks on modem dial + UDP send + 20s receive timeout. Running this
 * on the daemon would block every other software timer in the app. */
typedef enum {
    SNTP_EVT_STOP,
    SNTP_EVT_REQUEST,
} sntp_evt_type_t;

typedef struct {
    sntp_evt_type_t type;
} sntp_evt_t;

#define SNTP_EVT_Q_DEPTH 4

typedef struct{
    char* hostname;
    uint16_t port;
    lwgsm_netconn_p udp_pcb;
    esp_lwgsm_sntp_sync_mode_t sync_mode;
    esp_lwgsm_sntp_sync_status_t sync_status;
    esp_lwgsm_sntp_sync_time_cb_t notify_cb;
    TimerHandle_t timer;
    struct{
        uint8_t init : 1;
    } flags;
    uint16_t to_next_update;
    QueueHandle_t sntp_evt_q;
    TaskHandle_t  sntp_worker_task;
} esp_lwgsm_sntp_ctx_t;

/*******************************************************************************
 * 
 * Static variables
 * 
*******************************************************************************/

const char* TAG = "ESP_LWGSM_SNTP";
static esp_lwgsm_sntp_ctx_t* pctx;

/*******************************************************************************
 * 
 * Private function prototypes
 * 
*******************************************************************************/

static esp_err_t esp_lwgsm_sntp_request(void);
static esp_err_t esp_lwgsm_sntp_recv(esp_lwgsm_sntp_packet_t* msg);
static esp_err_t esp_lwgsm_sntp_process(esp_lwgsm_sntp_packet_t* msg);
static void esp_lwgsm_sntp_process_timestamps(esp_lwgsm_sntp_packet_t* msg);
static inline void esp_lwgsm_sntp_set_sync_status(esp_lwgsm_sntp_sync_status_t sync_status);
static inline int32_t esp_lwgsm_stnp_htonl(int32_t value);
static void esp_lwgsm_timer_cb( TimerHandle_t xTimer );
static void sntp_worker_task(void* args);

/*******************************************************************************
 * 
 * Public function bodies
 * 
*******************************************************************************/

/**
 * @brief Initialize the SNTP service 
 * 
 * @return ESP_OK if success, esp_err_t member otherwise 
 */
esp_err_t esp_lwgsm_sntp_init(void)
{
    esp_err_t ret = ESP_OK;
    char* timezone = NULL;
    BaseType_t tret;

    ESP_LOGD(TAG, "SNTP initialised\n");

    if(pctx != NULL){
        if(pctx->flags.init){
            return ESP_OK;
        }
    }

    /* Allocate context struct. NULL-check before memset (PC-1 from
     * earlier phases — under heap pressure, an unchecked pvPortMalloc
     * followed by memset hard-faults the device). */
    pctx = pvPortMalloc(sizeof(esp_lwgsm_sntp_ctx_t));
    if(pctx == NULL){
        ESP_LOGE(TAG, "Failed to allocate SNTP context");
        return ESP_ERR_NO_MEM;
    }
    memset(pctx, 0, sizeof(esp_lwgsm_sntp_ctx_t));

    // Get metadata from flash
    ret = aws_prov_get_sntp_settings(&(pctx->hostname), &(pctx->port), &timezone);
    if(ret != ESP_OK){
        goto clean;
    }

    // Set timezone. setenv copies the value, so we can free our local
    // immediately after. Without this free, every call to _init leaks the
    // timezone string returned by aws_prov_get_sntp_settings.
    ESP_LOGI(TAG, "Setting timezone to %s", timezone);
    setenv("TZ", timezone, 1);
    tzset();
    vPortFree(timezone);
    timezone = NULL;

    /* Get a LWGSM connection handler */
    pctx->udp_pcb = lwgsm_netconn_new(LWGSM_NETCONN_TYPE_UDP);
    if(pctx->udp_pcb == NULL){
        ret = ESP_FAIL;
        goto clean;
    }

    /* Set receive timeout */
    lwgsm_netconn_set_receive_timeout(pctx->udp_pcb, 20000);

    /* Set the default sync mode */
    pctx->sync_mode = ESP_LWGSM_SNTP_SYNC_MODE_SMOOTH;

    /* Set the reset sync status */
    pctx->sync_status = ESP_LWGSM_SNTP_SYNC_STATUS_RESET;

    /* Set notification callback to NULL */
    pctx->notify_cb = NULL;

    /* Set counter to next update */
    pctx->to_next_update = LWGSM_SNTP_UPDATE_INTERVAL;

    /* Create the worker queue + task BEFORE the timer so that even an
     * immediately-firing timer (which can happen if the system clock is
     * way off) finds the queue ready to receive its event. */
    pctx->sntp_evt_q = xQueueCreate(SNTP_EVT_Q_DEPTH, sizeof(sntp_evt_t));
    if(pctx->sntp_evt_q == NULL){
        ESP_LOGE(TAG, "Failed to create SNTP worker queue");
        ret = ESP_ERR_NO_MEM;
        goto clean;
    }

    tret = xTaskCreate(sntp_worker_task, "SNTP worker",
                       4096, NULL,
                       tskIDLE_PRIORITY + 3,
                       &pctx->sntp_worker_task);
    if(tret != pdPASS){
        ESP_LOGE(TAG, "Failed to create SNTP worker task");
        ret = ESP_FAIL;
        goto clean;
    }

    /* Create the timer to update */
    pctx->timer = xTimerCreate("ESP_LWGSM_SNTP_TIMER",
                               ESP_LWGSM_SNTP_UPDATE_PERIOD_TICKS,
                               pdTRUE,
                               0,
                               esp_lwgsm_timer_cb);
    if(pctx->timer == NULL){
        ESP_LOGE(TAG, "Failed to create SNTP timer");
        ret = ESP_ERR_NO_MEM;
        goto clean;
    }

    pctx->flags.init = true;

    return ret;

    clean:
        if(timezone != NULL){
            vPortFree(timezone);
        }
        esp_lwgsm_sntp_deinit();
        return ret;
}

/**
 * @brief Deinitialize the SNTP service
 * 
 * @return ESP_OK if success, esp_err_t member otherwise  
 */
esp_err_t esp_lwgsm_sntp_deinit(void)
{
    lwgsmr_t res = lwgsmOK;
    esp_err_t ret = ESP_OK;

    /* Guard against deinit called when init never ran (or after a
     * previous deinit). Pre-fix: dereferencing pctx->hostname here
     * crashed when init failed mid-way and called us via clean: */
    if(pctx == NULL){
        return ESP_OK;
    }

    /* Stop the timer FIRST so it cannot post a fresh REQUEST event into
     * a queue we're about to delete. xTimerDelete with portMAX_DELAY
     * blocks until the Tmr Svc daemon has processed the delete. */
    if(pctx->timer != NULL){
        if(xTimerStop(pctx->timer, portMAX_DELAY) != pdPASS){
            ESP_LOGW(TAG, "Failed to stop SNTP timer during deinit.");
        }
        if(xTimerDelete(pctx->timer, portMAX_DELAY) != pdPASS){
            ESP_LOGE(TAG, "Error while the timer removing.");
        }
        pctx->timer = NULL;
    }

    /* Close the netconn BEFORE waiting for the worker. If the worker is
     * blocked inside lwgsm_netconn_receive (20-second timeout), closing
     * the connection unblocks it with an error; otherwise we'd be polling
     * for up to 20 s for the recv to time out. After the worker exits we
     * still call lwgsm_netconn_delete below to release the handle. */
    if(pctx->udp_pcb != NULL){
        (void)lwgsm_netconn_close(pctx->udp_pcb);
    }

    /* Stop the worker via STOP sentinel and poll for self-delete. The poll
     * budget is generous (5 s) because even with the netconn-close above,
     * the worker may need to finish its current iteration's clean path. */
    if(pctx->sntp_evt_q != NULL){
        sntp_evt_t stop_evt = { .type = SNTP_EVT_STOP };
        (void)xQueueSend(pctx->sntp_evt_q, &stop_evt, portMAX_DELAY);
        bool worker_exited = false;
        for(int i = 0; i < 500 && pctx->sntp_worker_task != NULL; ++i){
            eTaskState state = eTaskGetState(pctx->sntp_worker_task);
            if(state == eDeleted || state == eInvalid){
                worker_exited = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        /* If the worker is still alive after the poll budget, leak pctx
         * rather than free it underneath a still-running worker — UAF
         * inside the worker would corrupt unrelated state, while a leaked
         * pctx is benign until the next _init (which re-checks pctx and
         * will now find a stale-but-stable pointer; not perfect but safe
         * enough for an exceptional path). */
        if(!worker_exited){
            ESP_LOGE(TAG, "SNTP worker did not exit within 5s; leaking pctx to avoid UAF.");
            return ESP_FAIL;
        }

        vQueueDelete(pctx->sntp_evt_q);
        pctx->sntp_evt_q = NULL;
        pctx->sntp_worker_task = NULL;
    }

    /* Now safe to release the netconn — worker is guaranteed exited. */
    if(pctx->udp_pcb != NULL){
        res = lwgsm_netconn_delete(pctx->udp_pcb);
        if(res != lwgsmOK){
            ret = ESP_FAIL;
            ESP_LOGW(TAG, "Failed to delete LWGSM connection (%d).",
                     lwgsm_netconn_getconnnum(pctx->udp_pcb));
        }
        pctx->udp_pcb = NULL;
    }

    pctx->notify_cb = NULL;
    pctx->flags.init = false;

    if(pctx->hostname != NULL){
        vPortFree(pctx->hostname);
        pctx->hostname = NULL;
    }

    /* Free pctx and NULL the global so the next _init starts clean.
     * Pre-fix: vPortFree(pctx) without nulling left a dangling pointer
     * that the next init would dereference at line 187. */
    vPortFree(pctx);
    pctx = NULL;

    return ret;
}

/**
 * @brief Start the SNTP synchronization service
 * 
 * @return ESP_OK if success, esp_err_t member otherwise  
 */
esp_err_t esp_lwgsm_sntp_start(void)
{
    /* Try request time from SNTP server */
    return esp_lwgsm_sntp_request();
}

/**
 * @brief Stop the SNTP synchronization service
 * 
 * @return ESP_OK if success, esp_err_t member otherwise  
 */
esp_err_t esp_lwgsm_sntp_stop(void)
{
    esp_err_t ret = ESP_OK;

    if(pctx == NULL || pctx->timer == NULL){
        return ESP_OK;
    }

    /* Just stop. The previous code also called xTimerReset here, which
     * in FreeRTOS re-arms a stopped timer and so contradicts the
     * function's name — _stop accidentally restarted the timer. */
    if(xTimerStop(pctx->timer, 1000/portTICK_PERIOD_MS) != pdPASS){
        ret = ESP_ERR_TIMEOUT;
        ESP_LOGE(TAG, "Not able to stop timer.");
    }

    pctx->to_next_update = LWGSM_SNTP_UPDATE_INTERVAL;

    return ret;
}

/**
 * @brief Set the system time from seconds and microseconds from 1970
 * 
 * @param sec Seconds from 1970
 * @param us Fraction of seconds in microseconds
 */
void esp_lwgsm_sntp_set_system_time(uint32_t sec, uint32_t us)
{
    struct timeval tv = { .tv_sec = sec, .tv_usec = us };
    esp_lwgsm_sntp_sync_time(&tv);
}

/**
 * @brief Synchronize the system time
 * 
 * @param timeval structure containing seconds and fractions of seconds from 1970
 * 
 */
void __attribute__((weak)) esp_lwgsm_sntp_sync_time(struct timeval *tv)
{
    if (pctx->sync_mode == ESP_LWGSM_SNTP_SYNC_MODE_IMMED) {
        settimeofday(tv, NULL);
        esp_lwgsm_sntp_set_sync_status(ESP_LWGSM_SNTP_SYNC_STATUS_COMPLETED);
    } else if (pctx->sync_mode == ESP_LWGSM_SNTP_SYNC_MODE_SMOOTH) {
        struct timeval tv_now;
        gettimeofday(&tv_now, NULL);
        int64_t cpu_time = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        int64_t sntp_time = (int64_t)tv->tv_sec * 1000000L + (int64_t)tv->tv_usec;
        int64_t delta = sntp_time - cpu_time;
        /* Always report the measured error: it is the one number that tells
         * how wrong the clock was (e.g. the post-reboot RC-oscillator error). */
        ESP_LOGI(TAG, "Clock error vs SNTP server: %lld ms", (long long)(delta / 1000LL));
        /* Slew only sub-second drift. adjtime() corrects at 1/64 of real time
         * and accepts errors up to ~35 min, so slewing a minutes-large
         * post-reboot error would leave every timestamp wrong for hours. */
        struct timeval tv_delta = { .tv_sec = delta / 1000000L, .tv_usec = delta % 1000000L };
        if (esp_lwgsm_sntp_should_step(delta) || adjtime(&tv_delta, NULL) == -1) {
            settimeofday(tv, NULL);
            ESP_LOGI(TAG, "Time was stepped through settimeofday");
            esp_lwgsm_sntp_set_sync_status(ESP_LWGSM_SNTP_SYNC_STATUS_COMPLETED);
        } else {
            esp_lwgsm_sntp_set_sync_status(ESP_LWGSM_SNTP_SYNC_STATUS_IN_PROGRESS);
        }
    }
    if(pctx->notify_cb){
        pctx->notify_cb(tv);
    }
}

/**
 * @brief Set the synchronization mode
 * 
 * @param sync_mode IMMED or SMOOTH mode
 */
void esp_lwgsm_sntp_set_sync_mode(esp_lwgsm_sntp_sync_mode_t sync_mode)
{
    pctx->sync_mode = sync_mode;
}

/**
 * @brief Get the current sync mode
 * 
 * @return IMMED or SMOOTH 
 */
esp_lwgsm_sntp_sync_mode_t esp_lwgsm_sntp_get_sync_mode(void)
{
    return pctx->sync_mode;
}

/**
 * @brief Set the time synchronization notification callback
 * 
 * @param callback The callback to use for notification
 */
void esp_lwgsm_sntp_set_notification_cb(esp_lwgsm_sntp_sync_time_cb_t callback)
{
    pctx->notify_cb = callback;
}

/**
 * @brief Get the time synchronization status
 * 
 * @return RESET, COMPLETE or IN PROGRESS 
 */
esp_lwgsm_sntp_sync_status_t esp_lwgsm_sntp_get_sync_status(void)
{
    esp_lwgsm_sntp_sync_status_t ret_sync_status = ESP_LWGSM_SNTP_SYNC_STATUS_RESET;
    esp_lwgsm_sntp_sync_status_t sync_status = pctx->sync_status;

    if (sync_status == ESP_LWGSM_SNTP_SYNC_STATUS_COMPLETED) {
        esp_lwgsm_sntp_set_sync_status(ESP_LWGSM_SNTP_SYNC_STATUS_RESET);
        ret_sync_status = ESP_LWGSM_SNTP_SYNC_STATUS_COMPLETED;
    } else if (sync_status == ESP_LWGSM_SNTP_SYNC_STATUS_IN_PROGRESS) {
        struct timeval outdelta;
        adjtime(NULL, &outdelta);
        if (outdelta.tv_sec == 0 && outdelta.tv_usec == 0) {
            esp_lwgsm_sntp_set_sync_status(ESP_LWGSM_SNTP_SYNC_STATUS_RESET);
            ret_sync_status = ESP_LWGSM_SNTP_SYNC_STATUS_COMPLETED;
        } else {
            ret_sync_status = ESP_LWGSM_SNTP_SYNC_STATUS_IN_PROGRESS;
        }
    }
    return ret_sync_status;
}

/*******************************************************************************
 * 
 * Private function bodies
 * 
*******************************************************************************/

/**
 * @brief Request time to SNTP server
 * 
 * @return ESP_OK if success, esp_err_t member otherwise   
 */
static esp_err_t esp_lwgsm_sntp_request(void)
{
    esp_err_t ret = ESP_OK;
    lwgsmr_t res;
    esp_lwgsm_sntp_packet_t* ntp_msg = NULL;
    bool is_error = false;

    ESP_LOGI(TAG, "Requesting time to SNTP server %s:%d", pctx->hostname, pctx->port);

    /* Connect to SNTP server */
    res = lwgsm_netconn_connect(pctx->udp_pcb, pctx->hostname, pctx->port);
    if(res != lwgsmOK){ 
        ESP_LOGE(TAG, "Not able to connect to SNTP server: %s:%d", pctx->hostname, pctx->port);
        is_error = true;
        goto clean;
    }

    /* Create and zero out the packet. All 48 bytes worth. */
    ntp_msg = pvPortMalloc(sizeof(*ntp_msg));
    memset(ntp_msg, 0, sizeof(*ntp_msg));
    /* Set the first byte's bits to 00,011,011 for li = 0, vn = 3, and mode = 3. The rest will be left set to zero. */
    ntp_msg->li_vn_mode = ESP_LWGSM_SNTP_LI_NO_WARNING | ESP_LWGSM_SNTP_VERSION | ESP_LWGSM_SNTP_MODE_CLIENT;

    /* Send the request message */
    res = lwgsm_netconn_send(pctx->udp_pcb, ntp_msg, sizeof(*ntp_msg));
    if(res != lwgsmOK){ 
        ESP_LOGE(TAG, "Failed to send SNTP request message.");
        is_error = true;
        goto clean;
    }

    /* Receive message from SNTP server */
    memset(ntp_msg, 0, sizeof(*ntp_msg));
    ret = esp_lwgsm_sntp_recv(ntp_msg);
    if(ret != ESP_OK){ 
        ESP_LOGE(TAG, "Failed to receive response from SNTP server.");
        is_error = true;
        goto clean;
    }

    /* Process the message */
    ret = esp_lwgsm_sntp_process(ntp_msg);
    if(ret != ESP_OK){ 
        ESP_LOGE(TAG, "Failed to process SNTP response.");
        is_error = true;
        goto clean;
    }

    clean:
        res = lwgsm_netconn_close(pctx->udp_pcb);
        if(res != lwgsmOK){
            ESP_LOGE(TAG, "Error closing the connection while clean. (%d)", res);
            is_error = true;
        }
        if(ntp_msg != NULL){
            vPortFree(ntp_msg);
        }
        // Start the timer to retry again later
        if(is_error && pctx->timer){
            xTimerReset(pctx->timer, 1000/portTICK_PERIOD_MS);
            xTimerChangePeriod(pctx->timer, ESP_LWGSM_SNTP_RETRY_PERIOD_TICKS, 1000/portTICK_PERIOD_MS);
            if( xTimerStart(pctx->timer, 0) != pdPASS){
                ESP_LOGE(TAG, "Error starting the update timer.");
            }
        }
        return ret;
}

/**
 * @brief Receive response from SNTP server
 * 
 * @param msg Data structure to fill with the SNTP server response
 * @return ESP_OK if success, esp_err_t member otherwise   
 */
static esp_err_t esp_lwgsm_sntp_recv(esp_lwgsm_sntp_packet_t* msg)
{
    esp_err_t ret = ESP_OK;
    lwgsmr_t res;
    lwgsm_pbuf_p pbuf = NULL;
    size_t pbuf_tot_len = 0, data_recv = 0, offset = 0;

    if(msg == NULL){
        ESP_LOGE(TAG, "SNTP message is NULL on %s", __func__);
        ret = ESP_FAIL;
        goto clean;
    }

    /* Receive into msg until we've accumulated sizeof(*msg) bytes (the
     * full 48-byte SNTP packet). The previous code checked pbuf_tot_len
     * <= ESP_LWGSM_SNTP_MSG_LEN per iteration but accumulated `offset`
     * across iterations, so two 40-byte pbufs (80 bytes total) would
     * stomp past the buffer. Bound by remaining capacity instead and
     * free each pbuf inside the loop to prevent the per-iteration leak
     * where the next lwgsm_netconn_receive overwrote the local pointer. */
    while(offset < sizeof(*msg)){
        res = lwgsm_netconn_receive(pctx->udp_pcb, &pbuf);
        if(res != lwgsmOK){
            ret = ESP_FAIL;
            goto clean;
        }
        pbuf_tot_len = lwgsm_pbuf_length(pbuf, 1);
        size_t remaining = sizeof(*msg) - offset;
        if(pbuf_tot_len > remaining){
            /* Server sent a malformed/oversized chunk. Log and abort the
             * receive — better to fail and let the retry timer handle it
             * than to silently truncate or overflow. */
            ESP_LOGE(TAG, "SNTP pbuf chunk %u exceeds remaining capacity %u; aborting recv.",
                     (unsigned)pbuf_tot_len, (unsigned)remaining);
            ret = ESP_ERR_INVALID_RESPONSE;
            goto clean;
        }
        data_recv = lwgsm_pbuf_copy(pbuf, msg, pbuf_tot_len, offset);
        offset += data_recv;
        lwgsm_pbuf_free(pbuf);
        pbuf = NULL;
    }

    clean:
        if(pbuf != NULL) { lwgsm_pbuf_free(pbuf); }
        return ret;
}

/**
 * @brief Process the SNTP data structure received from SNTP server
 * 
 * @param msg The data received from the SNTP server
 * @return ESP_OK if success, esp_err_t member otherwise  
 */
static esp_err_t esp_lwgsm_sntp_process(esp_lwgsm_sntp_packet_t* msg)
{
    esp_err_t ret = ESP_OK;
    uint8_t mode;

    mode = msg->li_vn_mode & ESP_LWGSM_SNTP_MODE_MASK;
    if(mode != ESP_LWGSM_SNTP_MODE_SERVER){
        /* Check response mode */
        ESP_LOGE(TAG, "The response mode is %d and should be %d.", mode, ESP_LWGSM_SNTP_MODE_SERVER);
        ret = ESP_ERR_INVALID_STATE;
        goto clean;
    }

    if(msg->stratum == ESP_LWGSM_SNTP_STRATUM_KOD){
        /* Kiss-of-death packet. Use another server or increase UPDATE_DELAY. */
        ESP_LOGE(TAG, "Kiss-of-Death packet received.");
        ret = ESP_ERR_INVALID_RESPONSE;
        goto clean;
    }

    /* We don't check origin timestamp as we send zero timestamp in the request message */

    esp_lwgsm_sntp_process_timestamps(msg);

    clean:
        if(ret == ESP_OK){
            if(pctx->timer){ 
                xTimerReset(pctx->timer, 1000/portTICK_PERIOD_MS);
                xTimerChangePeriod(pctx->timer, ESP_LWGSM_SNTP_UPDATE_PERIOD_TICKS, 1000/portTICK_PERIOD_MS);
                if( xTimerStart(pctx->timer, 0) != pdPASS){
                    ESP_LOGE(TAG, "Error starting the update timer.");
                }
            }
        }else if(ret == ESP_ERR_INVALID_STATE || ret == ESP_ERR_INVALID_RESPONSE){
            if(pctx->timer){ 
                xTimerReset(pctx->timer, 1000/portTICK_PERIOD_MS);
                xTimerChangePeriod(pctx->timer, ESP_LWGSM_SNTP_RETRY_PERIOD_TICKS, 1000/portTICK_PERIOD_MS);
                if( xTimerStart(pctx->timer, 0) != pdPASS){
                    ESP_LOGE(TAG, "Error starting the update timer.");
                }
            }
        }
        return ret;
}

/**
 * @brief Process the time in the SNTP server response
 * 
 * @param msg The data received from the SNTP server
 */
static void esp_lwgsm_sntp_process_timestamps(esp_lwgsm_sntp_packet_t* msg)
{
    int32_t sec;
    uint32_t frac;

    sec = esp_lwgsm_stnp_htonl(msg->txTm_s);
    frac = esp_lwgsm_stnp_htonl(msg->txTm_f);

    ESP_LWGSM_SNTP_SET_SYSTEM_TIME_NTP(sec, frac);
}

/**
 * @brief Set the current synchronization status
 * 
 * @param sync_status The new synchronization status
 */
static inline void esp_lwgsm_sntp_set_sync_status(esp_lwgsm_sntp_sync_status_t sync_status)
{
    pctx->sync_status = sync_status;
}

/**
 * @brief Swap the byte order of an uin32_t value
 * 
 * @param value The value to swap
 * @return The swapped value
 */
static inline int32_t esp_lwgsm_stnp_htonl(int32_t value)
{
    return ((value >> 24) & 0x000000FF) | ((value >> 8) & 0x0000FF00) | ((value << 8) & 0x00FF0000) | ((value << 24) & 0xFF000000);
}

/*******************************************************************************
 * 
 * Stack callbacks
 * 
*******************************************************************************/

/**
 * @brief Thin timer callback — posts a REQUEST event when the countdown
 * elapses, then returns. Previously this called esp_lwgsm_sntp_request
 * directly on the Tmr Svc daemon, which blocked the daemon for
 * lwgsm_netconn_connect + send + 20-second receive timeout. All of
 * that now runs on sntp_worker_task.
 *
 * @param xTimer The timer which triggers the callback
 */
static void esp_lwgsm_timer_cb( TimerHandle_t xTimer )
{
    (void)xTimer;
    if(pctx == NULL || pctx->timer == NULL){
        return;
    }

    if((xTimerGetPeriod(pctx->timer) == ESP_LWGSM_SNTP_UPDATE_PERIOD_TICKS) &&
       (pctx->to_next_update > ESP_LWGSM_SNTP_UPDATE_PERIOD_MS / 1000)){
        pctx->to_next_update -= ESP_LWGSM_SNTP_UPDATE_PERIOD_MS / 1000;
        return;
    }

    /* Countdown elapsed — drop the request to the worker and stop the
     * timer; the worker will restart it on success/failure via
     * esp_lwgsm_sntp_process / esp_lwgsm_sntp_request's clean path. */
    pctx->to_next_update = LWGSM_SNTP_UPDATE_INTERVAL;
    if(xTimerStop(pctx->timer, 0) != pdPASS){
        ESP_LOGE(TAG, "Error while stop the timer");
    }

    sntp_evt_t evt = { .type = SNTP_EVT_REQUEST };
    if(pctx->sntp_evt_q != NULL &&
       xQueueSend(pctx->sntp_evt_q, &evt, 0) != pdTRUE){
        ESP_LOGE(TAG, "SNTP worker queue full; request event dropped.");
    }
}

/**
 * @brief SNTP worker task.
 *
 * Single-consumer on pctx->sntp_evt_q. Drains REQUEST events and calls
 * esp_lwgsm_sntp_request, which performs the modem-blocking work
 * (connect, send, receive with 20s timeout). Exits on STOP via
 * vTaskDelete(NULL); deinit polls eTaskGetState to confirm.
 */
static void sntp_worker_task(void* args)
{
    sntp_evt_t evt;
    (void)args;

    ESP_LOGI(TAG, "SNTP worker task started.");

    while(true){
        if(xQueueReceive(pctx->sntp_evt_q, &evt, portMAX_DELAY) != pdTRUE){
            continue;
        }
        if(evt.type == SNTP_EVT_STOP){
            break;
        }
        if(evt.type == SNTP_EVT_REQUEST){
            (void)esp_lwgsm_sntp_request();
        }
    }

    ESP_LOGI(TAG, "SNTP worker task exiting.");
    vTaskDelete(NULL);
}