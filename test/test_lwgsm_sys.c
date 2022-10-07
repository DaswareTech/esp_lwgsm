#include <stdio.h>
#include "esp_log.h"
#include "unity.h"

#include "lwgsm/lwgsm.h"
#include "lwgsm/lwgsm_network_api.h"
#include "sim_manager.h"
#include "network_utils.h"
#include "operator_utils.h"
#include "lwgsm/lwgsm_mem.h"

#define DEVICE_IS_CONNECTED     1

#define SEND_RES(x,d)   do{ \
                            lwgsm_delay((d)/portTICK_PERIOD_MS);    \
                            ESP_LOGI(TAG, "%s", (x));               \
                            lwgsm_input_process((x), strlen((x)));  \
                        }while(0)   \

#if !DEVICE_IS_CONNECTED
static SemaphoreHandle_t syncSem = NULL;
static const char* OK_RES = "OK\r\n";
static const char* CGMI_RES = "CISKITO\r\n\r\nOK\r\n";
static const char* CGMR_RES = "Revision:CISKITO\r\n\r\nOK\r\n";
static const char* CPIN_GET_RES = "+CPIN: SIM PIN\r\n\r\nOK\r\n";
static const char* CREG_ATT_RES = "OK\r\n\r\n+CREG: 1\r\n";
static const char* COPS_GET_RES = "+COPS: 0,0,\"CHN-CT\",9\r\n\r\nOK\r\n";
static const char* CGNAPN_RES = "+CGNAPN: 1,\"ctnb\"\r\n\r\nOK\r\n";
static const char* CSQ_GET_RES = "+CSQ: 20,0\r\n\r\nOK\r\n";
static const char* CGATT_GET_RES = "+CGATT: 1\r\n\r\nOK\r\n";
#endif

static const char* TAG = "TEST";
static lwgsm_sim_state_t simState;
static lwgsmr_t lwgsm_callback_func(lwgsm_evt_t* evt);
#if !DEVICE_IS_CONNECTED
static void vTaskResponses(void* pvParameters);
#endif

TEST_CASE("Initializing -> deinitializing loop (memory leak)", "[lwgsm_sys]")
{
        int times = 2;
        lwgsmr_t ret;

        while(times--){
                ESP_LOGI(TAG, "Initializing LWGSM... \n");
                ret = lwgsm_init(lwgsm_callback_func, 1);
                if (ret != lwgsmOK) {
                        ESP_LOGE(TAG, "Cannot initialize LwGSM\r\n");
                }

                lwgsm_delay(500/portTICK_PERIOD_MS);

                ESP_LOGI(TAG, "DeInitializing LWGSM... \n");
                ret = lwgsm_deinit();
                if (ret != lwgsmOK) {
                        ESP_LOGE(TAG, "Cannot deinitialize LwGSM\r\n");
                }

                lwgsm_delay(500/portTICK_PERIOD_MS);
        }
}

TEST_CASE("Initializing with reset and attach network", "[lwgsm_sys][lwgsm_netconn]")
{
    lwgsmr_t ret;
    int16_t rssi = 0;

#if !DEVICE_IS_CONNECTED
    xTaskCreate(vTaskResponses, "GSM responses", 4096, NULL, tskIDLE_PRIORITY+5, NULL);
    syncSem = xSemaphoreCreateBinary();
#endif

    // TEST INIT
    ESP_LOGI(TAG, "Initializing LWGSM...");
    ret = lwgsm_init(lwgsm_callback_func, 1);
    if(ret != lwgsmOK){
        ESP_LOGE(TAG, "Cannot initialize LwGSM\r\n");
        return;
    }
    
    /* Check SIM state and enter pin if needed */
    if(simState == LWGSM_SIM_STATE_PIN){
        if (configure_sim_card()) {
            ESP_LOGI(TAG, "SIM card configured.");
        } else {
            ESP_LOGI(TAG, "Cannot configure SIM card! Is it inserted, pin valid and not under PUK? Closing down...");
            while (1) { lwgsm_delay(1000); }
        }
    }

    /* Check signal strength */
    do{
        ret = lwgsm_network_rssi(&rssi, NULL, NULL, pdTRUE);
        lwgsm_delay(5000);
    } while((rssi == 0) && (ret == lwgsmOK));

    while (lwgsm_network_request_attach() != lwgsmOK) {
        lwgsm_delay(1000);
    }

#if !DEVICE_IS_CONNECTED
    xSemaphoreTake(syncSem, portMAX_DELAY);
#endif
}

/**
 * \brief           Event callback function for GSM stack
 * \param[in]       evt: Event information with data
 * \return          \ref lwgsmOK on success, member of \ref lwgsmr_t otherwise
 */
static lwgsmr_t lwgsm_callback_func(lwgsm_evt_t* evt)
{
    char* report = NULL;
    
    switch (lwgsm_evt_get_type(evt)) {
        case LWGSM_EVT_INIT_FINISH: 
            ESP_LOGI(TAG, "Library initialized!");
            break;
        case LWGSM_EVT_RESET:
            ESP_LOGI(TAG, "Device reset complete.");
            break;
        case LWGSM_EVT_DEVICE_IDENTIFIED:
            ESP_LOGI(TAG, "Device identified.");
            break;
        case LWGSM_EVT_CMD_TIMEOUT:
            ESP_LOGW(TAG, "LWGSM timeout!");
            break;
        // /* Process and print registration change */
        case LWGSM_EVT_NETWORK_REG_CHANGED:
            network_utils_process_reg_change(evt, &report);
            break;
        /* Process current network operator */
        case LWGSM_EVT_NETWORK_OPERATOR_CURRENT:
            network_utils_process_curr_operator(evt, &report);
            break;
        /* Process signal strength */
        case LWGSM_EVT_SIGNAL_STRENGTH:
            network_utils_process_rssi(evt, &report);
            break;
        case LWGSM_EVT_SIM_STATE_CHANGED:
            ESP_LOGI(TAG, "SIM state changed to: %d", evt->evt.cpin.state);
            simState = evt->evt.cpin.state;
            break;
        case LWGSM_EVT_OPERATOR_SCAN:
            operator_utils_print_scan(evt, &report);
            break;
        /* Other user events here... */
        default:
            ESP_LOGI(TAG, "CB called but not captured. Event: %d", lwgsm_evt_get_type(evt));
            break;
    }

    if(report != NULL){
        ESP_LOGI(TAG, "%s", report);
        lwgsm_mem_free(report);
    }

    return lwgsmOK;
}

#if !DEVICE_IS_CONNECTED
/**
 * \brief           Responses the AT commands from test case (3)
 * \param[in]       pvParameters: Parameters passed to the task
 */
static void vTaskResponses(void* pvParameters)
{
    ESP_LOGI(TAG, "Task responses waiting...");
    vTaskDelay(1000/portTICK_PERIOD_MS);
    // CFUN=1,1
    SEND_RES(OK_RES, LWGSM_CFG_RESET_DELAY_DEFAULT);
    // ATE
    SEND_RES(OK_RES, 500);
    // CFUN=1
    SEND_RES(OK_RES, 500);
    // AT+CMEE=1
    SEND_RES(OK_RES, 500);
    // AT+CGMI
    SEND_RES(CGMI_RES, 500);
    // AT+CGMM
    SEND_RES(CGMI_RES, 500);
    // AT+CGSN
    SEND_RES(CGMI_RES, 500);
    // AT+CGMR
    SEND_RES(CGMR_RES, 500);
    // AT+CREG=1
    SEND_RES(OK_RES, 500);
    // AT+CCLC=1
    SEND_RES(OK_RES, 500);
    // AT+CPIN?
    SEND_RES(CPIN_GET_RES, 500);
    // AT+CMNB=<Network_type>
    SEND_RES(OK_RES, 500);
    // AT+CPIN?
    SEND_RES(CPIN_GET_RES, 500);
    // AT+CPIN=<PIN_NUMBER>
    SEND_RES(CREG_ATT_RES, 500);
    // AT+COPS?
    SEND_RES(COPS_GET_RES, 500);
    // AT+CSQ
    SEND_RES(CSQ_GET_RES, 500);
    // AT+CSQ
    SEND_RES(CSQ_GET_RES, 500);
    // AT+CGATT?
    SEND_RES(CGATT_GET_RES, 5000);
    // AT+CGNAPN
    SEND_RES(CGNAPN_RES, 500);
    // AT+CNCFG=0,1,<APN_NAME>
    SEND_RES(OK_RES, 500);
    // AT+CNACT=0,1
    SEND_RES(OK_RES, 500);

    vTaskDelay(5000/portTICK_PERIOD_MS);
    xSemaphoreGive(syncSem);
    vTaskDelete(NULL);
}
#endif