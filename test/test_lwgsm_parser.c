#include <stdio.h>
#include "unity.h"
#include "esp_log.h"
#include "lwgsm/lwgsm.h"
#include "lwgsm/lwgsm_mem.h"
#include "operator_utils.h"


static lwgsmr_t lwgsm_callback_func(lwgsm_evt_t* evt);
const char* TAG = "LWGSM_PARSER_TEST";

TEST_CASE("Operator scan parser (AT+COPS=?)", "[lwgsm_parser]")
{
        const char* inputStr = "+COPS:(2,\"CHINA MOBILE\",\"CMCC\",\"46000\",0),(1,\"CHINA MOBILE\",\"CMCC\",\"46000\",1),(3,\"CHN-UNICOM\",    \
                                \"UNICOM\",\"46001\",2),(1,\"CHN-CT\",\"CT\",\"46011\",3),(3,\"CHN-UNICOM\",\"UNICOM\",\"46001\",4),,(0,1,2,3,4),(0,1,2)\r\nOK\r\n\r\n";
        lwgsmr_t ret;
        size_t ops_size, ops_found;
        lwgsm_operator_t* ops_array;

        /* Allocate memory for operator scanning */
        ops_size = 10;
        ops_array = pvPortMalloc(sizeof(*ops_array) * ops_size);
        memset(ops_array, 0, sizeof(*ops_array) * ops_size);
        configASSERT(ops_array);

        /* Initialize GSM with default callback function */
        ESP_LOGI(TAG,"Initializing LWGSM... \n");
        ret = lwgsm_init(lwgsm_callback_func, 1);
        if (ret != lwgsmOK) {
                ESP_LOGE(TAG,"Cannot initialize LwGSM\r\n");
        }

        /* Operator scanning */
        ret = lwgsm_operator_scan(ops_array, ops_size, &ops_found, NULL, NULL, 0);
        configASSERT(ret == lwgsmOK);

        lwgsm_delay(1000/portTICK_PERIOD_MS);

        if(ret == lwgsmOK){
                ret = lwgsmERR;
                ret = lwgsm_input_process(inputStr, strlen(inputStr));
                if(ret != lwgsmOK){
                        ESP_LOGE(TAG,"Data processing error.\r\n");
                }
                else{
                        ESP_LOGI(TAG,"Data processed correctly.\r\n");
                }
        }

        lwgsm_delay(5000/portTICK_PERIOD_MS);

        lwgsm_deinit();

        vPortFree(ops_array);
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
            ESP_LOGI(TAG,"Library initialized!\r\n");
            break;
        case LWGSM_EVT_RESET:
            ESP_LOGI(TAG,"Device reset complete.\r\n");
            break;
        case LWGSM_EVT_OPERATOR_SCAN:
            operator_utils_print_scan(evt, &report);
            break;
        default:
            ESP_LOGI(TAG, "CB called but not captured. Event: %d\r\n", lwgsm_evt_get_type(evt));
            break;
    }

    if(report != NULL){
        ESP_LOGI(TAG, "%s", report);
        lwgsm_mem_free(report);
    }

    return lwgsmOK;
}