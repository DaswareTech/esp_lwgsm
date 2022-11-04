/* LWGSM includes */
#include "lwgsm/lwgsm.h"
#include "lwgsm/lwgsm_netconn.h"
#include "lwgsm/lwgsm_network_api.h"
#include "lwgsm/lwgsm_certs.h"
#include "lwgsm/lwgsm_mem.h"

/* ESP IDF includes */
#include "esp_log.h"

/* Own include */
#include "esp_lwgsm.h"

/* Utils */
#include "sim_manager.h"
#include "network_utils.h"
#include "operator_utils.h"

/*******************************************************************************
 * 
 * Defines
 * 
*******************************************************************************/

#define ESP_LWGSM_MBEDTLS_ERR_NET_INVALID_CONTEXT   -0x0045
#define ESP_LWGSM_MBEDTLS_ERR_NET_SEND_FAILED       -0x004E
#define ESP_LWGSM_MBEDTLS_ERR_NET_RECV_FAILED       -0x004C
#define ESP_LWGSM_MBEDTLS_ERR_SSL_WANT_READ         -0x6900

#define CHECK_LWGSMOK(x)    do{     \
                                if((x) != lwgsmOK){ \
                                    return -1;  \
                                }   \
                            } while(0)

/*******************************************************************************
 * 
 * Typedefs, enums and structs
 * 
*******************************************************************************/

typedef struct esp_lwgsm_mbedtls_net_context
{
    int fd;             /**< The underlying file descriptor                 */
} esp_lwgsm_mbedtls_net_context_t;

/*******************************************************************************
 * 
 * Static variables
 * 
*******************************************************************************/

static const char* TAG = "ESP_LWGSM";

static lwgsm_sim_state_t simState;
lwgsm_netconn_p client;
lwgsm_pbuf_p pbuf;

/*******************************************************************************
 * 
 * Private function prototypes
 * 
*******************************************************************************/

static lwgsmr_t esp_lwgsm_cb_func(lwgsm_evt_t* evt);
static esp_err_t prv_esp_lwgsm_init(uint8_t reinit);

/*******************************************************************************
 * 
 * Public function bodies
 * 
*******************************************************************************/

/**
 * \brief           Initialize the LWGSM stack, reset the GSM module, unlock SIM card and attach to operator network
 * \return          \ref lwgsmOK on success, member of \ref lwgsmr_t otherwise
 */
esp_err_t esp_lwgsm_init()
{
    return prv_esp_lwgsm_init(0);
}

/**
 * \brief           Reset the GSM module, unlock SIM card and attach to operator network
 * \return          \ref lwgsmOK on success, member of \ref lwgsmr_t otherwise
 */
esp_err_t esp_lwgsm_reinit()
{
    return prv_esp_lwgsm_init(1);
}

/**
 * @brief Reset the GSM module
 * 
 * @return esp_err_t ESP_OK if success, ESP_FAIL otherwise
 */
esp_err_t esp_lwgsm_reset()
{
    lwgsmr_t ret = lwgsmOK;

    if(lwgsm_network_is_attached()){
        ret = lwgsm_network_request_detach();
    }

    if(ret == lwgsmOK){
        ret = lwgsm_reset_with_delay(LWGSM_CFG_RESET_DELAY_DEFAULT, NULL, NULL, 1);
    }

    return ret == lwgsmOK ? ESP_OK : ESP_FAIL;
}

/**
 * \brief           Creates a TCP connection to a specified host and port
 * \param[in]       fd: Socket handler. The number of the 'lwgsm_conn_t' to use
 * \param[in]       host: The host address
 * \param[in]       port: The connection port
 * \param[in]       block: Status whether command should be blocking or not
 * \return          \ref ESP_OK on success, member of \ref esp_err_t otherwise
 */
esp_err_t esp_lwgsm_connect(int* fd, const char* host, int port, uint8_t block)
{
    esp_err_t res;
    lwgsmr_t ret = lwgsmERR;

    client = lwgsm_netconn_new(LWGSM_NETCONN_TYPE_TCP);

    if (client != NULL) {
        ESP_LOGI(TAG, "Connecting to %s:%d", host, port);
        if(block){
            ret = lwgsm_netconn_connect(client, host, port);
            if(ret == lwgsmOK){
                ESP_LOGI(TAG, "Connection success...");
            }
        }else{
            ret = lwgsm_netconn_connect_async(client, host, port);
            if(ret == lwgsmOK){
                ESP_LOGD(TAG, "Connection request sended...");
            }
        }
    }
    
    if(ret == lwgsmERR){
        ESP_LOGE(TAG, "Error connecting server...");
    }

    *fd = lwgsm_netconn_getconnnum(client);

    res = ret == lwgsmOK ? ESP_OK : ESP_FAIL;

    return res;
}

/**
 * \brief           Close a connection
 * \param[in]       fd: Socket handler. The number of the 'lwgsm_conn_t' to use
 * \return          \ref ESP_OK on success, member of \ref esp_err_t otherwise
 */
esp_err_t esp_lwgsm_close(int fd)
{
    lwgsmr_t ret;

    if(lwgsm_netconn_getconnnum(client) != fd){
        return -1;
    }

    ret = lwgsm_netconn_close(client);

    ret = lwgsm_netconn_delete(client);

    client = NULL;

    ESP_LOGI(TAG, "Connection closed.");

    return ret == lwgsmOK ? 0 : -1;
}

/**
 * \brief           Send data over a connection stablished
 * \param[in]       fd: Socket handler. The number of the 'lwgsm_conn_t' to use
 * \param[in]       data: Pointer to data to send
 * \param[in]       datalen: Number of bytes to send
 * \param[in]       flags: (Not used) Added for compatibility purposes
 * \return          \ref Number of bytes sent, -1 in case of error or timeout
 */
int esp_lwgsm_send(int fd, const char* data, size_t datalen, int flags)
{
    lwgsmr_t ret;

    (void) flags;

    if(lwgsm_netconn_getconnnum(client) != fd){
        return -1;
    }

    ret = lwgsm_netconn_write(client, data, datalen);
    CHECK_LWGSMOK(ret);

    ret = lwgsm_netconn_flush(client);
    CHECK_LWGSMOK(ret);

    return datalen;
}

/**
 * \brief           Read data from a stablished connection
 * \param[in]       fd: Socket handler. The number of the 'lwgsm_conn_t' to use
 * \param[in]       data: Pointer to a buffer where the data is going to be stored
 * \param[in]       datalen: Number of bytes to receive at most
 * \param[in]       flags: (Not used) Added for compatibility purposes
 * \return          \ref Number of bytes read, -1 in case of error or timeout
 */
int esp_lwgsm_recv(int fd, char* data, size_t datalen, int flags)
{
    lwgsmr_t ret;
    int data_recv = -1;
    int to_copy = 0;
    static int pbuf_tot_len;
    static int offset = 0;

    (void) flags;

    ESP_LOGD(TAG, "\n ==> Queried: %d", datalen);

    if(lwgsm_netconn_getconnnum(client) != fd){
        return -1;
    }

    if(pbuf == NULL){
        ret = lwgsm_netconn_receive(client, &pbuf);
        if(ret == lwgsmTIMEOUT){
            return 0;
        }
        else if(ret != lwgsmOK){
            return -1;
        }
        pbuf_tot_len = lwgsm_pbuf_length(pbuf, 1);
        ESP_LOGD(TAG, "Pbuf_tot_len = %d", pbuf_tot_len);
    }

    if(pbuf != NULL){
        to_copy = LWGSM_MIN(pbuf_tot_len, datalen);
        data_recv = lwgsm_pbuf_copy(pbuf, data, to_copy, offset);
        offset += data_recv;

        if(offset == pbuf_tot_len){
            lwgsm_pbuf_free(pbuf);
            offset = 0;
            pbuf = NULL;
        }
    }

    ESP_LOGD(TAG, "==> Recv: %d/%d", data_recv, datalen);
    if(data_recv != to_copy){
        ESP_LOGD(TAG,   "The number of bytes copied from pbuf differs from"
                        "the data queried (recv: %d - query: %d) ", data_recv, to_copy);
    }

    return data_recv;
}

/**
 * \brief           esp_lwgsm_send wrapper to be compatible with esp_mbedtls
 * \param[in]       ctx: Network context, see \ref 'esp_lwgsm_mbedtls_net_context_t'
 * \param[in]       data: Pointer to data to send
 * \param[in]       datalen: Number of bytes to receive at most
 * \return          \ref Number of bytes read, -1 in case of error or timeout
 */
int esp_lwgsm_mbedtls_send(void* ctx, const unsigned char* data, size_t datalen)
{
    int ret;
    int fd = ((esp_lwgsm_mbedtls_net_context_t *) ctx)->fd;

    if ( fd < 0 ) {
        return ( ESP_LWGSM_MBEDTLS_ERR_NET_INVALID_CONTEXT );
    }

    ret = esp_lwgsm_send(fd, (const char*) data, datalen, 0);
    if(ret < 0){
        return ( ESP_LWGSM_MBEDTLS_ERR_NET_SEND_FAILED );
    }
    return ( ret );
}

/**
 * \brief           esp_lwgsm_recv wrapper to be compatible with esp_mbedtls
 * \param[in]       ctx: Network context, see \ref 'esp_lwgsm_mbedtls_net_context_t'
 * \param[in]       data: Pointer to a buffer where the data is going to be stored
 * \param[in]       datalen: Number of bytes to receive at most
 * \return          \ref Number of bytes read, -1 in case of error or timeout
 */
int esp_lwgsm_mbedtls_recv(void* ctx, unsigned char* data, size_t datalen)
{
    int ret;
    int fd = ((esp_lwgsm_mbedtls_net_context_t *) ctx)->fd;

    if ( fd < 0 ) {
        return ( ESP_LWGSM_MBEDTLS_ERR_NET_INVALID_CONTEXT );
    }

    ret = esp_lwgsm_recv(fd, (char*) data, datalen, 0);

    if(ret < 0){
        return ( ESP_LWGSM_MBEDTLS_ERR_NET_RECV_FAILED );
    }
    else if(ret == 0){
        return ( ESP_LWGSM_MBEDTLS_ERR_SSL_WANT_READ );
    }
    return ( ret );
}

/**
 * \brief           Check if connection has been stablished for a time period
 * \param[in]       fd: Socket handler. The number of the 'lwgsm_conn_t' to use
 * \param[in]       timeout: Maximum time checking the connection
 * \return          \ref 1 if connected, 0 otherwise
 */
uint8_t esp_lwgsm_is_connected(int fd, uint32_t timeout)
{
    uint8_t connected;
    uint32_t attempts = 10;
    uint32_t interval = 0;

    if(lwgsm_netconn_getconnnum(client) != fd){
        return -1;
    }      

    connected = lwgsm_netconn_is_connected(client);

    if(timeout > 0){
        interval = timeout / attempts;
    }
    else{
        return connected;
    }

    while(!connected && (attempts > 0)){
        connected = lwgsm_netconn_is_connected(client);
        lwgsm_delay(interval);
        --attempts;
    }

    return connected;
}

/**
 * @brief Perform an operator scan
 * 
 * @return esp_err_t ESP_OK if sucess, ESP_FAIL otherwise
 */
esp_err_t esp_lwgsm_oper_scan()
{
    lwgsmr_t ret;
    lwgsm_operator_t* operators;
    uint16_t operator_array_len = 10;
    size_t operators_len;

    operators = lwgsm_mem_malloc(operator_array_len*sizeof(lwgsm_operator_t));

    ret = lwgsm_operator_scan(operators, operator_array_len, &operators_len, NULL, NULL, 1);

    lwgsm_mem_free(operators);

    return ret == lwgsmOK ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Set receive timeout
 * 
 * @param fd Socket handler. The number of the 'lwgsm_conn_t' to use
 * @param timeout Timeout to wait a for receive
 * @return int 
 */
int esp_lwgsm_set_recv_timeout(int fd, uint32_t timeout)
{
    if(lwgsm_netconn_getconnnum(client) != fd){
        return -1;
    }

    lwgsm_netconn_set_receive_timeout(client, timeout);

    return 0;
}

/*******************************************************************************
 * 
 * Private function bodies
 * 
*******************************************************************************/

/**
 * \brief           Event callback function for GSM stack
 * \param[in]       evt: Event information with data
 * \return          \ref lwgsmOK on success, member of \ref lwgsmr_t otherwise
 */
static lwgsmr_t esp_lwgsm_cb_func(lwgsm_evt_t* evt)
{
    char* report = NULL;

    switch (lwgsm_evt_get_type(evt)) {
        case LWGSM_EVT_INIT_FINISH: 
            ESP_LOGI(TAG, "Initialized.");
            break;
        case LWGSM_EVT_RESET:
            ESP_LOGI(TAG, "Reset complete.");
            break;
        case LWGSM_EVT_DEVICE_IDENTIFIED:
            ESP_LOGD(TAG, "Device identified.");
            break;
        case LWGSM_EVT_CMD_TIMEOUT:
            ESP_LOGW(TAG, "Operation timeout.");
            break;
        case LWGSM_EVT_NETWORK_REG_CHANGED:
            network_utils_process_reg_change(evt, &report);
            break;
        case LWGSM_EVT_NETWORK_OPERATOR_CURRENT:
            network_utils_process_curr_operator(evt, &report);
            break;
        case LWGSM_EVT_SIGNAL_STRENGTH:
            network_utils_process_rssi(evt, &report);
            break;
        case LWGSM_EVT_SIM_STATE_CHANGED:
            process_sim_evt(evt, &report);
            simState = evt->evt.cpin.state;
            break;
        case LWGSM_EVT_OPERATOR_SCAN:
            operator_utils_print_scan(evt);
            break;
        case LWGSM_EVT_NETWORK_ATTACHED:
            ESP_LOGI(TAG, "Operator network attached.");
            break;
        case LWGSM_EVT_NETWORK_DETACHED:
            ESP_LOGW(TAG, "Operator network detached.");
            break;
        default:
            ESP_LOGI(TAG, "CB called but not captured. Event: %d", lwgsm_evt_get_type(evt));
            break;
    }

    if(report != NULL) {
        ESP_LOGI(TAG, "%s", report);
        lwgsm_mem_free(report);
    }

    return lwgsmOK;
}

/**
 * \brief           Initialize the LWGSM stack, unlock SIM card and attach to operator network
 * \param[in]       reinit: Flag to indicate if the function should initialize or reinitialize
 * \return          \ref lwgsmOK on success, member of \ref lwgsmr_t otherwise
 */
static esp_err_t prv_esp_lwgsm_init(uint8_t reinit)
{
    lwgsmr_t ret;
    esp_err_t res;
    int16_t rssi = 0;

    if(!reinit){
        ret = lwgsm_init(esp_lwgsm_cb_func, 1);
        if(ret != lwgsmOK){
            ESP_LOGE(TAG, "Cannot initialize.\r\n");
            return ret;
        }
    }
    else{
        
        ESP_LOGD(TAG, "Reset GSM module...");
        ret = esp_lwgsm_reset();
        if(ret != lwgsmOK){
            ESP_LOGE(TAG, "Cannot reset.\r\n");
            return ret;
        }
    }

    /* Check SIM state and enter pin if needed */
    if(simState == LWGSM_SIM_STATE_PIN){
        if (configure_sim_card()) {
            ESP_LOGD(TAG, "SIM card configured.");
            lwgsm_delay(10000);
        } else {
            ESP_LOGE(TAG, "Cannot configure SIM card! Is it inserted, pin valid and not under PUK? Closing down...");
            return lwgsmERR;
        }
    }

    /* Check signal strength */
    do{
        ret = lwgsm_network_rssi(&rssi, NULL, NULL, pdTRUE);
        if(rssi != 0){ break; }
        lwgsm_delay(5000);
    } while((rssi == 0) && (ret == lwgsmOK));

    ret = lwgsm_network_request_define_pdp_context(ESP_LWGSM_PDP_INDEX, LWGSM_PDP_TYPE_IP, ESP_LWGSM_APN_NAME, ESP_LWGSM_PDP_ADDRESS, LWGSM_APN_D_COMP_OFF,
                LWGSM_APN_H_COMP_OFF, false, true);
    if(ret != lwgsmOK){
        ESP_LOGE(TAG, "Cannot set APN credentials.\r\n");
        return ret;
    }

    do{
        ret = lwgsm_network_request_attach();
        if(ret != lwgsmOK){lwgsm_delay(5000);};
    } while(ret != lwgsmOK);
    
    res = ret == lwgsmOK ? ESP_OK : ESP_FAIL;

    return res;
}