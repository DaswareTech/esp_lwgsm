set(submodule lwgsm)

set(SRC_FILES   ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/api/${submodule}_netconn.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/api/${submodule}_network_api.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/apps/mqtt/${submodule}_mqtt_client_api.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/apps/mqtt/${submodule}_mqtt_client_evt.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/apps/mqtt/${submodule}_mqtt_client.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_buff.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_call.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_conn.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_debug.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_device_info.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_evt.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_ftp.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_http.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_input.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_int.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_mem.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_network.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_operator.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_parser.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_pbuf.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_phonebook.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_ping.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_sim.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_sms.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_threads.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_timeout.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_unicode.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_ussd.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_utils.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}_certs.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/${submodule}/${submodule}.c
                # LIBRARY PORT
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/system/${submodule}_ll_esp32.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/system/${submodule}_sys_freertos_os.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/system/${submodule}_provision_esp32.c
                # SNIPPETS
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/snippets/sim_manager.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/snippets/device_info.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/snippets/netconn_client.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/snippets/network_utils.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/snippets/client.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/snippets/mqtt_client_api.c
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/snippets/operator_utils.c
                # ${CMAKE_CURRENT_SOURCE_DIR}/snippets/call_sms.c
                # ${CMAKE_CURRENT_SOURCE_DIR}/snippets/call.c
                # ${CMAKE_CURRENT_SOURCE_DIR}/snippets/sms_send_receive_thread.c
                # ${CMAKE_CURRENT_SOURCE_DIR}/snippets/sms_send_receive.c
                # CUSTOM APPS
                ${CMAKE_CURRENT_SOURCE_DIR}/apps/esp_lwgsm_sntp.c
                # MAIN
                ${CMAKE_CURRENT_SOURCE_DIR}/esp_lwgsm.c
    )

set(HEADER_FILES 
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/include
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/${submodule}/src/include/system/port/esp32
                ${CMAKE_CURRENT_SOURCE_DIR}/${submodule}/snippets/include
                ${CMAKE_CURRENT_SOURCE_DIR}/include
                ${CMAKE_CURRENT_SOURCE_DIR}/config
    )

idf_component_register(
    SRCS
        "${SRC_FILES}"
    INCLUDE_DIRS
        "${HEADER_FILES}"
    PRIV_REQUIRES
        log
        aws_provisioning
)