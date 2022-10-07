set(LWGSM_OPTS_DIR		${CMAKE_CURRENT_SOURCE_DIR}/../../../config)

idf_component_register( SRC_DIRS .
                        INCLUDE_DIRS     .
                                        ${LWGSM_OPTS_DIR}
                        PRIV_REQUIRES unity esp_lwgsm
                       )