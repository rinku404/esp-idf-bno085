idf_component_register(SRCS "bno085.c"
                       INCLUDE_DIRS "."
                       SRC_DIRS "../sh2"
                       PRIV_INCLUDE_DIRS "../sh2"
                       PRIV_REQUIRES driver esp_timer
                       )
