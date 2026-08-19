idf_component_register(SRCS "bno085.c"
                               "sh2/sh2.c"
                               "sh2/shtp.c"
                               "sh2/sh2_SensorValue.c"
                               "sh2/sh2_util.c"
                               "sh2/euler.c"
                       INCLUDE_DIRS "." "sh2"
                       PRIV_REQUIRES driver esp_timer
                       )
