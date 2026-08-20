if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    include(${CMAKE_CURRENT_LIST_DIR}/cmake/GenerateSh2Instances.cmake)
endif()

idf_component_register(SRCS "bno085.c"
                               ${SH2_GENERATED_SRCS}
                               "sh2/sh2_SensorValue.c"
                               "sh2/sh2_util.c"
                               "sh2/euler.c"
                       INCLUDE_DIRS "." "sh2"
                       PRIV_REQUIRES driver esp_timer
                       )
