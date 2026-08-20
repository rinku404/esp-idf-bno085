idf_component_register(SRCS "main.c" "sensor.c" "output.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES bno085 driver)
