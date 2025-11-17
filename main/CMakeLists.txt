idf_component_register(SRCS "ads1220.c" "main.c"
                       PRIV_REQUIRES spi_flash driver
                       INCLUDE_DIRS "")
