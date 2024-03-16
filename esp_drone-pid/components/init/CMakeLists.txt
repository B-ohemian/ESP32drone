idf_component_register(SRCS "init.c"  
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES  udp wifi led motor battery nvs_flash)