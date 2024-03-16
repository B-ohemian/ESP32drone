idf_component_register(SRCS "battery.c"  
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES  udp wifi led init)