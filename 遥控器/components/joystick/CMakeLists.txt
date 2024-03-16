idf_component_register(SRCS "joystick.c"  
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES led udp wifi nvs_flash)