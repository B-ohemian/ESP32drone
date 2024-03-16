idf_component_register(SRCS "ledc_motor.c" "motor.c"  
                    INCLUDE_DIRS "." 
                    PRIV_REQUIRES  udp wifi led init)