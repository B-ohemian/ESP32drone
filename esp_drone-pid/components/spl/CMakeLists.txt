idf_component_register(SRCS "spl06.c"  
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES  udp wifi led motor init mpu6050_driver pid  spl)