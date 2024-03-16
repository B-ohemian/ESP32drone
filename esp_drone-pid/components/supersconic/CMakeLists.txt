idf_component_register(SRCS "supersconic.c"  
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES  udp wifi led motor init mpu6050_driver pid  spl)