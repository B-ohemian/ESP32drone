idf_component_register(SRCS "imu.c" "filter.c" "mpu6050.c" 
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES  udp wifi led motor init mpu6050_driver pid  spl battery supersconic)
