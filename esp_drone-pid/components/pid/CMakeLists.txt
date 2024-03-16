idf_component_register(SRCS "pid.c" "altitude.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES  udp wifi led motor init mpu6050_driver spl battery)
