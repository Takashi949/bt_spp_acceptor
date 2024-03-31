idf_component_register(SRCS "main.cpp" "bl_comm.cpp" "ESP32_i2c_LSM9DS1.cpp" "MadgwickAHRS.cpp" "motor.cpp"
                    REQUIRES driver
                    REQUIRES nvs_flash
                    REQUIRES bt
                    INCLUDE_DIRS ".")
