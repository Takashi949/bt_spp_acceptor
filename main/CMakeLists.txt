idf_component_register(SRCS "main.cpp" "bl_comm.cpp" "ESP32_i2c_LSM9DS1.cpp" "MadgwickAHRS.cpp" "motor.cpp"
                    INCLUDE_DIRS "." 
                    REQUIRES driver nvs_flash bt)
