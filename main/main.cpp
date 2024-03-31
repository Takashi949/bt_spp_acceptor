/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "time.h"
#include "sys/time.h"

#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_types.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_gen.h"
#include "bl_comm.h"

#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"

#include "motor.h"

// Please consult the datasheet of your servo before changing the following parameters

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

#define TAG "ESP_SPP_DRONE"

uint32_t throttle = 0;
uint32_t oldThrottle = 0;

Bl_comm bl_comm;
void (*Bl_comm::command_cb)(uint8_t* data, uint16_t len) = command_cb; // or assign it to a valid function
uint32_t Bl_comm::clientHandle = 0;
bool Bl_comm::isConnecting = false;
bool Bl_comm::isWriting = false;

LSM9DS1 imu;
Madgwick madgwick;
Motor *Thrust;

// タイマーコールバック関数を定義します。
void timer_callback(TimerHandle_t xTimer)
{
    imu.readAccel();
    imu.readGyro();
    imu.readMag();
    imu.readTemp();
    madgwick.update(imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz),
                    imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az),
                    imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
    if(bl_comm.isClientConnecting()){
        char msg[32];
        float pitchDeg = madgwick.getPitch();
        float rollDeg = madgwick.getRoll();
        float YawDeg = madgwick.getYaw();
        sprintf(msg, "imu,%3f,%3f,%3f", pitchDeg, rollDeg, YawDeg);
        bl_comm.sendMsg(msg);
    }
}

static void command_cb(uint8_t *msg, uint16_t msglen){
    ESP_LOGI(TAG, "%s", msg);
    
    if(msg[0] == 'c' && msg[1] == 't'){
        //char*から三桁の数字に変換
        uint8_t val = msg[2];
        if(0 <= val && val <= 100){
            throttle = val;
            if(oldThrottle != throttle){
                //set the throttle value
                //ESP_ERROR_CHECK(Thrust->setPWM(throttle));   

                if (bl_comm.isClientConnecting())
                {
                    char msg[32] = "";
                    sprintf(msg, "Throttole %ld", throttle);
                    ESP_LOGI(TAG, "Throttle value Write to SPP.");
                    ESP_LOGI(TAG, "%s", (uint8_t*)msg);

                    bl_comm.sendMsg(msg);
                }
                oldThrottle = throttle;
            }
        }
    }
    else {
        if (bl_comm.isClientConnecting())
        {
            ESP_LOGI(TAG, "Unknow command Recieved.");
            char *msg = "Unknow command Recieved.";
            bl_comm.sendMsg(msg);
        }
    }
}

static void i2c_master_init(void)
{
    i2c_master_bus_config_t bus_conf = {
        .i2c_port = -1,
        .sda_io_num = GPIO_NUM_18,
        .scl_io_num = GPIO_NUM_19,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_conf, &bus_handle));

    if(imu.begin(LSM9DS1_AG_ADDR(0), LSM9DS1_M_ADDR(0), bus_handle) == false){
        ESP_LOGE(TAG, "imu initialize faile");
    }
}

static void pwm_init(){
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(SPP_TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    Thrust = new Motor(GPIO_NUM_21, oper, 1000, 2000);
    Thrust->setTimer(timer);
}

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    
    bl_comm.setCommandCb(command_cb);
    ESP_ERROR_CHECK(bl_comm.begin());
    ESP_LOGI(TAG, "Own address:[%s]", bl_comm.get_bt_addr());

    ESP_LOGI(TAG, "Create IMU");
 
    i2c_master_init();
    madgwick.begin(10);
    
   // pwm_init();

    // タイマーを作成し、コールバック関数を設定します。
    TimerHandle_t timer = xTimerCreate("IMU Timer", pdMS_TO_TICKS(200), pdTRUE, (void *) 0, timer_callback);
    if (timer == NULL) {
        ESP_LOGE(TAG, "Failed to create timer.");
        return;
    }

    // タイマーを開始します。
    if (xTimerStart(timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start timer.");
        return;
    }

    // タスクをブロックします。
    vTaskDelay(portMAX_DELAY);
}
