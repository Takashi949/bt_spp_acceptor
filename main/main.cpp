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

uint16_t gx0 = 0, gy0 = 0, gz0 = 0;
uint16_t ax0 = 0, ay0 = 0, az0 = 0;
uint16_t mx0 = 0, my0 = 0, mz0 = 0;

TaskHandle_t bl_telem_handle_t = NULL;
//blでIMUデータを送信するのをノンブロッキングでやるためのタスク
void telemetry_task(){
    ESP_LOGI("Telem", "sending imu");
    char msg[32];
    float pitchDeg = madgwick.getPitch();
    float rollDeg = madgwick.getRoll();
    float YawDeg = madgwick.getYaw();
    sprintf(msg, "imu,%3f,%3f,%3f", pitchDeg, rollDeg, YawDeg);
    bl_comm.sendMsg(msg);
    vTaskDelete(bl_telem_handle_t); // タスクを削除します。
}
// 新たな定期的にスマホに送信するタイマーコールバック関数を定義します。
void bl_telemetry_callback(TimerHandle_t xTimer)
{
    if(bl_comm.isClientConnecting()){
        // 新たなタスクを作成してメッセージを送信します。

        ESP_LOGI("Timer", "telemetring");
        xTaskCreate( (TaskFunction_t)telemetry_task, "TelemetryTask", 2048, NULL, 1, &bl_telem_handle_t);
    }
}

// タイマーコールバック関数を定義します。
void timer_callback(TimerHandle_t xTimer)
{
    ESP_LOGI("Timer", "reading.. imu");
    imu.readAccel();
    imu.readGyro();
    imu.readMag();
    imu.readTemp();
    madgwick.update(imu.calcGyro(imu.gx - gx0), imu.calcGyro(imu.gy - gy0), imu.calcGyro(imu.gz - gz0),
                    imu.calcAccel(imu.ax ax0), imu.calcAccel(imu.ay -ay0), imu.calcAccel(imu.az -az0),
                    imu.calcMag(imu.mx - mx0), imu.calcMag(imu.my - my0), imu.calcMag(imu.mz - mz0));
}

static void command_cb(uint8_t *msg, uint16_t msglen){
    ESP_LOGI(TAG, "%s", msg);
    
    if(msg[0] == 'c' && msg[1] == 't'){
        //char*から三桁の数字に変換
        uint8_t val = msg[2];
        if(val <= 100){
            throttle = val;
            if(oldThrottle != throttle){
                //set the throttle value
                ESP_ERROR_CHECK(Thrust->setPWM(throttle));   

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
        .glitch_ignore_cnt = 5,
        .intr_priority = 3,
        .flags{
            .enable_internal_pullup = false,
        },
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_conf, &bus_handle));

    if(imu.begin(LSM9DS1_AG_ADDR(0), LSM9DS1_M_ADDR(0), bus_handle) == 0){
        ESP_LOGE(TAG, "imu initialize faile");
    }

    //ジャイロセンサの初期値を取得
    //whileループで平均をとる
    int16_t gxt = 0, gyt = 0, gzt = 0;
    int16_t axt = 0, ayt = 0, azt = 0;
    int16_t mxt = 0, myt = 0, mzt =0;
    ESP_LOGI("IMU", "calibrate start");
    
    for (uint16_t i = 0; i < 1000; i++){
        ESP_LOGI(TAG, "rec...");
        imu.readGyro();
        gxt = (gxt + imu.gx)/2;
        gyt = (gyt + imu.gy)/2;
        gzt = (gzt + imu.gz)/2;

        imu.readAccel();
        axt = (axt + imu.ax)/2;
        ayt = (ayt + imu.ay)/2;
        azt = (azt + imu.az)/2;

        imu.readMag();
        mxt = (mxt + imu.mx)/2;
        myt = (myt + imu.my)/2;
        mzt = (mzt + imu.mz)/2;
    }
    
    gx0 = gxt; gy0 = gyt; gz0 = gzt;
    ax0 = axt; ay0 = ayt; az0 = azt;
    mx0 = mxt; my0 = myt; mz0 = mzt;

    ESP_LOGI("IMU", "calibrate FINISH");
}

static void pwm_init(){
    //モーターのタスク優先度　最優先0
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .intr_priority = 2,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    Thrust = new Motor(GPIO_NUM_21, oper, 1000, 2000);
    Thrust->begin();
}

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    pwm_init();
    
    bl_comm.setCommandCb(command_cb);
    ESP_ERROR_CHECK(bl_comm.begin());
    ESP_LOGI(TAG, "Own address:[%s]", bl_comm.get_bt_addr());

    ESP_LOGI(TAG, "Create IMU");
    i2c_master_init();
    const int IMU_sampling_ms = 20;
    madgwick.begin(1000/IMU_sampling_ms);
    // タイマーを作成し、コールバック関数を設定します。
    TimerHandle_t timer = xTimerCreate("IMU Timer", pdMS_TO_TICKS(IMU_sampling_ms), pdTRUE, (void *) 1, timer_callback);
    if (timer == NULL) {
        ESP_LOGE(TAG, "Failed to create timer.");
        return;
    }

    // タイマーを開始します。
    if (xTimerStart(timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start timer.");
        return;
    }

    // 新たなタイマーを作成し、コールバック関数を設定します。
    TimerHandle_t bl_telemetry = xTimerCreate("BL Telemetry", pdMS_TO_TICKS(200), pdTRUE, (void *) 2, bl_telemetry_callback);
    if (bl_telemetry == NULL) {
        ESP_LOGE(TAG, "Failed to create new timer.");
        return;
    }

    // 新たなタイマーを開始します。
    if (xTimerStart(bl_telemetry, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start new timer.");
        return;
    }

    // タスクをブロックします。
    //vTaskDelay(portMAX_DELAY);
}
