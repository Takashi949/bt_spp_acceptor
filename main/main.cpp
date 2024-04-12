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
#include "Motion_control.h"

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

#define TAG "ESP_SPP_DRONE"

Bl_comm bl_comm;
void (*Bl_comm::command_cb)(uint8_t* data, uint16_t len) = command_cb; // or assign it to a valid function
uint32_t Bl_comm::clientHandle = 0;
bool Bl_comm::isConnecting = false;
bool Bl_comm::isWriting = false;

Motor *Thrust, *ServoSG, *ServoFS;
Motion_control motion;

TaskHandle_t bl_telem_handle_t = NULL;
bool isControlEnable = false;
//blでIMUデータを送信するのをノンブロッキングでやるためのタスク
void telemetry_task(){
    //ESP_LOGI("Telem", "sending imu");
    char msg[32];
    float pry[3];
    motion.getPRY(pry);
    sprintf(msg, "imu,%4.f,%4.f,%4.f,", pry[0], pry[1], pry[2]);
    bl_comm.sendMsg(msg);

    sprintf(msg, "Throttle %d,", Thrust->getPercent());
    bl_comm.sendMsg(msg);

    sprintf(msg, "x,%4.f,%4.f,%4.f,", motion.x[0], motion.x[1], motion.x[2]);
    bl_comm.sendMsg(msg); 
    
    sprintf(msg, "v,%4.f,%4.f,%4.f,", motion.v[0], motion.v[1], motion.v[2]);
    bl_comm.sendMsg(msg); 

    sprintf(msg, "a,%4.f,%4.f,%4.f,", motion.a[0], motion.a[1], motion.a[2]);
    bl_comm.sendMsg(msg); 

    vTaskDelete(bl_telem_handle_t); // タスクを削除します。
}
// 新たな定期的にスマホに送信するタイマーコールバック関数を定義します。
void bl_telemetry_callback(TimerHandle_t xTimer)
{
    if(bl_comm.isClientConnecting()){
        // 新たなタスクを作成してメッセージを送信します。

        //ESP_LOGI("Timer", "telemetring");
        xTaskCreate( (TaskFunction_t)telemetry_task, "TelemetryTask", 2048, NULL, 1, &bl_telem_handle_t);
    }
}

// タイマーコールバック関数を定義します。
void IRAM_ATTR timer_callback(TimerHandle_t xTimer)
{
    //ESP_LOGI("Timer", "reading.. imu");
    motion.update();
    if(isControlEnable){
        motion.calcU();
        Thrust->setPWM(motion.u[0]);
        ServoFS->setPWM(motion.u[1]);
        ServoSG->setPWM(motion.u[2]);
    }
}

static void command_cb(uint8_t *msg, uint16_t msglen){
    ESP_LOGI(TAG, "%s", msg);
    char SPPmsg[32] = "";

    if(msg[0] == 'c' && msg[1] == 't'){
        //char*から三桁の数字に変換
        uint8_t val = msg[2];   
        if(val <= 100){
            if(Thrust->getPercent() != val){
                //set the throttle value
                ESP_ERROR_CHECK(Thrust->setPWM(val));   
                sprintf(SPPmsg, "Throttole %d,", Thrust->getPercent());
            }
        }
    }else if(msg[0] == 'c' && msg[1] == 'r'){
        //char*から三桁の数字に変換
        uint8_t val = msg[2];
        if(val <= 100){
            if(ServoSG->getPercent() != val){
                //set the Angle value
                ESP_ERROR_CHECK(ServoSG->setPWM(val));
                sprintf(SPPmsg, "ServoSG %d,", ServoSG->getPercent());
            }
        }
    }else if(msg[0] == 'c' && msg[1] == 'l'){
        //char*から三桁の数字に変換
        uint8_t val = msg[2];
        if(val <= 100){
            if(ServoFS->getPercent() != val){
                //set the Angle value
                //向きが反対だから100-val注意
                ESP_ERROR_CHECK(ServoFS->setPWM(100-val));   
                sprintf(SPPmsg, "ServoFS %d,", ServoFS->getPercent());
            }
        }
    }else if(msg[0] == 'b' && msg[1] == 'c'){
        //制御開始
        isControlEnable = true;
        sprintf(SPPmsg, "Begin Control");
    }else if(msg[0] == 'e' && msg[1] == 'c'){
        //制御終了
        isControlEnable = false;
        sprintf(SPPmsg, "End Control");
    }
    else {
        ESP_LOGI(TAG, "Unknow command Recieved.");
        sprintf(SPPmsg, "Unknow command Recieved.");
    }

    //もしBlueToothがつながってたら送信する
    if (bl_comm.isClientConnecting())
    {
        ESP_LOGI(TAG, "MSG Write to SPP.");
        ESP_LOGI(TAG, "%s", (uint8_t*)SPPmsg);

        bl_comm.sendMsg(SPPmsg);
    }
}

static void i2c_master_init(float sampleFreq)
{
    i2c_master_bus_config_t bus_conf = {
        .i2c_port = -1,
        .sda_io_num = GPIO_NUM_18,
        .scl_io_num = GPIO_NUM_19,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 4,
        .intr_priority = 3,
        .flags{
            .enable_internal_pullup = true,
        },
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_conf, &bus_handle));

    motion.begin(sampleFreq, bus_handle);
    //motion.correctInitValue(100);
}

static void pwm_init(){
    //モーターのタスク優先度　最優先
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

    //servo
    mcpwm_oper_handle_t operServo = NULL;
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operServo));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operServo, timer));
    ServoSG = new Motor(GPIO_NUM_2, operServo, 500, 2400);
    ServoFS = new Motor(GPIO_NUM_0, operServo, 500, 2500);
    ServoSG->begin();
    ServoFS->begin();
    ServoSG->setPWM(50);
    ServoFS->setPWM(50);
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
    const int IMU_sampling_ms = 50;
    i2c_master_init(1000/IMU_sampling_ms);
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
    TimerHandle_t bl_telemetry = xTimerCreate("BL Telemetry", pdMS_TO_TICKS(100), pdTRUE, (void *) 2, bl_telemetry_callback);
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
    vTaskDelay(portMAX_DELAY);
}
