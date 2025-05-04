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
const float IMU_sampling_ms = 50;

Bl_comm bl_comm;
void (*Bl_comm::command_cb)(uint8_t* data, uint16_t len) = command_cb; // or assign it to a valid function
uint32_t Bl_comm::clientHandle = 0;
bool Bl_comm::isConnecting = false;
bool Bl_comm::isWriting = false;

Motor *Thrust, *Servo1, *Servo2, *Servo3, *Servo4;
Motion_control motion;

TaskHandle_t bl_telem_handle_t = NULL;
bool isControlEnable = false;
//blでIMUデータを送信するのをノンブロッキングでやるためのタスク
void telemetry_task(){
    //ESP_LOGI("Telem", "sending imu");
    float pry[3];
    motion.getPRY(pry);
    //ESP_LOGI(TAG, "PRY: %1.2f,%1.2f,%1.2f", pry[0], pry[1], pry[2]);  

    //Throttle PRY x v a u  
    float msg[] = {Thrust->getPercent(),
        pry[0], pry[1], pry[2],
        motion.x(0, 0), motion.x(1, 0), motion.x(2, 0),
        motion.xhat(3, 0), motion.xhat(4, 0), motion.xhat(5, 0),
        motion.xhat(0, 0), motion.xhat(1, 0), motion.xhat(2, 0),
        motion.u(0, 0), motion.u(1, 0), motion.u(2, 0), motion.u(3, 0), motion.u(4, 0),};
    
    //ESP_LOGI(TAG, "msg len %d", sizeof(msg));
    bl_comm.sendMsg((char *)msg, sizeof(msg));
    vTaskDelete(bl_telem_handle_t); // タスクを削除します。
}
// 新たな定期的にスマホに送信するタイマーコールバック関数を定義します。
void bl_telemetry_callback(TimerHandle_t xTimer)
{
    if(bl_comm.isClientConnecting()){
        // 新たなタスクを作成してメッセージを送信します。

        //ESP_LOGI("Timer", "telemetring");
        xTaskCreate( (TaskFunction_t)telemetry_task, "TelemetryTask", 4096, NULL, 1, &bl_telem_handle_t);
    }
}

// タイマーコールバック関数を定義します。
void IRAM_ATTR timer_callback(TimerHandle_t xTimer)
{
    //ESP_LOGI("Timer", "reading.. imu");
    motion.update();
    if(isControlEnable){
        //Thrust->setPWM((motion.u(0, 0)));
        Servo1->setPWM((motion.u(1, 0) + 50.0f));
        Servo2->setPWM((motion.u(2, 0) + 50.0f));
        Servo3->setPWM((motion.u(3, 0) + 50.0f));
        Servo4->setPWM((motion.u(4, 0) + 50.0f));
    }
}

static void command_cb(uint8_t *msg, uint16_t msglen){
    ESP_LOGI(TAG, "%s", msg);
    char SPPmsg[32] = "";
    float val;
    memcpy(&val, &msg[1], sizeof(float));
    ESP_LOGI(TAG, "Command Recieved. %2.1f", val);
    switch (msg[0])
    {
    case 0:
        /* code */
        ESP_ERROR_CHECK(Thrust->setPWM(val));   
        break;
    case 1:
        ESP_ERROR_CHECK(Servo1->setPWM(val));
        break;
    case 2:
        ESP_ERROR_CHECK(Servo2->setPWM(val));
        break;
    case 3:
        ESP_ERROR_CHECK(Servo3->setPWM(val));
        break;
    case 4:
        ESP_ERROR_CHECK(Servo4->setPWM(val));
        break;
    case 5:
        isControlEnable = true;
        break;   
    case 6:
        isControlEnable = false;
        break;
    case 7:
        memcpy(&motion.KC.data[0], &msg[1], sizeof(float) * 6);
        break;
    case 8:
        memcpy(&motion.KC.data[1 * 6], &msg[1], sizeof(float) * 6);
        break;
        case 9:
        memcpy(&motion.KC.data[2 * 6], &msg[1], sizeof(float) * 6);
        break;
    case 10:
        memcpy(&motion.KC.data[3 * 6], &msg[1], sizeof(float) * 6);
        break;
    case 11:
        memcpy(&motion.KC.data[4 * 6], &msg[1], sizeof(float) * 6);
        break;
    case 12:
        memcpy(&motion.KC.data[5 * 6], &msg[1], sizeof(float) * 6);
        break;
    default:
        ESP_LOGI(TAG, "Unknow command Recieved.");
        sprintf(SPPmsg, "tUnknow command Recieved.");
        //もしBlueToothがつながってたら送信する
        if (bl_comm.isClientConnecting())
        {
            ESP_LOGI(TAG, "MSG Write to SPP.");
            ESP_LOGI(TAG, "%s", (uint8_t*)SPPmsg);

            bl_comm.sendMsg(SPPmsg, strlen(SPPmsg));
        }
        break;
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
            .enable_internal_pullup = false,
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
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    Thrust = new Motor(GPIO_NUM_21, oper, 1000, 2000);
    Thrust->begin();
    ESP_LOGI(TAG, "Motor Breaking ...");
    Thrust->setPWM(0);

    //servo
    mcpwm_oper_handle_t operServo = NULL;
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operServo));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operServo, timer));

    //comparaterが足りないのでoperatorを追加
    mcpwm_oper_handle_t operServo2 = NULL;
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operServo2));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operServo2, timer));

    Servo1 = new Motor(GPIO_NUM_4, operServo, 900, 2100);
    Servo2 = new Motor(GPIO_NUM_16, operServo, 900, 2100);
    Servo3 = new Motor(GPIO_NUM_15, operServo2, 900, 2100);
    Servo4 = new Motor(GPIO_NUM_2, operServo2, 900, 2100);
    Servo1->begin();
    Servo2->begin();
    Servo3->begin();
    Servo4->begin();
    Servo1->setPWM(50);
    Servo2->setPWM(50);
    Servo3->setPWM(50);
    Servo4->setPWM(50);
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
    const int IMU_sampling_ms = 10;
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
