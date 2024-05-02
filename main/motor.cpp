#include "motor.h"
#include <string.h>
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_types.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_gen.h"
#include "esp_err.h"
#include <esp_log.h>

#define MOTOR_TAG "motor"

Motor::Motor(int pin, mcpwm_oper_handle_t _oper, uint16_t min_pulse, uint16_t max_pulse){
    gpio_pin_num = pin;
    min_pulse_us = min_pulse;
    max_pulse_us = max_pulse;
    oper = _oper;
    ESP_LOGI(MOTOR_TAG, "Create comparator and generator from the operator");

    mcpwm_comparator_config_t comparator_config = {
        .flags = {.update_cmp_on_tez = true},
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = gpio_pin_num,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));
}
void Motor::setTimer(mcpwm_timer_handle_t _timer){
    timer = _timer;
}
void Motor::begin(){
    // set the initial compare value, so that the servo will spin to the center position
    ESP_LOGI(MOTOR_TAG, "Motor Braking ...");
    ESP_ERROR_CHECK(setPWM(0));

    ESP_LOGI(MOTOR_TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));
}
uint8_t Motor::getPercent(){
    return percent;
}
esp_err_t Motor::setPWM(uint8_t percentage){
    if(percentage > 100)percentage = 100;
    esp_err_t err = ESP_OK;
    if((err = mcpwm_comparator_set_compare_value(comparator, (max_pulse_us - min_pulse_us)/100 * percentage + min_pulse_us)) != ESP_OK){
        ESP_LOGE(MOTOR_TAG, "%s", esp_err_to_name(err));
        return err;
    }
    oldPercent = percent;
    percent = percentage;
    return ESP_OK;
}
