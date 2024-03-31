#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_types.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_gen.h"

class Motor{
    int gpio_pin_num;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;
    mcpwm_oper_handle_t oper;
    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
    mcpwm_timer_handle_t timer;
public:
    Motor(int pin, mcpwm_oper_handle_t oper, uint16_t min_pulse, uint16_t max_pulse);
    void setTimer(mcpwm_timer_handle_t timer);
    void setComparatorGenerator(mcpwm_oper_handle_t oper);
    esp_err_t setPWM(uint8_t percentage);
};