#include <stdio.h>
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/mcpwm.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define AC_LAMP_BURN_UP_SPEED 100
#define AC_LAMP_IMMEDIATELY_OFF 0

#ifdef __cplusplus

#include <vector>

typedef struct
{
    gpio_num_t gpio_num;
    bool gpio_phase;
    bool pullup_en;
    bool pulldown_en;
} ac_lamp_dimmer_input_config_t;
typedef struct
{
    mcpwm_unit_t mcpwm_timer_unit;
    gpio_num_t gpio_num;
    bool gpio_phase;
} ac_lamp_dimmer_output_config_t;

class ac_lamp_dimmer_intrn
{
public:
    ac_lamp_dimmer_output_config_t* ac_dimmer_channel;
    uint8_t current_output_state;
    uint8_t internal_current_duty;
    uint8_t target_duty;

    void fade_running_callback();

    ac_lamp_dimmer_intrn();

    esp_err_t init_intrn(const ac_lamp_dimmer_input_config_t* ac_sin_zcross_input_channel, const ac_lamp_dimmer_output_config_t* ac_lamp_dimmer_channel);
};


class ac_lamp_dimmer: protected ac_lamp_dimmer_intrn
{
public:
    void set(uint8_t value);
    uint8_t get();
    bool isComplete();

    esp_err_t init(const ac_lamp_dimmer_input_config_t* ac_sin_zcross_input_channel, const ac_lamp_dimmer_output_config_t* ac_lamp_dimmer_channel);
};
extern "C" {
#endif


#ifdef __cplusplus
}
#endif
