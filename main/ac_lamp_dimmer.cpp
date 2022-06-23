#include "ac_lamp_dimmer.h"



#include "driver/pcnt.h"
#include "soc/mcpwm_periph.h"
#include "esp_log.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"




#define AC_LAMP_MAX_DUTY 85 //< 100
#define AC_LAMP_MIN_DUTY 0 //>=0


#define AC_LAMP_PHASE_OFFSET_uS 1200



//STATIC INTERRUPT CALLBACKS//////////////////

ac_lamp_dimmer_input_config_t ac_lamp_dimmer_input = {GPIO_NUM_MAX, 0};

std::vector<ac_lamp_dimmer_intrn*> resetting_timers;
uint32_t anti_glitch_buf = 0;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t anti_glitch_temp = xTaskGetTickCount();
    volatile uint32_t div121 = anti_glitch_temp - anti_glitch_buf;

    if (div121 >= pdMS_TO_TICKS(10))
    {
        anti_glitch_buf = anti_glitch_temp;

        if ((ac_lamp_dimmer_input.gpio_phase && gpio_get_level(ac_lamp_dimmer_input.gpio_num)) || (!ac_lamp_dimmer_input.gpio_phase && !gpio_get_level(ac_lamp_dimmer_input.gpio_num)))
        {
            for (size_t i=0; i<resetting_timers.size(); i++)
            {
                ac_lamp_dimmer_intrn* dm_ptr = (ac_lamp_dimmer_intrn*)resetting_timers[i];

                mcpwm_timer_trigger_soft_sync(dm_ptr->ac_dimmer_channel->mcpwm_timer_unit, MCPWM_TIMER_1);
            }
        }
    }
}

void ac_lamp_dimmer_task(void *pvParameters)
{
    for (;;vTaskDelay(1 + pdMS_TO_TICKS(1000/AC_LAMP_BURN_UP_SPEED)))
    {
        for (size_t i=0; i<resetting_timers.size(); i++)
        {
            ac_lamp_dimmer_intrn* dm_ptr = (ac_lamp_dimmer_intrn*)resetting_timers[i];
            dm_ptr->fade_running_callback();
        }
    }
}

///////////////////////////////////////


void ac_lamp_dimmer_intrn::fade_running_callback()
{
    if (internal_current_duty != target_duty)
    {
        if (internal_current_duty < target_duty) internal_current_duty++;
        else
        {
#if AC_LAMP_IMMEDIATELY_OFF
            internal_current_duty = target_duty;
#else
            internal_current_duty--;
#endif
        }

        int32_t temp = internal_current_duty;

        if (temp >= AC_LAMP_MAX_DUTY) temp = 100;
        mcpwm_set_duty(ac_dimmer_channel->mcpwm_timer_unit, MCPWM_TIMER_0, MCPWM_GEN_A, 100 - temp);
    }
}

ac_lamp_dimmer_intrn::ac_lamp_dimmer_intrn()
{
    ac_dimmer_channel = NULL;
    current_output_state = 0;
    target_duty = internal_current_duty = AC_LAMP_MIN_DUTY;
}

esp_err_t ac_lamp_dimmer_intrn::init_intrn(const ac_lamp_dimmer_input_config_t* ac_sin_zcross_input_channel, const ac_lamp_dimmer_output_config_t* ac_lamp_dimmer_channel)
{
    ac_dimmer_channel = (ac_lamp_dimmer_output_config_t*)ac_lamp_dimmer_channel;


    mcpwm_config_t pwm_config = {
        .frequency = 100,
        .cmpr_a = 100,
        .cmpr_b = 0,
        .duty_mode = (ac_dimmer_channel->gpio_phase)?MCPWM_DUTY_MODE_1:MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    ESP_ERROR_CHECK(mcpwm_init(ac_dimmer_channel->mcpwm_timer_unit, MCPWM_TIMER_0, &pwm_config));
    ESP_ERROR_CHECK(mcpwm_init(ac_dimmer_channel->mcpwm_timer_unit, MCPWM_TIMER_1, &pwm_config));

    ESP_ERROR_CHECK(mcpwm_gpio_init(ac_dimmer_channel->mcpwm_timer_unit, MCPWM0A, ac_dimmer_channel->gpio_num));

    mcpwm_sync_config_t sync_conf = {
        .sync_sig = MCPWM_SELECT_TIMER1_SYNC,
        .timer_val = AC_LAMP_PHASE_OFFSET_uS/10,
        .count_direction = MCPWM_TIMER_DIRECTION_UP,
    };

    ESP_ERROR_CHECK(mcpwm_set_timer_sync_output(ac_dimmer_channel->mcpwm_timer_unit, MCPWM_TIMER_1, MCPWM_SWSYNC_SOURCE_TEZ));
    ESP_ERROR_CHECK(mcpwm_sync_configure(ac_dimmer_channel->mcpwm_timer_unit, MCPWM_TIMER_0, &sync_conf));




    ac_lamp_dimmer_intrn* cb_arg = this;
    resetting_timers.push_back(cb_arg);

    if (ac_lamp_dimmer_input.gpio_num >= GPIO_NUM_MAX)
    {
        ac_lamp_dimmer_input = *ac_sin_zcross_input_channel;

        gpio_pad_select_gpio(ac_lamp_dimmer_input.gpio_num);
        gpio_set_direction(ac_lamp_dimmer_input.gpio_num, GPIO_MODE_INPUT);

        if (ac_lamp_dimmer_input.pullup_en) gpio_pullup_en(ac_lamp_dimmer_input.gpio_num);
        else gpio_pullup_dis(ac_lamp_dimmer_input.gpio_num);

        if (ac_lamp_dimmer_input.pulldown_en) gpio_pulldown_en(ac_lamp_dimmer_input.gpio_num);
        else gpio_pulldown_dis(ac_lamp_dimmer_input.gpio_num);

        gpio_install_isr_service(0);
        gpio_set_intr_type(ac_lamp_dimmer_input.gpio_num, (ac_lamp_dimmer_input.gpio_phase)?GPIO_INTR_POSEDGE:GPIO_INTR_NEGEDGE);
        gpio_isr_handler_add(ac_lamp_dimmer_input.gpio_num, gpio_isr_handler, NULL);

        xTaskCreate(&ac_lamp_dimmer_task, "operation task", 3000, NULL, 5, NULL);
    }


    

    ESP_LOGI("AC LAMP DIMMER", "HAS BEEN INITIALIZED");
    return ESP_OK;
}



esp_err_t ac_lamp_dimmer::init(const ac_lamp_dimmer_input_config_t* ac_sin_zcross_input_channel, const ac_lamp_dimmer_output_config_t* ac_lamp_dimmer_channel)
{
    return ac_lamp_dimmer_intrn::init_intrn(ac_sin_zcross_input_channel, ac_lamp_dimmer_channel);
}
void ac_lamp_dimmer::set(uint8_t value)
{
    if (value > AC_LAMP_MAX_DUTY) value = AC_LAMP_MAX_DUTY;
    else if (value < AC_LAMP_MIN_DUTY) value = AC_LAMP_MIN_DUTY;

    target_duty = value;
}
uint8_t ac_lamp_dimmer::get()
{
    uint8_t res = target_duty;

    if (res >= AC_LAMP_MAX_DUTY) res = 100;
    else if (res <= AC_LAMP_MIN_DUTY) res = 0;

    return res;
}
bool ac_lamp_dimmer::isComplete()
{
    return internal_current_duty == target_duty;
}