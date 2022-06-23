#include "ac_lamp_dimmer.h"


#ifdef __cplusplus
extern "C" {
#endif

void app_main(void);

#ifdef __cplusplus
}
#endif


const ac_lamp_dimmer_input_config_t sin_zc_gpio_config_data = 
{
    GPIO_NUM_2,
    0,
    1,
    0,
};
const ac_lamp_dimmer_input_config_t* sin_zc_gpio_config = &sin_zc_gpio_config_data;

const ac_lamp_dimmer_output_config_t hl_ctrl_config_data = 
{
    MCPWM_UNIT_0,
    GPIO_NUM_4,
    1,
};
const ac_lamp_dimmer_output_config_t* hl_ctrl_config = &hl_ctrl_config_data;


ac_lamp_dimmer bulb_control;


void ac_lamp_main_task(void *pvParameters)
{
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));

        bulb_control.set(100);

    #if AC_LAMP_IMMEDIATELY_OFF
        vTaskDelay(pdMS_TO_TICKS(5000));
    #else
        for(;!bulb_control.isComplete();vTaskDelay(1));
    #endif

        bulb_control.set(0);
    
        vTaskDelay(pdMS_TO_TICKS(2000));

        bulb_control.set(20);

        vTaskDelay(pdMS_TO_TICKS(10000));

        bulb_control.set(0);

        vTaskDelay(pdMS_TO_TICKS(4000));
    }
}


void app_main(void)
{
    bulb_control.init(sin_zc_gpio_config, hl_ctrl_config);

    xTaskCreate(ac_lamp_main_task, "ac lamp main task", 2000, NULL, 5, NULL);
}
