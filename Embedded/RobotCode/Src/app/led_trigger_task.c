#include "led_trigger_task.h"
#include "bsp_led.h"
#include "cmsis_os.h"
#include "IST8310driver.h"
#include "mpu6500driver.h"
#include "bsp_power_ctrl.h"

extern void MX_USB_DEVICE_Init(void);

	
void led_trigger_task(void const * argument)
{
    MX_USB_DEVICE_Init();
    while(1)
    {

        led_green_on();
        vTaskDelay(500);
        led_green_off();
        vTaskDelay(500);
    }

}
