/* FFT Example

   This example runs a few FFTs and measure the timing.

  Author: Robin Scheibler, 2017
   This code is released under MIT license. See the README for more details.§
*/
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "driver/rmt.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
// #include "driver/timer.h"
#include "driver/gpio.h"
#include "fft.h"
#include "led_strip.h"
#include "speech_srcif.h"
#include "lcd.h"
/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define REP 100
#define MIN_LOG_N 6
#define MAX_LOG_N 12
#define RMT_TX_GPIO 18
#define RMT_TX_CHANNEL 0
#define STRIP_LED_NUMBER 120
#define GPIO_OUTPUT 27
extern QueueHandle_t ledQueue;
double start, end;

// timer_config_t timer_config = {
//   .alarm_en = false,
//   .counter_en = true,
//   .counter_dir = TIMER_COUNT_UP,
//   .divider = 80   /* 1 us per tick */
// };

// gpio_config_t gpio_conf = {
//   // disable interrupt
//   .intr_type = GPIO_PIN_INTR_DISABLE,
//   //set as output mode
//   .mode = GPIO_MODE_OUTPUT,
//   //bit mask of the pins that you want to set,e.g.GPIO18/19
//   .pin_bit_mask = (1 << GPIO_OUTPUT),
//   //disable pull-down mode
//   .pull_down_en = 0,
//   //disable pull-up mode
//   .pull_up_en = 0
// };


// void clock_init()
// {
//   timer_init(TIMER_GROUP_0, TIMER_0, &timer_config);
//   timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
//   timer_start(TIMER_GROUP_0, TIMER_0);
// }

void led_task()
{
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(RMT_TX_GPIO, RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(STRIP_LED_NUMBER, (led_strip_dev_t)config.channel);
    led_strip_t *strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip)
    {
        // ESP_LOGE(TAG, "install WS2812 driver failed");
        printf("install WS2812 driver failed\n");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));
    // Show simple rainbow chasing pattern
    printf("LED Rainbow Chase Start\n");
    // ESP_LOGI(TAG, "LED Rainbow Chase Start");
    while (true) 
    {
		int16_t wave[16];
		// int16_t last_wave[8];
		xQueueReceive(ledQueue, wave, portMAX_DELAY);
		//10个灯为一组，共8组，按照S行排列，单数组从上到下1-10，双数行从下到上1-10
		for(int i=1;i<9;i++)
		{
			for(int j=0;j<wave[i];j++)
			{
				uint32_t r,g,b;
				if(j<3)//绿色
				{
					r=0;
					g=128;
					b=0;
				}
				else if(j<6)//橙色
				{
					r=128;
					g=64;
					b=0;
				}
				else//红色
				{
					r=128;
					g=b=0;
				}
				if(i%2==1)
				{
					strip->set_pixel(strip,(8-i)*10+j,r,g,b);
				}
				else
				{
					strip->set_pixel(strip,(8-i)*10+9-j,r,g,b);
				}
				
			}
			for(int j=wave[i];j<10;j++)
			{
				if(i%2==1)
				{
					strip->set_pixel(strip,(8-i)*10+j,0,0,0);
				}
				else
				{	
					strip->set_pixel(strip,(8-i)*10+9-j,0,0,0);
				}
				
			}
		}
		strip->refresh(strip,10);
    }  
}

void app_main()
{
	// Lcd_Init();
	// LCD_Clear(0xff00);
	speech_init();
	led_task();
	while (1)
	{
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}
