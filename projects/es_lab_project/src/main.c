#include <stdlib.h>
#include "dorobo32.h"
#include "FreeRTOS.h"
#include "task.h"
#include "trace.h"
#include "fft.h"
#include "wifi.h"

static void ConfigUART();

static void StatusLED(void *pvParameters);
static void Switch_0(void *pvParameters);
static void Switch_1(void *pvParameters);
static void WiFi(void *pvParameters);

static void IRSens_0(void *pvParameters);
static void IRSens_1(void *pvParameters);


int main()
{
    dorobo_init();			//Call dorobo_init() function to initialize HAL, Clocks, Timers etc.	
	
    ConfigUART();

    //xTaskCreate(blinky, "BLINKYTASK", 512, NULL, 2, NULL);	//create blinky task
    xTaskCreate(Switch_0, "SWITCH0", 128, NULL, 3, NULL);
    xTaskCreate(Switch_1, "SWITCH1", 128, NULL, 3, NULL);
    xTaskCreate(IRSens_0, "IRSens0", 128, NULL, 2, NULL);
    //xTaskCreate(IRSens_1, "IRSens1", 128, NULL, 2, NULL);
    xTaskCreate(StatusLED, "STATUSLED", 128, NULL, 1, NULL);
    //xTaskCreate(WiFi, "WIFI", 512, NULL, 2, NULL);
    vTaskStartScheduler();	//start the freertos scheduler

	return 0;				//should not be reached!
}

static void StatusLED(void *pvParameters)
{
  trace_init();

  while (1)
  {
    led_green_toggle();
    vTaskDelay(20);        //delay the task for 20 ticks (1 ticks = 50 ms)
  }
}

static void Switch_1(void *pvParameters)
{
	trace_init();

	digital_configure_pin(DD_PIN_PD14, DD_CFG_INPUT_PULLUP);
	//digital_set_pin(DD_PIN_PF10, DD_LEVEL_HIGH);

	while (1) 
	{
	  vTaskDelay(20);

		if(digital_get_pin(DD_PIN_PD14) == DD_LEVEL_HIGH)
		  led_green(DD_LED_OFF);
		else
		  led_green(DD_LED_ON);
	}
}



static void Switch_0(void *pvParameters)
{
  trace_init();

  digital_configure_pin(DD_PIN_PD15, DD_CFG_INPUT_PULLUP);
  //digital_configure_pin(DD_PIN_PF9, DD_CFG_OUTPUT);
  //digital_set_pin(DD_PIN_PF9, DD_LEVEL_HIGH);

  while (1)
  {
    vTaskDelay(20);

    if(digital_get_pin(DD_PIN_PD15) == DD_LEVEL_HIGH)
      led_red(DD_LED_OFF);
    else
      led_red(DD_LED_ON);

    //vTaskSuspend(NULL);

    /*vTaskDelay(200);
    digital_set_pin(DD_PIN_PF9, DD_LEVEL_HIGH);
    vTaskDelay(200);
    digital_set_pin(DD_PIN_PF9, DD_LEVEL_LOW);*/

  }
}



static void WiFi(void *pvParameters)
{
  trace_init();

  wifi_init();

  while (1)
  {
  }
}

static void ConfigUART()
{
  uart_init();
}


static void IRSens_0(void *pvParameters)
{
  trace_init();
  ft_init();

  //vTaskDelay(1);

  uint16_t IRSense0;

  digital_configure_pin(DD_PIN_PF10, DD_CFG_INPUT_PULLDOWN);
  //digital_set_pin(DD_PIN_PF10, DD_LEVEL_LOW);

  while (1)
  {
    ft_start_sampling(DD_PIN_PF10);

    while(!ft_is_sampling_finished(DD_PIN_PF10))
    {
      vTaskDelay(1);
    }

    //ft_set_sampling_finished_handler();
    IRSense0 = ft_get_transform(DFT_FREQ100);
  }
}

/*static void IRSens_1(void *pvParameters)
{
  trace_init();

  digital_configure_pin(DD_PIN_PF9, DD_CFG_INPUT_PULLDOWN);
  digital_set_pin(DD_PIN_PF9, DD_LEVEL_LOW);

  while (1)
  {


  }
}*/


