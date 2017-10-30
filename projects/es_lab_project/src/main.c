#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "dorobo32.h"
#include "FreeRTOS.h"
#include "task.h"
#include "trace.h"
#include "fft.h"
#include "wifi.h"
#include "adc.h"

#define IRSENSOR0 DD_PIN_PF10
#define IRSENSOR1 DD_PIN_PF9
#define IRDISTANCE0 DA_ADC_CHANNEL0
#define IRDISTANCE1 DA_ADC_CHANNEL1

static void ConfigWiFi();

static void StatusLED(void *pvParameters);
static void Switch_0(void *pvParameters);
static void Switch_1(void *pvParameters);

static void IRSensors(void *pvParameters);
static void IRDistance(void *pvParameters);

static uint32_t IRDistanceDescriptor[4][2] = {
    {4095, 0}, // 0cm
    {3400, 10}, // 10cm
    {1900, 20}, // 20cm
    {1000, 40}, // 40cm
    {500, 80}, // 80cm
    {0, 100}, // Overflow protection
};

int main()
{
    dorobo_init();			//Call dorobo_init() function to initialize HAL, Clocks, Timers etc.	
	
    ConfigWiFi();

    //xTaskCreate(blinky, "BLINKYTASK", 512, NULL, 2, NULL);	//create blinky task
    //xTaskCreate(Switch_0, "SWITCH0", 128, NULL, 3, NULL);
    //xTaskCreate(Switch_1, "SWITCH1", 128, NULL, 3, NULL);
    xTaskCreate(IRSensors, "IRSENSORS", 256, NULL, 2, NULL);
    xTaskCreate(IRDistance, "IRDISTANCE", 256, NULL, 2, NULL);
    xTaskCreate(StatusLED, "STATUSLED", 128, NULL, 1, NULL);

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



static void ConfigWiFi()
{
  trace_init();

  wifi_init();
}

static void IRSensors(void *pvParameters)
{
  trace_init();
  ft_init();

  digital_configure_pin(IRSENSOR0, DD_CFG_INPUT_PULLUP);
  digital_configure_pin(IRSENSOR1, DD_CFG_INPUT_PULLUP);

  uint16_t IRSensor0;
  uint16_t IRSensor1;
  char data[14];

  while (1)
  {
    ft_start_sampling(IRSENSOR0);

    while(!ft_is_sampling_finished())
    {
      vTaskDelay(1);
    }

    IRSensor0 = ft_get_transform(DFT_FREQ100);

    ft_start_sampling(IRSENSOR1);

    while(!ft_is_sampling_finished())
    {
      vTaskDelay(1);
    }

    IRSensor1 = ft_get_transform(DFT_FREQ100);

    //sprintf(data,"%" PRIu16 "_%" PRIu16, IRSensor0, IRSensor1);
    //traces(data);
  }
}

static void IRDistance(void *pvParameters)
{
  trace_init();
  adc_init();

  uint32_t IRDistance0;
  uint32_t Distance;

  char data[21];

  while (1)
  {
    vTaskDelay(10);

    IRDistance0 = adc_get_value(IRDISTANCE0);

    for(uint8_t index = 0; index < 5; index ++) {
      if(IRDistance0 <= IRDistanceDescriptor[index][0] && IRDistance0 > IRDistanceDescriptor[index+1][0]) {
        Distance = IRDistanceDescriptor[index][1];
        break;
      }
    }

    sprintf(data,"%" PRIu32 "_%" PRIu32, IRDistance0, Distance);
    traces(data);
  }
}

