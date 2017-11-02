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
#include "queue.h"

#define SWITCH0 DD_PIN_PD14
#define SWITCH1 DD_PIN_PD15
#define IRSENSOR0 DD_PIN_PF10
#define IRSENSOR1 DD_PIN_PF9
#define IRDISTANCE0 DA_ADC_CHANNEL0
#define IRDISTANCE1 DA_ADC_CHANNEL1

QueueHandle_t Switches_Queue;
QueueHandle_t IRSensors_Queue;

static void StatusLED(void *pvParameters);
static void MainHandler(void *pvParameters);

static void Switches(void *pvParameters);

static void IRSensors(void *pvParameters);
static void IRDistance(void *pvParameters);

struct Switches {
  DD_DIP_STATE_T Switch_0;
  DD_DIP_STATE_T Switch_1;
};

struct IRSensors {
  uint16_t IRSensor_0;
  uint16_t IRSensor_1;
};

static uint32_t IRDistanceDescriptor[6][2] = {
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
    wifi_init();
    led_red(DD_LEVEL_HIGH);

    if(xTaskCreate(Switches, "SWITCHES", 128, NULL, 2, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) traces("SWITCHES ERROR");
    if(xTaskCreate(IRSensors, "IRSENSORS", 512, NULL, 2, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) traces("IRSENSORS ERROR");
    //if(xTaskCreate(IRDistance, "IRDISTANCE", 256, NULL, 2, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) led_red(DD_LEVEL_HIGH);
    if(xTaskCreate(MainHandler, "MAINHANDLER", 512, NULL, 2, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) traces("MAINHANDLER ERROR");
    //if(xTaskCreate(StatusLED, "STATUSLED", 128, NULL, 1, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) traces("STATUSLED ERROR");

    Switches_Queue = xQueueCreate(1, sizeof(struct Switches*));
    Switches_Queue = xQueueCreate(1, sizeof(struct IRSensors*));

    vTaskStartScheduler();	//start the freertos scheduler

  return 0;				//should not be reached!
}

static void MainHandler(void *pvParameters)
{
  portBASE_TYPE Status;

  struct Switches Switches_State;
  struct IRSensors IRSensors_Value;

  while (1)
  {
    vTaskDelay(5);

    traces("MAINHANDLER");

    Status = xQueueReceive(Switches_Queue, &Switches_State, 0);
    //Status = xQueueReceive(IRSensors_Queue, &IRSensors_Value, 0);

    if(Status == pdPASS) {
      //char data[16];
      //sprintf(data, "%d_%d_" PRIu16 "_" PRIu16, Switches_State.Switch_0, Switches_State.Switch_1, IRSensors_Value.IRSensor_0, IRSensors_Value.IRSensor_1);
      char data[5];
      sprintf(data, "%d_%d_", Switches_State.Switch_0, Switches_State.Switch_1);
      traces(data);
    }
  }
}

static void StatusLED(void *pvParameters)
{
  trace_init();

  while (1)
  {
    // Shows, that the board is still operating
    vTaskDelay(20);        //delay the task for 20 ticks (1 ticks = 50 ms)
    led_green_toggle();
  }
}

static void Switches(void *pvParameters)
{
	trace_init();
	// Configure the defined pins as pullup input
	digital_configure_pin(SWITCH0, DD_CFG_INPUT_PULLUP);
	digital_configure_pin(SWITCH1, DD_CFG_INPUT_PULLUP);
	// Set the variable, which holds the state
	struct Switches Switches_State;

	while (1) 
	{
	  // Wait for ~500ms
	  vTaskDelay(10);
	  // Read the pins state into a struct
	  Switches_State.Switch_0 = digital_get_pin(SWITCH0);
	  Switches_State.Switch_1 = digital_get_pin(SWITCH1);
	  // Send the pins state into the queue
	  xQueueSend(Switches_Queue, (void*)&Switches_State, 0);
	}
}

static void IRSensors(void *pvParameters)
{
  trace_init();
  ft_init();
  // Configure the defined pins as pullup input
  digital_configure_pin(IRSENSOR0, DD_CFG_INPUT_PULLUP);
  digital_configure_pin(IRSENSOR1, DD_CFG_INPUT_PULLUP);
  // Set the variable, which holds the measurements
  struct IRSensors IRSensors_Value;

  while (1)
  {
    vTaskDelay(10);

    ft_start_sampling(IRSENSOR0);
    while(!ft_is_sampling_finished()) vTaskDelay(1);
    IRSensors_Value.IRSensor_0 = ft_get_transform(DFT_FREQ100);

    ft_start_sampling(IRSENSOR1);
    while(!ft_is_sampling_finished()) vTaskDelay(1);
    IRSensors_Value.IRSensor_1 = ft_get_transform(DFT_FREQ100);

    // Send the pins state into the queue
    xQueueSend(IRSensors_Queue, (void*)&IRSensors_Value, 0);
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
  }
}

