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
#include "motor.h"
#include "queue.h"

#define SWITCH0 DD_PIN_PD14
#define SWITCH1 DD_PIN_PD15
#define IRSENSOR0 DD_PIN_PF10
#define IRSENSOR1 DD_PIN_PF9
#define IRDISTANCE0 DA_ADC_CHANNEL0
#define IRDISTANCE1 DA_ADC_CHANNEL1

#define MOTOR0 DM_MOTOR0
#define MOTOR1 DM_MOTOR1
#define MOTOR2 DM_MOTOR2

#define SWITCHON DD_LEVEL_LOW
#define SWITCHOFF DD_LEVEL_HIGH

struct Switches {
  DD_DIP_STATE_T Switch_0;
  DD_DIP_STATE_T Switch_1;
};

struct IRSensors {
  uint16_t IRSensor_0;
  uint16_t IRSensor_1;
};

struct IRDistances {
  uint16_t IRDistance_0;
  uint16_t IRDistance_1;
};

enum MOTOR_SPEEDS {
    LOWSPEED = 20,
    FULLSPEED = 50
};
typedef enum MOTOR_SPEEDS MOTOR_SPEED;

enum MOTOR_DIRECTIONS {
    FORWARD = 1,
    BACKWARD = -1
};
typedef enum MOTOR_DIRECTIONS MOTOR_DIRECTION;

struct MotorsCommand {
  int8_t Speed_0;
  int8_t Speed_1;
  int8_t Speed_2;
};

static const uint16_t IRDistancesLookupTable[6][2] = {
    {4095, 0}, // 0cm
    {3400, 10}, // 10cm
    {1900, 20}, // 20cm
    {1000, 40}, // 40cm
    {500, 80}, // 80cm
    {0, 100}, // Overflow protection
};

QueueHandle_t Switches_Queue;
QueueHandle_t IRSensors_Queue;
QueueHandle_t IRDistances_Queue;

QueueHandle_t Motors_Queue;

// Main functions
static void StatusLED(void *pvParameters);
static void MainHandler(void *pvParameters);
// Sensors
static void Switches(void *pvParameters);
static void IRSensors(void *pvParameters);
static void IRDistances(void *pvParameters);
// Actuators
static void Motors(void *pvParameters);
// Auxiliary
static void RoverGo(MOTOR_SPEED, MOTOR_DIRECTION);
static void RoverTurn(MOTOR_SPEED);
static void RoverStop();

int main()
{
    dorobo_init();			//Call dorobo_init() function to initialize HAL, Clocks, Timers etc.	
    wifi_init();
    led_red(DD_LEVEL_HIGH);

    if(xTaskCreate(Motors, "MOTORS", 64, NULL, 2, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) traces("MOTORS ERROR");
    if(xTaskCreate(Switches, "SWITCHES", 64, NULL, 2, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) traces("SWITCHES ERROR");
    if(xTaskCreate(IRSensors, "IRSENSORS", 64, NULL, 2, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) traces("IRSENSORS ERROR");
    if(xTaskCreate(IRDistances, "IRDISTANCES", 64, NULL, 2, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) traces("IRDISTANCES ERROR");
    if(xTaskCreate(MainHandler, "MAINHANDLER", 256, NULL, 2, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) traces("MAINHANDLER ERROR");
    if(xTaskCreate(StatusLED, "STATUSLED", 32, NULL, 1, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) traces("STATUSLED ERROR");

    Switches_Queue = xQueueCreate(1, sizeof(struct Switches*));
    IRSensors_Queue = xQueueCreate(1, sizeof(struct IRSensors*));
    IRDistances_Queue = xQueueCreate(1, sizeof(struct IRDistances*));

    Motors_Queue = xQueueCreate(1, sizeof(struct Motors*));

    vTaskStartScheduler();	//start the freertos scheduler

  return 0;				//should not be reached!
}

static void MainHandler(void *pvParameters)
{
  portBASE_TYPE Status;

  struct Switches Switches_State;
  struct IRSensors IRSensors_Value;
  struct IRDistances IRDistances_Value;

  while (1)
  {
    // Wait for ~100ms
    vTaskDelay(2);

    Status = xQueueReceive(Switches_Queue, &Switches_State, 5);
    Status = xQueueReceive(IRSensors_Queue, &IRSensors_Value, 5);
    Status = xQueueReceive(IRDistances_Queue, &IRDistances_Value, 5);

    // What we should do;
    // 1. Turn, until all of the IRSensors receiving
    // 2. Go into that direction
    // 3. Look out for the IRDistances
    // 4. If something is too close -> Turn -> Direction?
    // 5. Look out for switches
    // 6. If collision detected -> Stop -> Go backwards -> Stop -> Turn -> Direction?

    // Implement controlling logic
    if(Switches_State.Switch_0 == SWITCHON) {
      //RoverTurn(LOWSPEED);
      RoverGo(FULLSPEED, FORWARD);
    }

    if(Switches_State.Switch_1 == SWITCHON) {
      RoverStop();
      RoverGo(FULLSPEED, BACKWARD);
      vTaskDelay(100);
      RoverStop();
      RoverTurn(LOWSPEED);
      vTaskDelay(60);
      RoverStop();
    }
    // ----- *** -----

    if(Status == pdPASS) {
      char data[28];
      sprintf(data, "%d_%d_%" PRIu16 "_%" PRIu16 "_%" PRIu16 "_%" PRIu16, Switches_State.Switch_0, Switches_State.Switch_1, IRSensors_Value.IRSensor_0, IRSensors_Value.IRSensor_1, IRDistances_Value.IRDistance_0, IRDistances_Value.IRDistance_1);
      traces(data);
    }
  }
}

static void RoverGo(MOTOR_SPEED speed, MOTOR_DIRECTION direction) {
  struct MotorsCommand Motors_Speeds;
  Motors_Speeds.Speed_0 = 0;
  Motors_Speeds.Speed_1 = speed * direction;
  Motors_Speeds.Speed_2 = speed * direction * (-1);
  xQueueSend(Motors_Queue, (void*)&Motors_Speeds, 2);
}

static void RoverTurn(MOTOR_SPEED speed) {
  struct MotorsCommand Motors_Speeds;
  Motors_Speeds.Speed_0 = speed;
  Motors_Speeds.Speed_1 = speed;
  Motors_Speeds.Speed_2 = speed;
  xQueueSend(Motors_Queue, (void*)&Motors_Speeds, 2);
}

static void RoverStop() {
  struct MotorsCommand Motors_Speeds;
  Motors_Speeds.Speed_0 = 0;
  Motors_Speeds.Speed_1 = 0;
  Motors_Speeds.Speed_2 = 0;
  xQueueSend(Motors_Queue, (void*)&Motors_Speeds, 2);
  // Default 500ms delay after stop
  vTaskDelay(500/portTICK_PERIOD_MS);
}

static void Motors(void *pvParameters)
{
  trace_init();
  motor_init();
  portBASE_TYPE Status;
  // Set the variable, which holds the commands
  struct MotorsCommand Motors_Speeds;

  while (1)
  {
    // Wait for ~100ms
    vTaskDelay(2);
    // Get the next instruction from the queue
    Status = xQueueReceive(Motors_Queue, &Motors_Speeds, 2);
    // Process it
    if(Status == pdPASS) {
      motor_set(MOTOR0, Motors_Speeds.Speed_0);
      motor_set(MOTOR1, Motors_Speeds.Speed_1);
      motor_set(MOTOR2, Motors_Speeds.Speed_2);
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
	  // Wait for ~250ms
	  vTaskDelay(5);
	  // Read the pins state into a struct
	  Switches_State.Switch_0 = digital_get_pin(SWITCH0);
	  Switches_State.Switch_1 = digital_get_pin(SWITCH1);
	  // Send the pins state into the queue
	  xQueueSend(Switches_Queue, (void*)&Switches_State, 2);
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
    // Wait for ~250ms
    vTaskDelay(5);
    // Sample from the first sensor
    ft_start_sampling(IRSENSOR0);
    while(!ft_is_sampling_finished()) vTaskDelay(1);
    IRSensors_Value.IRSensor_0 = ft_get_transform(DFT_FREQ100);
    // Sample from the second sensor
    ft_start_sampling(IRSENSOR1);
    while(!ft_is_sampling_finished()) vTaskDelay(1);
    IRSensors_Value.IRSensor_1 = ft_get_transform(DFT_FREQ100);
    // Send the pins state into the queue
    xQueueSend(IRSensors_Queue, (void*)&IRSensors_Value, 2);
  }
}

static void IRDistances(void *pvParameters)
{
  trace_init();
  adc_init();
  // Set the variable, which holds the measurements
  struct IRDistances IRDistances_Value;

  while (1)
  {
    // Read the average of five samples into a struct
    IRDistances_Value.IRDistance_0 = 0;
    IRDistances_Value.IRDistance_1 = 0;
    for(uint8_t index = 0; index < 5; index ++) {
      IRDistances_Value.IRDistance_0 += (uint16_t)adc_get_value(IRDISTANCE0);
      IRDistances_Value.IRDistance_1 += (uint16_t)adc_get_value(IRDISTANCE1);
      // Wait for ~50ms
      vTaskDelay(1);
    }
    IRDistances_Value.IRDistance_0 /= 5;
    IRDistances_Value.IRDistance_1 /= 5;

    // Calculate the first distance from the lookup table
    for(uint8_t index = 0; index < 5; index ++) {
      if(IRDistances_Value.IRDistance_0 <= IRDistancesLookupTable[index][0] && IRDistances_Value.IRDistance_0 > IRDistancesLookupTable[index+1][0]) {
        IRDistances_Value.IRDistance_0 = IRDistancesLookupTable[index][1];
        break;
      }
    }
    // Calculate the second distance from the lookup table
    for(uint8_t index = 0; index < 5; index ++) {
      if(IRDistances_Value.IRDistance_1 <= IRDistancesLookupTable[index][0] && IRDistances_Value.IRDistance_1 > IRDistancesLookupTable[index+1][0]) {
        IRDistances_Value.IRDistance_1 = IRDistancesLookupTable[index][1];
        break;
      }
    }
    // Send the pins state into the queue
    xQueueSend(IRDistances_Queue, (void*)&IRDistances_Value, 2);
  }
}
