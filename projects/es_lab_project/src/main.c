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
#define IRDISTANCE0 DA_ADC_CHANNEL1
#define IRDISTANCE1 DA_ADC_CHANNEL0

#define MOTOR0 DM_MOTOR1
#define MOTOR1 DM_MOTOR2
#define MOTOR2 DM_MOTOR0

#define SWITCHON DD_LEVEL_LOW
#define SWITCHOFF DD_LEVEL_HIGH

#define ONSWITCH DD_DIP2

struct Switches {
  DD_DIP_STATE_T Switch_Right;
  DD_DIP_STATE_T Switch_Left;
};

struct IRSensors {
  uint16_t IRSensor_Right;
  uint16_t IRSensor_Left;
};

struct IRDistances {
  uint16_t IRDistance_Right;
  uint16_t IRDistance_Left;
};

enum MOTOR_SPEEDS {
    LOWSPEED = 50,
    FULLSPEED = 75
};
typedef enum MOTOR_SPEEDS MOTOR_SPEED;

enum GO_DIRECTIONS {
    FORWARD = -1,
    BACKWARD = 1
};
typedef enum GO_DIRECTIONS GO_DIRECTION;

enum TURN_DIRECTIONS {
    RIGHT = -1,
    LEFT = 1
};
typedef enum TURN_DIRECTIONS TURN_DIRECTION;

typedef int8_t ANGLE;

struct MotorsCommand {
  int8_t Speed_0;
  int8_t Speed_1;
  int8_t Speed_2;
};

enum COMMANDS {
    RETREAT = 1,
    SCAN = 2
};
typedef enum COMMANDS COMMAND;

struct SuperVise {
  COMMAND Command;
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

QueueHandle_t SuperVisor_Queue;
QueueHandle_t SuperVisor_Motors_Queue;

// Main functions
static void StatusLED(void *pvParameters);
static void MainHandler(void *pvParameters);
static void SuperVisor(void *pvParameters);
// Sensors
static void Switches(void *pvParameters);
static void IRSensors(void *pvParameters);
static void IRDistances(void *pvParameters);
// Actuators
static void Motors(void *pvParameters);
// Auxiliary
static void RoverGo(MOTOR_SPEED, GO_DIRECTION);
static void RoverTurn(MOTOR_SPEED, TURN_DIRECTION, ANGLE);
static void RoverStop();
static void RoverChangeDirection(MOTOR_SPEED, TURN_DIRECTION);

static bool isRoverStopped();

int main()
{
    dorobo_init();			//Call dorobo_init() function to initialize HAL, Clocks, Timers etc.	
    wifi_init();

    if(xTaskCreate(Motors, "MOTORS", 64, NULL, 2, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) traces("MOTORS ERROR");
    if(xTaskCreate(Switches, "SWITCHES", 64, NULL, 2, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) traces("SWITCHES ERROR");
    if(xTaskCreate(IRSensors, "IRSENSORS", 64, NULL, 2, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) traces("IRSENSORS ERROR");
    if(xTaskCreate(IRDistances, "IRDISTANCES", 64, NULL, 2, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) traces("IRDISTANCES ERROR");
    if(xTaskCreate(MainHandler, "MAINHANDLER", 256, NULL, 2, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) traces("MAINHANDLER ERROR");
    if(xTaskCreate(SuperVisor, "SUPERVISOR", 64, NULL, 1, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) traces("SUPERVISOR ERROR");
    if(xTaskCreate(StatusLED, "STATUSLED", 32, NULL, 1, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) traces("STATUSLED ERROR");

    Switches_Queue = xQueueCreate(1, sizeof(struct Switches*));
    IRSensors_Queue = xQueueCreate(1, sizeof(struct IRSensors*));
    IRDistances_Queue = xQueueCreate(1, sizeof(struct IRDistances*));

    Motors_Queue = xQueueCreate(1, sizeof(struct Motors*));

    SuperVisor_Queue = xQueueCreate(1, sizeof(struct SuperVisor*));
    SuperVisor_Motors_Queue = xQueueCreate(5, sizeof(struct Motors*));

    vTaskStartScheduler();	//start the freertos scheduler

  return 0;				//should not be reached!
}

static void MainHandler(void *pvParameters)
{
  struct Switches Switches_State;
  struct IRSensors IRSensors_Value;
  struct IRDistances IRDistances_Value;

  struct SuperVisor SuperVisor_Commands;

  portBASE_TYPE Status;

  // Wait for 1 second
  vTaskDelay(1000/portTICK_PERIOD_MS);

  // Main power switch configuration
  digital_configure_pin(ONSWITCH, DD_CFG_INPUT_PULLUP);

  while (1)
  {
    // Stop the rover in runtime
    if(digital_get_dip(ONSWITCH) == DD_DIP_OFF) {
      RoverStop();
      // Block the main thread
      while(digital_get_dip(ONSWITCH) == DD_DIP_OFF);
      // Wait for 2 second
      vTaskDelay(2000/portTICK_PERIOD_MS);
    }

    // Wait for 20ms
    vTaskDelay(20/portTICK_PERIOD_MS);


    xQueueReceive(Switches_Queue, &Switches_State, 0);
    xQueueReceive(IRSensors_Queue, &IRSensors_Value, 0);
    xQueueReceive(IRDistances_Queue, &IRDistances_Value, 0);

    Status = xQueueReceive(SuperVisor_Queue, &SuperVisor_Command, 0);
    
    if(Status == pdPASS) { 
      switch(SuperVisor_Commands.Command) {
        case RETREAT: {
          RoverStop();
          RoverTurn(LOWSPEED, LEFT, 180);
          RoverStop();
        } break;
        case SCAN: break;
      }
    }

    // What we should do;
    // 1. Turn, until all of the IRSensors receiving
    // 2. Go into that direction
    // 3. Look out for the IRDistances
    // 4. If something is too close -> Turn -> Direction?
    // 5. Look out for switches
    // 6. If collision detected -> Stop -> Go backwards -> Stop -> Turn -> Direction?

    // Implement controlling logic

    bool GO = true;

    if(Switches_State.Switch_Right == SWITCHON || Switches_State.Switch_Left == SWITCHON) {
      RoverStop();
      RoverGo(FULLSPEED, BACKWARD);
      vTaskDelay(500/portTICK_PERIOD_MS);
      RoverStop();

      if(Switches_State.Switch_Right == SWITCHON && Switches_State.Switch_Left == SWITCHOFF)
        RoverTurn(LOWSPEED, LEFT, 60);
      else
        RoverTurn(LOWSPEED, RIGHT, 60);

      RoverStop();
    }


    if(IRDistances_Value.IRDistance_Right <= 10 || IRDistances_Value.IRDistance_Left <= 10) {

    	GO = false;

    	if(IRDistances_Value.IRDistance_Right <= 10 && IRDistances_Value.IRDistance_Left > 10) {

    		RoverChangeDirection(FULLSPEED, LEFT);

		} else if(IRDistances_Value.IRDistance_Left <= 10 && IRDistances_Value.IRDistance_Right > 10) {

			RoverChangeDirection(FULLSPEED, RIGHT);

		} else if(IRDistances_Value.IRDistance_Right <= 10 && IRDistances_Value.IRDistance_Left <= 10) {

			RoverStop();
			RoverGo(FULLSPEED, BACKWARD);
			vTaskDelay(500/portTICK_PERIOD_MS);
			RoverStop();
			RoverTurn(LOWSPEED, RIGHT, 60);
			RoverStop();
		}
    }


    // Main task -> Keep the target locked
    if(IRSensors_Value.IRSensor_Right != 0 || IRSensors_Value.IRSensor_Left != 0) {

    	GO = false;

		if(IRSensors_Value.IRSensor_Right != 0 && IRSensors_Value.IRSensor_Left == 0) {
			RoverTurn(LOWSPEED, LEFT, 0);

		} else if(IRSensors_Value.IRSensor_Right == 0 && IRSensors_Value.IRSensor_Left != 0) {
			RoverTurn(LOWSPEED, RIGHT, 0);

		} else
			GO = true;
    }


    if(GO) RoverGo(FULLSPEED, FORWARD);

    //char data[28];
    //sprintf(data, "%d_%d_%" PRIu16 "_%" PRIu16 "_%" PRIu16 "_%" PRIu16, Switches_State.Switch_Right, Switches_State.Switch_Left, IRSensors_Value.IRSensor_Right, IRSensors_Value.IRSensor_Left, IRDistances_Value.IRDistance_Right, IRDistances_Value.IRDistance_Left);
    //sprintf(data,"%" PRIu16 "_%" PRIu16, IRSensors_Value.IRSensor_Right, IRSensors_Value.IRSensor_Left);

    //traces(data);
  }
}

static bool isRoverStopped() {
  if(motor_get_speed(MOTOR0) == 0 &&
     motor_get_speed(MOTOR1) == 0 &&
     motor_get_speed(MOTOR2) == 0) return true;
  return false;
}

static void RoverGo(MOTOR_SPEED speed, GO_DIRECTION direction) {
  struct MotorsCommand Motors_Speeds;
  Motors_Speeds.Speed_0 = 0;
  Motors_Speeds.Speed_1 = speed * direction;
  Motors_Speeds.Speed_2 = speed * direction * (-1);
  xQueueOverwrite(Motors_Queue, (void*)&Motors_Speeds);
}

static void RoverChangeDirection(MOTOR_SPEED speed, TURN_DIRECTION direction) {
  struct MotorsCommand Motors_Speeds;
  Motors_Speeds.Speed_0 = direction * speed;
  if(direction == RIGHT) {
	  Motors_Speeds.Speed_1 = 0;
	  Motors_Speeds.Speed_2 = LOWSPEED * direction;
  } else {
	  Motors_Speeds.Speed_1 = LOWSPEED * direction;
	  Motors_Speeds.Speed_2 = 0;
  }

  xQueueOverwrite(Motors_Queue, (void*)&Motors_Speeds);
}

static void RoverTurn(MOTOR_SPEED speed, TURN_DIRECTION direction, ANGLE angle) {

  struct MotorsCommand Motors_Speeds;
  Motors_Speeds.Speed_0 = speed * direction;
  Motors_Speeds.Speed_1 = speed * direction;
  Motors_Speeds.Speed_2 = speed * direction;
  xQueueOverwrite(Motors_Queue, (void*)&Motors_Speeds);

  float turn_time = ((((float)2920/(float)360)*(float)angle)/(float)speed*(float)LOWSPEED);
  vTaskDelay(turn_time/portTICK_PERIOD_MS);
}

static void RoverStop() {
  struct MotorsCommand Motors_Speeds;
  Motors_Speeds.Speed_0 = 0;
  Motors_Speeds.Speed_1 = 0;
  Motors_Speeds.Speed_2 = 0;
  xQueueOverwrite(Motors_Queue, (void*)&Motors_Speeds);
  // Default 250ms delay after stop
  vTaskDelay(250/portTICK_PERIOD_MS);
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
    // Wait for 20ms
    vTaskDelay(20/portTICK_PERIOD_MS);
    // Get the next instruction from the queue
    Status = xQueueReceive(Motors_Queue, &Motors_Speeds, 100/portTICK_PERIOD_MS);
    // Process it
    if(Status == pdPASS) {

      if( motor_get_speed(MOTOR0) != Motors_Speeds.Speed_0 ||
		      motor_get_speed(MOTOR1) != Motors_Speeds.Speed_1 ||
		      motor_get_speed(MOTOR2) != Motors_Speeds.Speed_2) {

		    motor_set(MOTOR0, Motors_Speeds.Speed_0);
		    motor_set(MOTOR1, Motors_Speeds.Speed_1);
		    motor_set(MOTOR2, Motors_Speeds.Speed_2);

        xQueueSend(SuperVisor_Motors_Queue, (void*)&Motors_Speeds);
      }
    }
  }
}

static void SuperVisor(void *pvParameters)
{
  trace_init();

  while (1)
  {
    // Wait for 1000ms
    vTaskDelay(1000/portTICK_PERIOD_MS);

    if(uxQueueSpacesAvailable(SuperVisor_Motors_Queue) == 0) {
      struct SuperVisor SuperVisor_Commands;
      SuperVisor_Commands.Command = RETREAT;
      xQueueSend(SuperVisor_Queue, (void*)&SuperVisor_Commands);
    }

    xQueueReset(SuperVisor_Motors_Queue);

  }
}

static void StatusLED(void *pvParameters)
{
  trace_init();

  while (1)
  {
    // Shows, that the board is still operating
    // Wait for 500ms (1 second)
    vTaskDelay(500/portTICK_PERIOD_MS);
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
	  // Wait for 50ms
	  vTaskDelay(50/portTICK_PERIOD_MS);
	  // Read the pins state into a struct
	  Switches_State.Switch_Right = digital_get_pin(SWITCH0);
	  Switches_State.Switch_Left = digital_get_pin(SWITCH1);
	  // Send the pins state into the queue (overwrite unread values)
	  xQueueOverwrite(Switches_Queue, (void*)&Switches_State);
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
    // Wait for 50ms
    vTaskDelay(50/portTICK_PERIOD_MS);
    // Sample from the first sensor
    ft_start_sampling(IRSENSOR0);
    while(!ft_is_sampling_finished()) vTaskDelay(10/portTICK_PERIOD_MS); // Wait for 10ms
    IRSensors_Value.IRSensor_Right = ft_get_transform(DFT_FREQ100);
    // Sample from the second sensor
    ft_start_sampling(IRSENSOR1);
    while(!ft_is_sampling_finished()) vTaskDelay(10/portTICK_PERIOD_MS); // Wait for 10ms
    IRSensors_Value.IRSensor_Left = ft_get_transform(DFT_FREQ100);
    // Send the pins state into the queue (overwrite unread values)
    xQueueOverwrite(IRSensors_Queue, (void*)&IRSensors_Value);
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
    IRDistances_Value.IRDistance_Right = 0;
    IRDistances_Value.IRDistance_Left = 0;
    for(uint8_t index = 0; index < 5; index ++) {
      IRDistances_Value.IRDistance_Right += (uint16_t)adc_get_value(IRDISTANCE0);
      IRDistances_Value.IRDistance_Left += (uint16_t)adc_get_value(IRDISTANCE1);
      // Wait for 10ms
      vTaskDelay(10/portTICK_PERIOD_MS);
    }
    IRDistances_Value.IRDistance_Right /= 5;
    IRDistances_Value.IRDistance_Left /= 5;

    // Calculate the first distance from the lookup table
    for(uint8_t index = 0; index < 5; index ++) {
      if(IRDistances_Value.IRDistance_Right <= IRDistancesLookupTable[index][0] && IRDistances_Value.IRDistance_Right > IRDistancesLookupTable[index+1][0]) {
        IRDistances_Value.IRDistance_Right = IRDistancesLookupTable[index][1];
        break;
      }
    }
    // Calculate the second distance from the lookup table
    for(uint8_t index = 0; index < 5; index ++) {
      if(IRDistances_Value.IRDistance_Left <= IRDistancesLookupTable[index][0] && IRDistances_Value.IRDistance_Left > IRDistancesLookupTable[index+1][0]) {
        IRDistances_Value.IRDistance_Left = IRDistancesLookupTable[index][1];
        break;
      }
    }
    // Send the pins state into the queue (overwrite unread values)
    xQueueOverwrite(IRDistances_Queue, (void*)&IRDistances_Value);
  }
}
