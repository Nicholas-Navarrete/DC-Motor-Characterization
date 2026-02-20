#include <Arduino_FreeRTOS.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).

// define two Tasks for Reading Console Inputs and Writing to PWM
void TaskConsoleInput( void *pvParameters );
void TaskWriteMotors( void *pvParameters );
TaskHandle_t ConsoleTaskHandle = NULL;
TaskHandle_t MotorTaskHandle = NULL;

//Initialize motor values
uint8_t MotorValue = 0;
uint8_t *MotorValuePointer =   &MotorValue;
SemaphoreHandle_t MotorMutex;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  MotorMutex = xSemaphoreCreateMutex();

  xTaskCreate(
  TaskConsoleInput
  ,  "ConsoleInput"  // A name just for humans
  ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
  ,  NULL //Parameters for the task
  ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  ,  &ConsoleTaskHandle ); //Task Handle

  xTaskCreate(
  TaskWriteMotors
  ,  "WriteMotors"  // A name just for humans
  ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
  ,  NULL //Parameters for the task
  ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  ,  &MotorTaskHandle ); //Task Handle

}

void loop() {
 //These things are done in tasks
}

void TaskConsoleInput( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  /*
  This task reads inputs from the console and writes the inputs to the Motor Value
  */
  Serial.print("Input Motor Value(0-255): ");
  for (;;){
    

    if ( xSemaphoreTake( MotorMutex, ( TickType_t ) 5 ) == pdTRUE ){
      if (Serial.available() > 0) {
        MotorValue = Serial.parseInt();
        if (Serial.peek() == '\n' || Serial.peek() == '\r'){
          Serial.read();  // clear newline
        }
        
      }
      xSemaphoreGive( MotorMutex ); // Now free or "Give" the Serial Port for others.
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

}

void TaskWriteMotors( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  /*
  This task reads inputs from the console and writes the inputs to the Motor Value
  */
  const int MotorPin = 3; // Pin 3 supports PWM
  pinMode(MotorPin, OUTPUT);
  for (;;){ 
    if ( xSemaphoreTake( MotorMutex, ( TickType_t ) 5 ) == pdTRUE ){
      analogWrite(MotorPin,MotorValue);
      xSemaphoreGive( MotorMutex ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
   
}
