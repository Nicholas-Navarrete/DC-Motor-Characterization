#include <Arduino_FreeRTOS.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
#include <stdlib.h>

/*
Currently this code does not do active breaking of the motor but it can drive the speed forwards and backwards.
Pin 3 controls the speed and should be connected to the enable pin of the L293D
Pins 7 and 8 control the direction and should be connected to Diver inputs 1 and 2 of the L293D respectively
*/


// define two Tasks for Reading Console Inputs and Writing to PWM
void TaskConsoleInput( void *pvParameters );
void TaskWriteMotors( void *pvParameters );
void TaskConsoleWrite( void *pvParameters );
TaskHandle_t ConsoleTaskHandle = NULL;
TaskHandle_t MotorTaskHandle = NULL;
TaskHandle_t ConsoleWriteHandle = NULL;

//Initialize motor values
enum Direction{
  Forwards,
  Backwards
};

uint8_t MotorValue = 0;
uint8_t MotorDirection = Forwards;
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

  xTaskCreate(
  TaskConsoleWrite
  ,  "ConsoleWrite"  // A name just for humans
  ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
  ,  NULL //Parameters for the task
  ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  ,  &ConsoleWriteHandle ); //Task Handle
}

void loop() {
 //These things are done in tasks
}

void TaskConsoleInput( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  /*
  This task reads inputs from the console and writes the inputs to the Motor Value
  */
  int16_t InputMotorValue = 0;
  for (;;){
    

    if ( xSemaphoreTake( MotorMutex, ( TickType_t ) 5 ) == pdTRUE ){
      if (Serial.available() > 0) {

        InputMotorValue = Serial.parseInt();
        if (Serial.peek() == '\n' || Serial.peek() == '\r'){
          Serial.read();  // clear newline
        }
        
        //Check Direction
        if(InputMotorValue>=0){
          MotorDirection = Forwards;
        }
        else{
          MotorDirection = Backwards;
        }
        
        //SetValue
        if(InputMotorValue>=-255 && InputMotorValue<=255){
          MotorValue = abs(InputMotorValue);
        }

      }
      xSemaphoreGive( MotorMutex ); // Now free or "Give" the Serial Port for others.
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

}

void TaskWriteMotors( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  /*
  This task reads inputs from the console and writes the inputs to the Motor Value
  */
  const int MotorPin = 3; // Pin 3 supports PWM
  const int Input1Pin = 7;
  const int Input2Pin = 8;

  pinMode(MotorPin, OUTPUT);
  pinMode(Input1Pin, OUTPUT);
  pinMode(Input2Pin, OUTPUT);


  for (;;){ 
    if ( xSemaphoreTake( MotorMutex, ( TickType_t ) 5 ) == pdTRUE ){

      analogWrite(MotorPin,MotorValue); //Write the motor speed

      if(MotorDirection == Forwards){ //Write the motor direction
        digitalWrite(Input2Pin,LOW);
        digitalWrite(Input1Pin,HIGH);
      }
      else{
        digitalWrite(Input1Pin,LOW);
        digitalWrite(Input2Pin,HIGH);
      }
      
      xSemaphoreGive( MotorMutex ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
   
}

void TaskConsoleWrite( void *pvParameters __attribute__((unused)) )  // This is a Task.
{ 
  for (;;){ 
    if (xSemaphoreTake(MotorMutex, 5) == pdTRUE)
    {
      Serial.print("MotorValue: ");
      Serial.print((MotorDirection == Forwards) ? '+' : '-');
      Serial.print(MotorValue);
      Serial.println("  | Enter value (-255 to 255):");

      xSemaphoreGive(MotorMutex);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  
}