float zeroAmpsVal = 0.0;
float maxAmps = 0.0;
float maxAmpsVal=0.0;
float measurementcoefficient = 0.0;

float analogTimeAverageA0();
float readFloatFromSerial();
/*
This program does linear fitting for the current sensing circuit. It has a precision of approximately 1mA, but this may be bettered with better calibrated power supplies.
The coefficient
*/

void setup() {
  Serial.begin(115200);
  Serial.println("Set the current to 0A, send Y when complete:");
  while (Serial.read() != 'Y'){}

  zeroAmpsVal = analogTimeAverageA0();

  Serial.println("Set the current to the maximum current of the system, then enter the value of the current as a float:");

  maxAmps = readFloatFromSerial();
  maxAmpsVal = analogTimeAverageA0();

  measurementcoefficient = (maxAmpsVal-zeroAmpsVal) / maxAmps;
  Serial.print("Amps offset (AnalogRead value) = ");
  Serial.println(zeroAmpsVal);
  Serial.print("AnalogRead/Amps = ");
  Serial.println(measurementcoefficient);
  Serial.println("Ready for live measurements? (Y,N)");

  // --- Y/N PARSE HERE ---
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();

      if (c == 'Y' || c == 'y') {
        break;                 // proceed to loop()
      }

      if (c == 'N' || c == 'n') {
        Serial.println("Waiting for Y...");
      }
    }
  }

}

void loop() {

  float measurement = analogRead(A0);
  float calibratedMeasurement = (measurement - zeroAmpsVal)/measurementcoefficient;

  Serial.print("Measured current (A) = ");
  Serial.println(calibratedMeasurement,4);

}
float analogTimeAverageA0(){
  int totalMeasurements = 200;
  long total = 0.0;
  for(int i =0; i<totalMeasurements; i = i+1){
    total = total + analogRead(A0);
  }
  return total/totalMeasurements;
}


float readFloatFromSerial() {
  String input = "";

  while (true) {
    if (Serial.available()) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        if (input.length() > 0) {
          return input.toFloat();   // <-- conversion happens here
        }
      } else {
        input += c;
      }
    }
  }
}