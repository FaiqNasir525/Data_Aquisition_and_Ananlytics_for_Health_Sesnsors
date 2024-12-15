  /*
  TTP223B-Capacitive-Touch-Switch-Module
  made on 08 Nov 2020
  by Amir Mohammad Shojaee @ Electropeak
  Home

*/
const int Touch_Sensor = 2; 

         
void setup() {
  Serial.begin(9600);
  // initialize the Arduino's pin as aninput
  pinMode(Touch_Sensor, INPUT);
}

void loop() {
  while(1){
  if (digitalRead(Touch_Sensor) == HIGH);
  break;
  }
}
