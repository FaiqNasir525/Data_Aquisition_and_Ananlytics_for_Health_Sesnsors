void setup() {
  Serial.begin(9600);
}
 
void loop() {
 float averageCurrent = 0;
 for(int i = 0; i < 1000; i++) {
   //averageCurrent = averageCurrent + (.0264 * analogRead(A1) -13.51);//for the 5A mode,  
   averageCurrent = averageCurrent + (.049 * analogRead(A1) -25);// for 20A mode
   delay(1);
 }
 Serial.print("Current :");
 Serial.print(averageCurrent/1000);
 Serial.print("A or");
 Serial.print(averageCurrent);
 Serial.println("mA");
}
