int Usensor = 11;
unsigned long start_time = 0;
unsigned long end_time = 0;
int steps=0;
float steps_old=0;
float temp=0;
float rps=0;

void setup() {
  Serial.begin(9600);
  pinMode(Usensor,INPUT_PULLUP);
  Serial.println(" STEPS - 0");
  Serial.println(" RPS   - 0.00");
}

void loop() {
  start_time = millis();
  end_time = start_time+10000;
  while(millis()<end_time)
  {
    if(digitalRead(Usensor))
    {
      steps=steps+1; 
      while(digitalRead(Usensor));
      Serial.print("Steps= ");
      Serial.println(steps);
    } 
 }
    temp=steps-steps_old;
    steps_old=steps;
    rps=(temp);
    Serial.print("RPS= ");
    Serial.println(rps);
}
