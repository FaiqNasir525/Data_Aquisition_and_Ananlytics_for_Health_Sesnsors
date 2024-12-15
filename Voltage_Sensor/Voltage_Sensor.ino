void setup() {
  Serial.begin(9600);
  Serial.println( "Voltage Sensor For 12V Supply");
}
float valVoltage = 0;
int rawVoltage = 0;
void loop() {
  rawVoltage=analogRead(A0);
  valVoltage = mapfloat(rawVoltage, 0, 1023, 0, 12);
  Serial.print("Battery voltage of Motor= ");
  Serial.print(val);
  Serial.println(" V");
  delay(1000);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
