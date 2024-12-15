#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  Serial.begin(9600);
  mlx.begin();
}

void loop() {
 Serial.print("Ambient= ");
 Serial.print(mlx.readAmbientTempF());
 Serial.println(" F");
 
 Serial.print("Target  ");
 Serial.print(mlx.readObjectTempF());
 Serial.println(" F ");

 delay(1000);
}
