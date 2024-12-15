#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <SPI.h>
#include <SD.h>

//SD Card Variables
File myFile;
const int chipSelect = 10;

//Voltage Sensor Variables
float valVoltage = 0;
int rawVoltage = 0;
int VoltagePin=A0;

//Current Sensor Variables
int CurrentPin=A1;

//U-Sensor Variables
int USensorPin = 4;
int USensorPower = 3;
unsigned long start_time = 0;
unsigned long end_time = 0;
int steps=0;
float temp=0;
float rpm=0;

//Touch Sensor Variable
const int TouchSensorPin = 2; 

//Water Level Variable
int LevelPin=A2;

//Temperature Sensor Variables
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
int TempSensorPower=5;

//HeartRate Variables
int pulsePin = A3;                 // Pulse Sensor purple wire connected to analog pin A0
int blinkPin = 7;                // pin to blink led at each beat
int HeartRatePower=6;
// Volatile Variables, used in the interrupt service routine!
volatile int BPM;                   // int that holds raw Analog in 0. updated every 2mS
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded! 
volatile boolean Pulse = false;     // "True" when User's live heartbeat is detected. "False" when not a "live beat". 
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.
static boolean serialVisual = true;   // Set to 'false' by Default.  Re-set to 'true' to see Arduino Serial Monitor ASCII Visual Pulse 
volatile int rate[10];                      // array to hold last ten IBI values
volatile unsigned long sampleCounter = 0;          // used to determine pulse timing
volatile unsigned long lastBeatTime = 0;           // used to find IBI
volatile int P = 512;                      // used to find peak in pulse wave, seeded
volatile int T = 512;                     // used to find trough in pulse wave, seeded
volatile int thresh = 525;                // used to find instant moment of heart beat, seeded
volatile int amp = 100;                   // used to hold amplitude of pulse waveform, seeded
volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat = false;      // used to seed rate array so we startup with reasonable BPM

//Oximetre Variables
/*MAX30105 particleSensor;
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#endif
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read
byte OximetrePower=6;*/

void setup() {
  Serial.begin(115200);
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("Data.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.println("Writing to Data.txt...");
    myFile.println("***********Data Aqqusition Started*********");
  }
  pinMode(USensorPower,OUTPUT);
  pinMode(USensorPin,INPUT_PULLUP);
  pinMode(TouchSensorPin, INPUT);
  pinMode(TempSensorPower, OUTPUT);
  pinMode(HeartRatePower, OUTPUT);
  pinMode(blinkPin,OUTPUT);
  //pinMode(pulseLED, OUTPUT);
  //pinMode(readLED, OUTPUT);
  //pinMode(OximetrePower, OUTPUT);
}

void loop() {
  VoltageSensor();
  CurrentSensor();
  WaterLevel();
  Serial.println("Press Touch Sensor to Start U Encoder");
  TouchSensor();
  USensor();
  Serial.println("Place the target in front of Temperature Sensor and Press Touch Sensor to Start Temperature Sensor");
  TouchSensor();
  TemperatureSensor();
  Serial.println("Place the finger on HeartRate Sensor and Press Touch Sensor to Start HeartRate Sensor");
  TouchSensor();
  HeartRate();
  /*Serial.println("Place the finger on Oximetre Sensor and Press Touch Sensor to Start Oximetre Sensor");
  TouchSensor();
  Oximetre();*/
}

void VoltageSensor(){
  rawVoltage=analogRead(VoltagePin);
  valVoltage = mapfloat(rawVoltage, 0, 1023, 0, 12);
  Serial.print("Supply Voltage= ");
  Serial.print(valVoltage);
  Serial.println(" V");
  myFile.print("Supply Voltage= ");
  myFile.print(valVoltage);
  myFile.println(" V");
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void CurrentSensor(){
 float averageCurrent = 0;
 for(int i = 0; i < 1000; i++) {
   //averageCurrent = averageCurrent + (.0264 * analogRead(CurrentPin) -13.51);//for the 5A mode,  
   averageCurrent = averageCurrent + (.049 * analogRead(CurrentPin) -25);// for 20A mode
   delay(1);
 }
 Serial.print("Current :");
 Serial.print(averageCurrent/1000);
 Serial.print("A or");
 Serial.print(averageCurrent);
 Serial.println("mA");
 myFile.print("Current :");
 myFile.print(averageCurrent/1000);
 myFile.print("A or");
 myFile.print(averageCurrent);
 myFile.println("mA");
}

void USensor(){
  digitalWrite(USensorPower, HIGH);
  steps=0;
  start_time = millis();
  end_time = start_time+10000;
  while(millis()<end_time)
  {
    if(digitalRead(USensorPin))
    {
      steps=steps+1;
    } 
  }
  temp=steps*6;
  rpm=(temp/2);
  Serial.print("RPM= ");
  Serial.println(rpm);
  myFile.print("RPM= ");
  myFile.println(rpm);
  digitalWrite(USensorPower,LOW);
}

void TouchSensor(){
  while(1){
  if (digitalRead(TouchSensorPin) == HIGH);
  break;
  }
}

void WaterLevel(){
  int Level=analogRead(LevelPin);
  Serial.print("Water Level= ");
  Serial.println(Level);
  myFile.print("Water Level= ");
  myFile.println(Level);
}

void TemperatureSensor(){
 digitalWrite(TempSensorPower, HIGH);
 mlx.begin();
 Serial.print("Ambient= ");
 Serial.print(mlx.readAmbientTempF());
 Serial.println(" F");
 Serial.print("Target  ");
 Serial.print(mlx.readObjectTempF());
 Serial.println(" F ");
 myFile.print("Ambient= ");
 myFile.print(mlx.readAmbientTempF());
 myFile.println(" F");
 myFile.print("Target  ");
 myFile.print(mlx.readObjectTempF());
 myFile.println(" F ");
 digitalWrite(TempSensorPower, LOW);
}

/*void Oximetre(){
  digitalWrite(OximetrePower, HIGH);
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)){ //Use default I2C port, 400kHz speed
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }
  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  while (Serial.available() == 0) ; //wait until user presses a key
  Serial.read();
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 25; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
  digitalWrite(OximetrePower, LOW);
}*/

void HeartRate(){
  digitalWrite(HeartRatePower, HIGH);
  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS 
  serialOutput();    
  if (QS == true) // A Heartbeat Was Found
    {     
      // BPM and IBI have been Determined
      // Quantified Self "QS" true when arduino finds a heartbeat
      serialOutputWhenBeatHappens(); // A Beat Happened, Output that to serial.     
      QS = false; // reset the Quantified Self flag for next time    
    }  
  delay(20); //  take a break
  digitalWrite(HeartRatePower, HIGH);
}
void interruptSetup(){     
  // Initializes Timer2 to throw an interrupt every 2mS.
  TCCR2A = 0x02;     // DISABLE PWM ON DIGITAL PINS 3 AND 11, AND GO INTO CTC MODE
  TCCR2B = 0x06;     // DON'T FORCE COMPARE, 256 PRESCALER 
  OCR2A = 0X7C;      // SET THE TOP OF THE COUNT TO 124 FOR 500Hz SAMPLE RATE
  TIMSK2 = 0x02;     // ENABLE INTERRUPT ON MATCH BETWEEN TIMER2 AND OCR2A
  sei();             // MAKE SURE GLOBAL INTERRUPTS ARE ENABLED      
} 
void serialOutput(){   // Decide How To Output Serial. 
 if (serialVisual == true)
  {  
     arduinoSerialMonitorVisual('-', Signal);   // goes to function that makes Serial Monitor Visualizer
  } 
 else
  {
      sendDataToSerial('S', Signal);     // goes to sendDataToSerial function
   }        
}
void serialOutputWhenBeatHappens(){    
 if (serialVisual == true) //  Code to Make the Serial Monitor Visualizer Work
   {            
     Serial.print(" Heart-Beat Found ");  //ASCII Art Madness
     Serial.print("BPM: ");
     Serial.println(BPM);
     myFile.print(" Heart-Beat Found ");  //ASCII Art Madness
     myFile.print("BPM: ");
     myFile.println(BPM);
   }
 else
   {
     sendDataToSerial('B',BPM);   // send heart rate with a 'B' prefix
     sendDataToSerial('Q',IBI);   // send time between beats with a 'Q' prefix
   }   
}
void arduinoSerialMonitorVisual(char symbol, int data ){    
  const int sensorMin = 0;      // sensor minimum, discovered through experiment
  const int sensorMax = 1024;    // sensor maximum, discovered through experiment
  int sensorReading = data; // map the sensor range to a range of 12 options:
  int range = map(sensorReading, sensorMin, sensorMax, 0, 11);
  // do something different depending on the 
  // range value:
}
void sendDataToSerial(char symbol, int data ){
   Serial.print(symbol);
   Serial.println(data);                
}
ISR(TIMER2_COMPA_vect){ //triggered when Timer2 counts to 124  
  cli();                                      // disable interrupts while we do this
  Signal = analogRead(pulsePin);              // read the Pulse Sensor 
  sampleCounter += 2;                         // keep track of the time in mS with this variable
  int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise
                                              //  find the peak and trough of the pulse wave
  if(Signal < thresh && N > (IBI/5)*3) // avoid dichrotic noise by waiting 3/5 of last IBI
    {      
      if (Signal < T) // T is the trough
      {                        
        T = Signal; // keep track of lowest point in pulse wave 
      }
    }
  if(Signal > thresh && Signal > P)
    {          // thresh condition helps avoid noise
      P = Signal;                             // P is the peak
    }                                        // keep track of highest point in pulse wave
  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
  if (N > 250)
  {                                   // avoid high frequency noise
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) )
      {        
        Pulse = true;                               // set the Pulse flag when we think there is a pulse
        digitalWrite(blinkPin,HIGH);                // turn on pin 13 LED
        IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
        lastBeatTime = sampleCounter;               // keep track of time for next pulse
        if(secondBeat)
        {                        // if this is the second beat, if secondBeat == TRUE
          secondBeat = false;                  // clear secondBeat flag
          for(int i=0; i<=9; i++) // seed the running total to get a realisitic BPM at startup
          {             
            rate[i] = IBI;                      
          }
        }
        if(firstBeat) // if it's the first time we found a beat, if firstBeat == TRUE
        {                         
          firstBeat = false;                   // clear firstBeat flag
          secondBeat = true;                   // set the second beat flag
          sei();                               // enable interrupts again
          return;                              // IBI value is unreliable so discard it
        }   
      // keep a running total of the last 10 IBI values
      word runningTotal = 0;                  // clear the runningTotal variable    
      for(int i=0; i<=8; i++)
        {                // shift data in the rate array
          rate[i] = rate[i+1];                  // and drop the oldest IBI value 
          runningTotal += rate[i];              // add up the 9 oldest IBI values
        }
      rate[9] = IBI;                          // add the latest IBI to the rate array
      runningTotal += rate[9];                // add the latest IBI to runningTotal
      runningTotal /= 10;                     // average the last 10 IBI values 
      BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
      QS = true;                              // set Quantified Self flag 
      // QS FLAG IS NOT CLEARED INSIDE THIS ISR
    }                       
  }
  if (Signal < thresh && Pulse == true)
    {   // when the values are going down, the beat is over
      digitalWrite(blinkPin,LOW);            // turn off pin 13 LED
      Pulse = false;                         // reset the Pulse flag so we can do it again
      amp = P - T;                           // get amplitude of the pulse wave
      thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
      P = thresh;                            // reset these for next time
      T = thresh;
    }
  if (N > 2500)
    {                           // if 2.5 seconds go by without a beat
      thresh = 512;                          // set thresh default
      P = 512;                               // set P default
      T = 512;                               // set T default
      lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
      firstBeat = true;                      // set these to avoid noise
      secondBeat = false;                    // when we get the heartbeat back
    }
  sei();                                   // enable interrupts when youre done!
}
