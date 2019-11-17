#include <SoftwareSerial.h>

#include <math.h>

#include <Time.h>
#include <TimeLib.h>

#include <SparkFunLSM6DS3.h>
#include <SPI.h>


LSM6DS3 myIMU(SPI_MODE, 10);

int ledPin = A5;

int fsrPinUR = A6;     // the FSR and 10K pulldown are connected to A6
int fsrPinUL = A7;
int fsrPinLL = A8;
int fsrPinLR = A9;

int tempPin = A4;

float fsrReadingUR=0;     // the analog reading from the FSR resistor divider
float fsrReadingUL=0;    
float fsrReadingLR=0;    
float fsrReadingLL=0;    
//int timealarm = 10000000; //desired time for alarm in microseconds

long rawTemp=0;
float voltage=0;
float celsius=0;

float AX=0;
float AY=0;
float AZ=0;

float X = 0;
float Y = 0;
float Z = 0;

float roll = 0;
float pitch = 0;
float rollF = 0;
float pitchF = 0;

const int numReadings = 100;
int readings[numReadings];
int readIndex = 0;
int total = 0;
float avg_rot = 0;

time_t t_now =0;
int first_flag = 0;


void setup() {
//  setSyncProvider(getTeensy3Time);

  Serial.begin(115200);
  while (!Serial);  // Wait for Arduino Serial Monitor to open
  delay(100);
  if (timeStatus()!= timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time"); 
  }

  pinMode(ledPin, OUTPUT);//initialize digital pin as OUTPUT
  myIMU.begin();
  pinMode(A2, OUTPUT);
  digitalWrite(A2, HIGH);

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}


 void loop() {
  if (Serial.available()) {
    time_t t = processSyncMessage();
    if (t != 0) {
      Teensy3Clock.set(t); // set the RTC
      setTime(t);
    }
  }
  digitalClockDisplay();
//
//  long timex = micros();
//
//  if (timex > 200000000){
//        return;
//  }

    fsrReadingUR = analogRead(fsrPinUR);
    fsrReadingUL = analogRead(fsrPinUL);  
    fsrReadingLL = analogRead(fsrPinLL);  
    fsrReadingLR = analogRead(fsrPinLR);    
    rawTemp = analogRead(tempPin);
    
    voltage = rawTemp * (3.3 / 1023.0);
    celsius = (voltage - 0.5) * 100;

    Serial.print("Celsius: "); // Print celcius temp to serial monitor
    Serial.println(celsius);
           
    Serial.print("Analog reading = ");
    Serial.print(fsrReadingUR);     // the raw analog reading

  Serial.println(fsrReadingUR);
  pressureDetector(fsrReadingUR,1);
  Serial.println(fsrReadingUL);
  pressureDetector(fsrReadingUL,2);
  pressureDetector(fsrReadingLL,3);
  pressureDetector(fsrReadingLR,4);
// FSR READ
    if (fsrReadingUR < 10) {
      Serial.println(" - UR No pressure");
      first_flag = 0;
      digitalWrite(ledPin, LOW);
      } 
    else if (fsrReadingUR > 10) {
      Serial.println(" - UR Pressure");
      if (first_flag == 0){
        first_flag = 1;
        t_now = now();
      }
      if (now() >= t_now+10){
       Serial.println("Turn patient!");
        digitalWrite(ledPin, HIGH);
        
      }
    }
   

    AX = myIMU.readFloatAccelX();
    AY = myIMU.readFloatAccelY();
    AZ = myIMU.readFloatAccelZ();

    X = AX - 0.01;
    Y = AY - 0.06;
    Z = AZ; 
  
//    Serial.print("Rotation: ");
//    Serial.println(RZ);
//    Serial.println(X);
//    Serial.println(Y);
//    Serial.println(Z);


   roll = atan( Y / sqrt(pow( X, 2) + pow( Z, 2))) * 180 / PI;
   pitch = atan(-1 * X / sqrt(pow( Y, 2) + pow( Z, 2))) * 180 / PI;

   rollF = 0.94 * rollF + 0.06 * roll;
   pitchF = 0.94 * pitchF + 0.06 * pitch;

  total = total - readings[readIndex]; // read from the sensor:
  readings[readIndex] = rollF; // add the reading to the total:
  total = total + readings[readIndex]; // advance to the next position in the array:
  readIndex = readIndex + 1; // if we're at the end of the array...
  if (readIndex >= numReadings) { // ...wrap around to the beginning:
    readIndex = 0;
  }
  
   avg_rot = total / numReadings; // calculate the average:
   Serial.println(avg_rot); // send it to the computer as ASCII digits





//    Serial.print("Roll: ");
//    Serial.println(rollF);

//    Serial.print("Pitch: ");
//    Serial.println(pitch);
//
//    
   delay(1000);

    //long endtime = micros();
    //long dur = 20000 - (endtime - timex);

    //if (dur < 0){
      //delayMicroseconds(20000);
    //}
    //else{
//      delayMicroseconds(dur);
//    }

    
 
    // We'll have a few threshholds, qualitatively determined 
  }

void pressureDetector(float fsrReading, int sensorNum){
//      String sensorName = " ";
      if(sensorNum == 1){
//        sensorName = "UR";
        Serial.print("UR ");
      }
      else if(sensorNum == 2){
//        sensorName = "UL";
        Serial.print("UL ");
      }
      else if(sensorNum == 3){
//        sensorName = "LL";
        Serial.print("LL ");
      }
      else if(sensorNum == 4){
//        sensorName = "LR";
        Serial.print("LR ");
      }
      else{
//        sensorName = "Unknown Sensor!";
        Serial.print("Unknown Sensor! ");
      }
      
      if (fsrReading < 10) {
      Serial.println(" - No pressure");
      first_flag = 0;
      digitalWrite(ledPin, LOW);
    } 
    else if (fsrReading > 10) {
      Serial.println(" - Pressure");
      if (first_flag == 0){
        first_flag = 1;
        t_now = now();
      }
      if (now() >= t_now+10){
       Serial.println("Turn patient!");
        digitalWrite(ledPin, HIGH);
        
      }
    }
}

void digitalClockDisplay() {
//  digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print("/");
  Serial.print(month());
  Serial.print("/");
  Serial.print(year()); 
  Serial.println(); 
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

/*  code to process time sync messages from the serial port   */
#define TIME_HEADER  "T"   // Header tag for serial time sync message

  unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     return pctime;
     if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
  }
  return pctime;
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
