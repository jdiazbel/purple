#include <math.h>
#include <Time.h>
#include <TimeLib.h>
#include <SparkFunLSM6DS3.h>
#include <SPI.h>

LSM6DS3 myIMU(SPI_MODE, 10);

//Define data pins
//int tempPinPwr = A4;
int fsrPinUR = A6;     // the FSR and 10K pulldown are connected to A6
int fsrPinUL = A7;
int fsrPinLL = A8;
int fsrPinLR = A9;
int tempPin = A5; //right shoulder sensor
int tempPin2 = A1;
//int ledPin = A2;

//Define variables
float fsrReadingUR=0;     // the analog reading from the FSR resistor divider
float fsrReadingUL=0;    
float fsrReadingLR=0;    
float fsrReadingLL=0;    

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

//Define Timers
time_t t_now = 0;
time_t URt_now =0;
time_t ULt_now =0;
time_t LRt_now =0;
time_t LLt_now =0;
time_t temp_now =0;
time_t URt_delay =0;
time_t ULt_delay =0;
time_t LRt_delay =0;
time_t LLt_delay =0;

//Define flags
int first_flag = 0;
int URfirst_flag = 0;
int ULfirst_flag = 0;
int LLfirst_flag = 0;
int LRfirst_flag = 0;
int tempFlag = 0;
int URdelay_flag = 0;
int ULdelay_flag = 0;
int LLdelay_flag = 0;
int LRdelay_flag = 0;
int init_flag = 0;
int URstart_delay = 0;
int ULstart_delay = 0;
int LLstart_delay = 0;
int LRstart_delay = 0;

int URturn = 0;
int ULturn = 0;
int LLturn = 0;
int LRturn = 0;

void setup() {

  setSyncProvider(getTeensy3Time);

  Serial1.begin(38400);
  
  Serial.begin(9600);
  while (!Serial);  // Wait for Arduino Serial Monitor to open
  delay(100);

  if (timeStatus()!= timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time"); 
  }

//  pinMode(tempPinPwr, OUTPUT);//initialize digital pin as OUTPUT
   
  myIMU.begin();
  pinMode(A2, OUTPUT);
  digitalWrite(A2, HIGH);
  
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {

    readings[thisReading] = 0;

  }

}

 void loop() {
//initialize realtime clock
  if (Serial.available()) {
    time_t t = processSyncMessage();
    if (t != 0) {
      Teensy3Clock.set(t); // set the RTC
      setTime(t);
    }
  }
    if(init_flag == 0){
      URt_now = now();
      ULt_now = now();
      LLt_now = now();
      LRt_now = now();
      URt_delay = now();
      ULt_delay = now();
      LLt_delay = now();
      LRt_delay = now();
      init_flag = 1;
    }
 //read FSR data
    fsrReadingUR = analogRead(fsrPinUR);
    fsrReadingUL = analogRead(fsrPinUL);  
    fsrReadingLL = analogRead(fsrPinLL);
    fsrReadingLR = analogRead(fsrPinLR);    

//display sensor values every second
  if(tempFlag == 0){
    tempFlag = 1;
    temp_now = now()+1;
  }

  if(temp_now == now()){
    digitalClockDisplay();
  
    pressureDetector(fsrReadingUR,1);
    pressureDetector(fsrReadingUL,2);
    pressureDetector(fsrReadingLL,3);
    pressureDetector(fsrReadingLR,4);
    patientOrientation();
    readTemp(1);
    readTemp(2); //readtemp2
    Serial.println();
    turnPatient();
    Serial.println();
    tempFlag = 0;
  }

//calculate orientation
    AX = myIMU.readFloatAccelX();
    AY = myIMU.readFloatAccelY();
    AZ = myIMU.readFloatAccelZ();

    X = AX - 0.01;
    Y = AY - 0.06;
    Z = AZ; 

   roll = atan( Y / sqrt(pow( X, 2) + pow( Z, 2))) * 180 / PI;
   pitch = atan(-1 * X / sqrt(pow( Y, 2) + pow( Z, 2))) * 180 / PI;
   rollF = 0.94 * rollF + 0.06 * roll;
   pitchF = 0.94 * pitchF + 0.06 * pitch;

  total = total - readings[readIndex]; // read from the sensor:
  readings[readIndex] = rollF; // add the reading to the total:
  total = total + readings[readIndex]; // advance to the next position in the array:
  readIndex = readIndex + 1; // if we're at the end of the array...

  if(readIndex >= numReadings) { // ...wrap around to the beginning:
    readIndex = 0;
  }

   avg_rot = total / numReadings; // calculate the average:
 }


//Function to detail if enough pressure is being detected from a FSR sensor
void pressureDetector(float fsrReading, int sensorNum){
      int timerSensor = 0;
      int flaggy = -1;
      
      if(sensorNum == 1){
        Serial.print("UR ");
        timerSensor = 1;
      }
      else if(sensorNum == 2){
        Serial.print("UL ");
        timerSensor = 2;
      }
      else if(sensorNum == 3){
        Serial.print("LL ");
        timerSensor = 3;
      }
      else if(sensorNum == 4){
        Serial.print("LR ");
        timerSensor = 4;
      }
      else{
        Serial.print("Unknown Sensor! ");
      }

    if (fsrReading < 100) {
      Serial.println(" - No pressure");
      Serial1.print("No Pressure");
      Serial1.print(",");   
      flaggy = 0;
    } 
    else if (fsrReading > 100) {
      Serial.println(" - Pressure");
      Serial1.print("Pressure");
      Serial1.print(",");
      flaggy = 1;
      if((URdelay_flag == 0)){
        URt_delay = now();
      }
      if((ULdelay_flag == 0)){
        ULt_delay = now();
      }
      if((LLdelay_flag == 0)){
        LLt_delay = now();
      }
      if((LRdelay_flag == 0)){
        LRt_delay = now();
      }
    }
    timerSet(flaggy,timerSensor);
}

//function to determine if timer should be started or reset for each FSR sensor
//timers are independant for each sensor
void timerSet(int pressureflag, int sensorNum){
  if(sensorNum != 0 && pressureflag == 1){
    if(sensorNum == 1){
      if(URfirst_flag == 0){
        URfirst_flag = 1;
        URt_now = now();    
        URstart_delay = 1;
      }
      else{
        URt_delay = now();
      }
    }
    if(sensorNum == 2){
      if(ULfirst_flag == 0){
        ULt_now = now();
        ULfirst_flag = 1;
        ULstart_delay = 1;
      }
      else{
        ULt_delay = now();
      }
    }
    if(sensorNum == 3){
      if(LLfirst_flag == 0){
        LLt_now = now();
        LLfirst_flag = 1;
        LLstart_delay = 1;
      }
      else{
        LLt_delay = now();
      }
    }
    if(sensorNum == 4){
      if(LRfirst_flag == 0){
        LRt_now = now();
        LRfirst_flag = 1;
        LRstart_delay = 1;
      }
      else{
        LRt_delay = now();
      }
    }
  }
  else if(sensorNum != 0 && pressureflag == 0){
    if((sensorNum == 1) and (now() >= URt_delay+5)){//no pressure and the delay is satisfied
      URt_now = now();
      URfirst_flag = 0; 
      URdelay_flag = 0;//do i need this?
      URstart_delay = 0;
      URt_delay = now();
    }
    else if(sensorNum == 1 and URstart_delay == 1){//there was pressure at a point
      //hold delay
      URdelay_flag = 1;
    }
    else if(sensorNum == 1){//no pressure ever
        URt_now = now();
        URfirst_flag = 0; 
    }
    
    if((sensorNum == 2) and (now() >= ULt_delay+5)){
      ULt_now = now();
      ULfirst_flag = 0; 
      ULdelay_flag = 0;
      ULstart_delay = 0;
      ULt_delay = now();
    }
    else if(sensorNum == 2 and ULstart_delay == 1){//there was pressure at a point
      //hold delay
      ULdelay_flag = 1;
    }
    else if(sensorNum == 2){
      ULt_now = now();
      ULfirst_flag = 0; 
    }
    
    if((sensorNum == 3) and (now() >= LLt_delay+5)){
      LLt_now = now();
      LLfirst_flag = 0; 
      LLdelay_flag = 0;
      LLstart_delay = 0;
      LLt_delay = now();
    }
    else if(sensorNum == 3 and LLstart_delay == 1){//there was pressure at a point
      //hold
      LLdelay_flag = 1;
    }
    else if(sensorNum == 3){
      LLt_now = now();
      LLfirst_flag = 0; 
    }
    
    if((sensorNum == 4) and (now() >= LRt_delay+5)){
      LRt_now = now();
      LRfirst_flag = 0;
      LRdelay_flag = 0;
      LRstart_delay = 0;
      LRt_delay = now();
    }
    else if(sensorNum == 4 and LRstart_delay == 1){//there was pressure at a point
      //Do NOTHING
      LRdelay_flag = 1;
    }
    else if(sensorNum == 4){
      LRt_now = now();
      LRfirst_flag = 0; 
    }
  }
}

//Function that sends alerts to relieve pressure based on how long it has been applied
void turnPatient(){
  String alert = " ";
  if (now() >= URt_now+10){
    Serial.println("Turn patient to relieve UR Pressure!");
    alert = alert + "Relieve Right Shoulder\n";
  }
  if(now() >= ULt_now+10){
    Serial.println("Turn patient to relieve UL Pressure!");
    alert = alert + "Relieve Left Shoulder\n";
  }
  if(now() >= LLt_now+10){
    Serial.println("Turn patient to relieve LL Pressure!");
    alert = alert + "Relieve Left Hip\n";
  }
  if(now() >= LRt_now+10){
    Serial.println("Turn patient to relieve LR Pressure!");
    alert = alert + "Relieve Right Hip\n";
  }
  alert = alert + ",";
  Serial1.print(alert);
}

//Function that outputs how the patient is orientated based on the angle calculated
void patientOrientation(){
  if(avg_rot <=-30){
    Serial.println("Patient is Laying on their Left Side.");
    Serial1.print("Left Side");
    Serial1.print(",");
  }
  else if(avg_rot >= 30){
    Serial.println("Patient is Laying on their Right Side.");
    Serial1.print("Right Side");
    Serial1.print(",");
  }
  else if(avg_rot < 30 && avg_rot >= -30){
    Serial.println("Patient is Laying on their Back.");
    Serial1.print("Supine");
    Serial1.print(",");
  }
  else{
    Serial.println("Patient Orientation is unclear.");
    Serial1.print("Patient Orientation is unclear.");
    Serial1.print(",");
  }
   Serial.print("Patient Orientation Angle: ");
   Serial.print(avg_rot);
   Serial.println("°");
}

//Function to read and output temperature data based on sensor reading
void readTemp(int sensor){
//  digitalWrite(tempPinPwr, HIGH);
  if(sensor == 1){
    rawTemp = analogRead(tempPin);
  }
  else if(sensor ==2){
    rawTemp = analogRead(tempPin2);
  }
  
  voltage = rawTemp * (3.3 / 1023.0);
  celsius = (voltage - 0.5) * 100;

  Serial.print("External Body Temperature: "); // Print celcius temp to serial monitor
  Serial.print(celsius); 
  Serial.println("°C");
  Serial1.print((String)celsius + "°C");
  Serial1.print(",");
  
//  digitalWrite(tempPinPwr, LOW);
}

//Function to display RTC
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

time_t getTeensy3Time(){
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
