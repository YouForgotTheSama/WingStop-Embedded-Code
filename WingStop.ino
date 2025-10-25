
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Servo.h>
#include <string.h>
#include <SD.h>

#define xbee_TX 0
#define xbee_RX 1
#define allSensorsSDA 4
#define allSensorsSCL 5
#define datalogger_TX 8
#define datalogger_RX 9
#define Buzzer 10 
#define ServoPin 16
#define LED 17
#define voltagemeasure 26

Adafruit_BMP3XX BMP;
SFE_UBLOX_GNSS GPS;
Adafruit_BNO055 BNO;
Servo Servo1;
File dataFile;

int TEAMID = 5; 
int PACKET_COUNT = 0;
char SW_STATE[50] = "LAUNCH-READY";
char PL_STATE = 'N';
float ALTITUDE = 0;
float TEMP = 0;
float VOLTAGE = 0;
float LAT = 0;
float LONG = 0;
float GYRO_R = 0;
float GYRO_P = 0;
float GYRO_Y = 0;
float PRESSURE = 0;
float VELOCITY = 0;
float SEALEVELPRESSURE_HPA = 1013.25;
char command[67];

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
sensors_event_t orientationData;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup (){
  Serial.begin (9600);
  Serial.println("(setup start)");
  Wire.setSDA(allSensorsSDA);
  Wire.setSCL(allSensorsSCL);
  Wire.begin();

  if (!BNO.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  Serial.println("(BNO start)");


  if (!BMP.begin_I2C()){
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  }
  BMP.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  BMP.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  BMP.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  BMP.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println("(BMP start)");
  //BMP calibration zone
  BMP.performReading();
  BMP.performReading();
  BMP.performReading();
  BMP.performReading();
  BMP.performReading();
  float ref1 = BMP.pressure / 100.0;
  BMP.performReading();
  float ref2 = BMP.pressure / 100.0;
  BMP.performReading();
  float ref3 = BMP.pressure / 100.0;
  BMP.performReading();
  float ref4 = BMP.pressure / 100.0;
  BMP.performReading();
  float ref5 = BMP.pressure / 100.0;
  BMP.performReading();
  float reference_press = (ref1 + ref2 + ref3 + ref4 + ref5)/5;
  SEALEVELPRESSURE_HPA = reference_press;


  if (GPS.begin() == false) {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
  }
  GPS.setI2COutput(COM_TYPE_UBX);
  GPS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  Serial.println("(GPS start)");

  //XBee is Serial 1
  Serial1.setTX (xbee_TX);
  Serial1.setRX (xbee_RX);
  Serial1.begin (9600);
  Serial.println("Xbee start");

  //DataLogger is Serial 2 
  Serial2.setTX (datalogger_TX);
  Serial2.setRX (datalogger_RX);
  Serial2.begin (9600);
  Serial.println("(datalogger start)");

  //DataLogger is Serial 2 
  if (SD.begin(datalogger_RX)){ 
    dataFile = SD.open("data.csv , FILE_WRITE");
  }

  Servo1.attach(ServoPin);

  pinMode(Buzzer, OUTPUT);
  digitalWrite(Buzzer, LOW);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

}

void takeCommand(){
  if (Serial.available() > 0){
    int i = 0;
    while (Serial.available()){
      command[i] = Serial.read();
      i++;
    }

    if (strcmp(command, "lemon pepper") == 0){
      SW_STATE == "SEPARATE";
      Servo1.write(180);
    }
    else if (strcmp(command, "hot honey") == 0){
      ALTITUDE = BMP.readAltitude (SEALEVELPRESSURE_HPA);
    }
  }
}

void takeData(){
  ALTITUDE = BMP.readAltitude (SEALEVELPRESSURE_HPA);
  TEMP = BMP.readTemperature();
  VOLTAGE = analogRead(voltagemeasure);
  VOLTAGE = ((3.3 * VOLTAGE)/4095.0);
  VOLTAGE = VOLTAGE * ((68000 + 100000)/100000);
  //resistor values need to be given by eletrical and resistor2 will be bigger value
  LAT = GPS.getLatitude();
  LONG = GPS.getLongitude();
  BNO.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  GYRO_R = (orientationData.orientation.x);
  GYRO_P = (orientationData.orientation.y);
  GYRO_Y = (orientationData.orientation.z);
  PRESSURE = BMP.readPressure();

}

void changeState(){
  if ((strcmp(SW_STATE, "LAUNCH-READY") == 0) && ALTITUDE >= 10){
    SW_STATE == "ASCENT"; //Add Mission start here
  }
  else if ((strcmp(SW_STATE, "ASCENT") == 0) && ALTITUDE >= 490){
    SW_STATE == "SEPARATE"; 
    Servo1.write(180); 
    PL_STATE = 'R' ;
  }
  else if ((strcmp(SW_STATE, "SEPARATE") == 0) && ALTITUDE <490){
    SW_STATE == "DESCENT";
  }
  else if ((strcmp(SW_STATE, "DESCENT") == 0) && ALTITUDE <10){
    SW_STATE == "LANDED";
    digitalWrite(Buzzer, HIGH);
    digitalWrite(LED, HIGH);
  }
  //make sure you end telemetry

}

void sendData(){
  PACKET_COUNT +=1;
  unsigned long RUNTIME = millis();
  char SPRING_TIME[25];
  sprintf(SPRING_TIME, "%02ld:%02ld:%02ld.%02ld", (RUNTIME / (1000*60*60)) %100, (RUNTIME / (1000*60)) %60, (RUNTIME / 1000) %60);
  char XbeeString[500];
  sprintf(XbeeString, "%i,%s,%i,%s,%c,%.2f,%.2f,%.1f,%.4f,%.4f,%.2f,%.2f,%.2f,,%.2f,%.5f",TEAMID,SPRING_TIME,PACKET_COUNT,SW_STATE,PL_STATE,ALTITUDE,TEMP,VOLTAGE,LAT,LONG,GYRO_R,GYRO_P,GYRO_Y,PRESSURE,VELOCITY);
  Serial1.println (XbeeString);
  Serial2.println (XbeeString);
  dataFile.println (XbeeString);
  dataFile.flush ();
}

unsigned long STARTTIME = millis();
float STARTALT = ALTITUDE;

void loop(){
  takeCommand();
  takeData();
  changeState();

  unsigned long CURRENTTIME = millis();
  if ((CURRENTTIME - STARTTIME) > 1000){
    VELOCITY = (ALTITUDE - STARTALT)/((CURRENTTIME - STARTTIME)/1000);
    STARTALT = ALTITUDE;
    sendData();
    STARTTIME = CURRENTTIME;
  }
}
