#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Adafruit_BMP3XX.h"
#include <Servo.h>

constexpr uint8_t PIN_XBEE_TX       = 0;
constexpr uint8_t PIN_XBEE_RX       = 1;
constexpr uint8_t PIN_SERVO         = 27;

constexpr uint8_t I2C_ADDR_GNSS     = 0x42;
constexpr uint8_t I2C_ADDR_BNO055   = 0x28;
constexpr uint8_t I2C_ADDR_BMP3XX_A = 0x77;
constexpr uint8_t I2C_ADDR_BMP3XX_B = 0x76;

constexpr uint8_t  TEAMID               = 5;
constexpr float    SEPARATION_ALT_M     = 490.0f;
constexpr float    APOGEE_MIN_ALT_M     = 100.0f;
constexpr uint32_t LOOP_PERIOD_MS       = 100;
constexpr uint16_t BASELINE_READINGS    = 20;
constexpr uint32_t XBEE_BAUD            = 9600;
constexpr uint8_t  SERVO_CLOSED_DEG     = 0;
constexpr uint8_t  SERVO_OPEN_DEG       = 180;
constexpr uint8_t  SERVO_REASSERT_N     = 5;
constexpr uint8_t  BNO055_SENSOR_ID     = 55;
constexpr float    V_ASCENT_THR         = 2.0f;
constexpr uint32_t GPS_PERIOD_MS        = 1000;

SFE_UBLOX_GNSS gnss;
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno(BNO055_SENSOR_ID, I2C_ADDR_BNO055, &Wire);
Servo servo;

enum FlightState { LAUNCH_READY, ASCENT, SEPARATE, DESCENT, LANDED };
const char* STATE_NAME[] = {"LAUNCH_READY","ASCENT","SEPARATE","DESCENT","LANDED"};

uint32_t lastTickMs=0, pkt=0, tPrevMs=0, t0Ms=0, lastGpsMs=0;
bool payloadReleased=false, baselineSet=false, missionStarted=false;
float pressureSum=0.0f, p0_Pa=0.0f, altRel=0.0f, lastTempC=0.0f, vVert=0.0f, altPrev=0.0f, lastPressurePa=0.0f;
int openReasserts=0;
long gpsLat_e7=0, gpsLon_e7=0;
float lastGr=0.0f, lastGp=0.0f, lastGy=0.0f;
uint16_t pressureReads=0;
uint8_t ascentConfirm=0;
FlightState curState=LAUNCH_READY;

static inline void missionTime(char* out, size_t n, uint32_t ms){
  const uint32_t s=ms/1000UL, hh=s/3600UL, mm=(s%3600UL)/60UL, ss=s%60UL, hs=(ms%1000UL)/10UL;
  snprintf(out,n,"%02lu:%02lu:%02lu.%02lu",(unsigned long)hh,(unsigned long)mm,(unsigned long)ss,(unsigned long)hs);
}

static inline void normalizeCommand(char* buf){
  char* start=buf; while(*start==' '||*start=='\t') ++start;
  char* end=start+strlen(start); while(end>start && (end[-1]==' '||end[-1]=='\t')) --end;
  size_t len=(size_t)(end-start); memmove(buf,start,len); buf[len]='\0';
  for(size_t i=0;i<len;++i) buf[i]=(char)toupper((unsigned char)buf[i]);
}

static inline void sendTelemetry(Print& out,const char* tStr,FlightState st,char payload,
                                 float alt,float temp,long lat_e7,long lon_e7,
                                 float gr,float gp,float gy,float pressPa,float v){
  const float lat=lat_e7/1e7f, lon=lon_e7/1e7f;
  char line[192];
  snprintf(line,sizeof(line),"%d,%s,%lu,%s,%c,%d,%d,%.5f,%.5f,%d,%d,%d,,%d,%d",
           TEAMID,tStr,(unsigned long)pkt,STATE_NAME[st],payload,
           (int)lroundf(alt),(int)lroundf(temp),
           (double)lat,(double)lon,
           (int)lroundf(gr),(int)lroundf(gp),(int)lroundf(gy),
           (int)lroundf(pressPa*0.001f),(int)lroundf(v));
  out.println(line);
}

static inline void initPeripherals(){
  Wire.begin(); Wire.setClock(400000);
  gnss.begin(Wire,I2C_ADDR_GNSS); gnss.setI2COutput(COM_TYPE_UBX);
  (void)(bmp.begin_I2C(I2C_ADDR_BMP3XX_A,&Wire)||bmp.begin_I2C(I2C_ADDR_BMP3XX_B,&Wire));
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_12_5_HZ);
  bno.begin(); bno.setExtCrystalUse(true); bno.setMode(OPERATION_MODE_NDOF);
  Serial1.setTX(PIN_XBEE_TX); Serial1.setRX(PIN_XBEE_RX); Serial1.begin(XBEE_BAUD);
  servo.attach(PIN_SERVO,500,2500); servo.write(SERVO_CLOSED_DEG);
}

static inline void tickGyro(){
  sensors_event_t ge;
  if(bno.getEvent(&ge,Adafruit_BNO055::VECTOR_GYROSCOPE)){
    lastGr=ge.gyro.x*RAD_TO_DEG; lastGp=ge.gyro.y*RAD_TO_DEG; lastGy=ge.gyro.z*RAD_TO_DEG;
  }
}

static inline void tickGPS(uint32_t now){
  if(now-lastGpsMs<GPS_PERIOD_MS) return;
  lastGpsMs=now;
  if(gnss.getFixType()>=2){ gpsLat_e7=gnss.getLatitude(); gpsLon_e7=gnss.getLongitude(); }
}

static inline bool tickBaro(){
  if(!bmp.performReading()) return false;
  lastTempC=bmp.temperature; lastPressurePa=bmp.pressure;
  if(!baselineSet){
    if(pressureReads<BASELINE_READINGS){ pressureSum+=lastPressurePa; ++pressureReads; }
    else { pressureSum-=pressureSum/(float)BASELINE_READINGS; pressureSum+=lastPressurePa; }
  } else {
    const float ratio=lastPressurePa/p0_Pa;
    altRel=44330.0f*(1.0f-powf(ratio,0.190295f));
  }
  return true;
}

static inline void updateVelocityIfNew(uint32_t now,bool haveAlt){
  if(!(haveAlt&&baselineSet)) return;
  if(tPrevMs){ const float dt=(now-tPrevMs)/1000.0f; if(dt>0.0f) vVert=(altRel-altPrev)/dt; }
  altPrev=altRel; tPrevMs=now;
}

static inline FlightState decideState(FlightState current,float alt,float v){
  switch(current){
    case LAUNCH_READY: return LAUNCH_READY;
    case ASCENT: if(alt>=SEPARATION_ALT_M || (v<0.0f && alt>APOGEE_MIN_ALT_M)) return SEPARATE; return ASCENT;
    case SEPARATE: return (v<-0.2f)?DESCENT:SEPARATE;
    case DESCENT: {bool near=(alt<5.0f), still=(fabsf(v)<0.3f); return (near&&still)?LANDED:DESCENT;}
    case LANDED: return LANDED;
  }
  return LAUNCH_READY;
}

static inline void updateStateAndServo(uint32_t now){
  if(curState==LAUNCH_READY){
    if(baselineSet && vVert>V_ASCENT_THR && altRel>10.0f){ if(ascentConfirm<3) ++ascentConfirm; }
    else ascentConfirm=0;
    if(ascentConfirm>=3){ curState=ASCENT; missionStarted=true; t0Ms=now; }
  } else {
    curState=decideState(curState,altRel,vVert);
  }
  bool open=false;
  if(curState==SEPARATE && !payloadReleased){ payloadReleased=true; openReasserts=SERVO_REASSERT_N; open=true; }
  else if(payloadReleased && openReasserts>0){ --openReasserts; open=true; }
  if(open) servo.write(SERVO_OPEN_DEG);
}

static inline void handleGroundCommands(uint32_t now_ms){
  static char buf[64]; static uint8_t idx=0;
  while(Serial1.available()>0){
    char c=(char)Serial1.read(); if(c=='\r') continue;
    if(c!='\n' && idx<sizeof(buf)-1){ buf[idx++]=c; continue; }
    buf[idx]='\0'; idx=0; normalizeCommand(buf);
    if(!strcmp(buf,"HOT HONEY")){
      float p_avg=(pressureReads>0)?(pressureSum/(float)pressureReads):lastPressurePa;
      if(p_avg<=0.0f && bmp.performReading()){ lastPressurePa=bmp.pressure; p_avg=lastPressurePa; }
      if(p_avg>0.0f){ p0_Pa=p_avg; baselineSet=true; altRel=0.0f; altPrev=0.0f; tPrevMs=now_ms;
        const int kPa=(int)lroundf(p0_Pa*0.001f);
        Serial1.print("ACK,HOT_HONEY,BASELINE_SET,"); Serial1.println(kPa);
      } else { Serial1.println("ERR,NO_PRESSURE"); }
      continue;
    }
    if(!strcmp(buf,"LEMON PEPPER")){
      servo.write(SERVO_OPEN_DEG); payloadReleased=true; openReasserts=SERVO_REASSERT_N;
      Serial1.println("ACK,LEMON_PEPPER,SERVO_OPEN");
      continue;
    }
    if(!strcmp(buf,"PING")){ Serial1.println("ACK,PONG"); continue; }
    if(buf[0]){ Serial1.print("ERR,BAD_CMD,"); Serial1.println(buf); }
  }
}

void setup(){
  Serial.begin(115200);
  initPeripherals();
}

void loop(){
  const uint32_t now=millis();
  handleGroundCommands(now);
  if(now-lastTickMs<LOOP_PERIOD_MS) return;
  lastTickMs=now;

  tickGyro();
  tickGPS(now);
  bool haveAlt=tickBaro();
  updateVelocityIfNew(now,haveAlt);
  updateStateAndServo(now);

  if(baselineSet){
    ++pkt;
    const uint32_t tMission = missionStarted ? (now - t0Ms) : 0;
    char tStr[16]; missionTime(tStr,sizeof(tStr),tMission);
    sendTelemetry(Serial1,tStr,curState,(payloadReleased?'R':'N'),
                  altRel,lastTempC,gpsLat_e7,gpsLon_e7,lastGr,lastGp,lastGy,lastPressurePa,vVert);
  }
}
