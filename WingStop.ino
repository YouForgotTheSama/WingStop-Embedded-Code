#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Adafruit_BMP3XX.h"
#include <Servo.h>

// Pins / I2C addrs
constexpr uint8_t PIN_XBEE_TX=0, PIN_XBEE_RX=1, PIN_SERVO=27, PIN_VBAT_ADC=26;
constexpr uint8_t I2C_GNSS=0x42, I2C_BNO=0x28, I2C_BMP=0x77;

// Config
constexpr uint8_t  TEAMID=5, SERVO_CLOSED=0, SERVO_OPEN=180, SERVO_REASSERT_N=5, BNO_ID=55;
constexpr uint16_t BASELINE_N=20;
constexpr uint32_t XBEE_BAUD=9600, LOOP_MS=100, GPS_MS=1000;
constexpr float    SEPARATION_ALT_M=490.0f, APOGEE_MIN_M=100.0f, V_ASCENT_THR=2.0f, SLP_HPA=1013.25f;

// ADC → VBAT (set your divider here)
constexpr float ADC_REF_V=3.3f; constexpr int ADC_MAX=4095;
constexpr float R1=200000.0f, R2=100000.0f, VBAT_SCALE=(R1+R2)/R2;

SFE_UBLOX_GNSS gnss;
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno(BNO_ID, I2C_BNO, &Wire);
Servo servo;

enum FlightState{LAUNCH_READY,ASCENT,SEPARATE,DESCENT,LANDED};
const char* STATE_NAME[]={"LAUNCH_READY","ASCENT","SEPARATE","DESCENT","LANDED"};

// State
uint32_t lastLoop=0,lastTx=0,lastGps=0, pkt=0, tPrev=0, t0=0;
bool payloadRel=false, baselineSet=false, missionStarted=false;
float pSum=0,p0=0, altRel=0, altStd=0, tempC=0, vVert=0, altPrev=0, pPa=0, gr=0,gp=0,gyaw=0;
uint16_t pReads=0; uint8_t reassert=0, ascentConfirm=0; long latE7=0,lonE7=0;
FlightState st=LAUNCH_READY;

static inline void toLowerTrim(char* s){
  char* a=s; while(*a==' '||*a=='\t') ++a;
  char* b=a+strlen(a); while(b>a && (b[-1]==' '||b[-1]=='\t'||b[-1]=='\r')) --b;
  size_t n=b-a; memmove(s,a,n); s[n]='\0'; for(size_t i=0;i<n;++i) s[i]=(char)tolower((unsigned char)s[i]);
}
static inline void fmtTime(char* o,size_t n,uint32_t ms){ uint32_t s=ms/1000,hh=s/3600,mm=(s%3600)/60,ss=s%60,hs=(ms%1000)/10; snprintf(o,n,"%02lu:%02lu:%02lu.%02lu",(unsigned long)hh,(unsigned long)mm,(unsigned long)ss,(unsigned long)hs); }
static inline int vbat_mV(){ uint32_t sum=0; for(int i=0;i<8;++i){ sum+=analogRead(PIN_VBAT_ADC); delayMicroseconds(200);} float v=(sum/8.0f)/ADC_MAX*ADC_REF_V*VBAT_SCALE; return (int)lroundf(v*1000.0f); }

static inline void sendTelemetry(Print& out,const char* tStr){
  const float altOut=(baselineSet?altRel:altStd);
  const int lat_i   = (int)lroundf((latE7/1e7f));
  const int lon_i   = (int)lroundf((lonE7/1e7f));
  const int gx_i    = (int)lroundf(gr);
  const int gy_i    = (int)lroundf(gp);
  const int gz_i    = (int)lroundf(gyaw);
  const int press_kPa_i = (int)lroundf(pPa*0.001f);
  const int vvert_i = (int)lroundf(vVert);
  const int alt_i   = (int)lroundf(altOut);
  const int tempC_i = (int)lroundf(tempC);
  const int vbat_i  = vbat_mV(); // millivolts (integer)

  char line[240];
  // TEAM, MISSION_TIME, PACKET, STATE, PL_STATE, ALT, TEMP, VOLTAGE(mV), LAT, LON, Gx,Gy,Gz,, PRESS(kPa), Vvert
  snprintf(line,sizeof(line),"%d,%s,%lu,%s,%c,%d,%d,%d,%d,%d,%d,%d,%d,,%d,%d",
           TEAMID,tStr,(unsigned long)pkt,STATE_NAME[st],(payloadRel?'R':'N'),
           alt_i,tempC_i,vbat_i,lat_i,lon_i,gx_i,gy_i,gz_i,press_kPa_i,vvert_i);
  out.println(line);
}

static inline void initHW(){
  Wire.begin(); Wire.setClock(400000);
  gnss.begin(Wire,I2C_GNSS); gnss.setI2COutput(COM_TYPE_UBX);

  bmp.begin_I2C(I2C_BMP,&Wire);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_12_5_HZ);

  bno.begin(); bno.setExtCrystalUse(true); bno.setMode(OPERATION_MODE_NDOF);

  Serial1.setTX(PIN_XBEE_TX); Serial1.setRX(PIN_XBEE_RX); Serial1.begin(XBEE_BAUD);

  pinMode(PIN_VBAT_ADC, INPUT);
  servo.attach(PIN_SERVO,500,2500); servo.write(SERVO_CLOSED);

#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN,OUTPUT); digitalWrite(LED_BUILTIN,HIGH);
#endif
}

static inline void sampleGyro(){
  sensors_event_t e; if(bno.getEvent(&e,Adafruit_BNO055::VECTOR_GYROSCOPE)){
    gr=e.gyro.x*RAD_TO_DEG; gp=e.gyro.y*RAD_TO_DEG; gyaw=e.gyro.z*RAD_TO_DEG;
  }
}
static inline void sampleGPS(uint32_t now){
  if(now-lastGps<GPS_MS) return; lastGps=now;
  if(gnss.getFixType()>=2){ latE7=gnss.getLatitude(); lonE7=gnss.getLongitude(); }
}
static inline bool sampleBaro(){
  if(!bmp.performReading()) return false;
  tempC=bmp.temperature; pPa=bmp.pressure;
  const float p_hPa=pPa*0.01f; altStd=44330.0f*(1.0f-powf(p_hPa/SLP_HPA,0.190295f));
  if(!baselineSet){
    if(pReads<BASELINE_N){ pSum+=pPa; ++pReads; }
    else { pSum-=pSum/(float)BASELINE_N; pSum+=pPa; }
  }else{
    const float r=pPa/p0; altRel=44330.0f*(1.0f-powf(r,0.190295f));
  }
  return true;
}
static inline void updateVelocity(uint32_t now,bool haveAlt){
  if(!(haveAlt&&baselineSet)) return;
  if(tPrev){ float dt=(now-tPrev)/1000.0f; if(dt>0) vVert=(altRel-altPrev)/dt; }
  altPrev=altRel; tPrev=now;
}
static inline FlightState nextState(FlightState s,float alt,float v){
  switch(s){
    case LAUNCH_READY: return LAUNCH_READY;
    case ASCENT:  return (alt>=SEPARATION_ALT_M || (v<0 && alt>APOGEE_MIN_M))?SEPARATE:ASCENT;
    case SEPARATE:return (v<-0.2f)?DESCENT:SEPARATE;
    case DESCENT: return (alt<5 && fabsf(v)<0.3f)?LANDED:DESCENT;
    case LANDED:  return LANDED;
  } return LAUNCH_READY;
}
static inline void updateFSM(uint32_t now){
  if(st==LAUNCH_READY){
    if(baselineSet && vVert>V_ASCENT_THR && altRel>10){ if(ascentConfirm<3) ++ascentConfirm; } else ascentConfirm=0;
    if(ascentConfirm>=3){ st=ASCENT; missionStarted=true; t0=now; }
  }else st=nextState(st,altRel,vVert);

  bool open=false;
  if(st==SEPARATE && !payloadRel){ payloadRel=true; reassert=SERVO_REASSERT_N; open=true; }
  else if(payloadRel && reassert>0){ --reassert; open=true; }
  if(open) servo.write(SERVO_OPEN);
}

static inline void handleCmd(uint32_t now){
  static char buf[64]; static uint8_t i=0;
  while(Serial1.available()){
    char c=(char)Serial1.read(); if(c=='\r') continue;
    if(c!='\n' && i<sizeof(buf)-1){ buf[i++]=c; continue; }
    buf[i]='\0'; i=0; toLowerTrim(buf); if(!buf[0]) continue;

    if(!strcmp(buf,"hot honey")){
      float p_avg=(pReads? (pSum/(float)pReads):pPa);
      if(p_avg<=0 && bmp.performReading()){ pPa=bmp.pressure; p_avg=pPa; }
      if(p_avg>0){
        p0=p_avg; baselineSet=true; altRel=altPrev=vVert=0; tPrev=now;
        missionStarted=false; t0=0; ascentConfirm=0; st=LAUNCH_READY;
        payloadRel=false; reassert=0;
        Serial1.print("ack,hot_honey,baseline_set,"); Serial1.println((int)lroundf(p0*0.001f));
      }else Serial1.println("err,no_pressure");
      continue;
    }
    if(!strcmp(buf,"lemon pepper")){
      servo.write(SERVO_OPEN); payloadRel=true; reassert=SERVO_REASSERT_N;
      Serial1.println("ack,lemon_pepper,servo_open"); continue;
    }
    if(!strcmp(buf,"lemon pepper close")){
      payloadRel=false; servo.write(SERVO_CLOSED);
      Serial1.println("ack,lemon_pepper,servo_closed"); continue;
    }
    if(!strcmp(buf,"ping")){ Serial1.println("ack,pong"); continue; }
    Serial1.print("err,bad_cmd,"); Serial1.println(buf);
  }
}

void setup(){ initHW(); }

void loop(){
  uint32_t now=millis();
  handleCmd(now);
  if(now-lastLoop<LOOP_MS) return; lastLoop=now;

  sampleGyro();
  sampleGPS(now);
  bool gotAlt=sampleBaro();
  updateVelocity(now,gotAlt);
  updateFSM(now);

  if(now-lastTx>=1000){
    lastTx=now; ++pkt;
    char t[16]; if(missionStarted) fmtTime(t,sizeof t,now-t0); else strcpy(t,"00:00:00.00");
    sendTelemetry(Serial1,t);
  }
}

