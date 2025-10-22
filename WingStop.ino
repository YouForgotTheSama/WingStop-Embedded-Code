// RP2040 (Philhower core recommended) — BMP3XX + BNO055 (I2C GP2/GP3) + Servo + XBee
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Adafruit_BMP3XX.h"
#include <Servo.h>
#include <stdio.h>
#include <math.h>

// ======== Config ========
#define TEAMID               5
#define SEALEVELPRESSURE_HPA 1013.25f
#define XBEE_TX_PIN          0
#define XBEE_RX_PIN          1
#define XBEE_BAUD            9600
#define I2C_SDA_PIN          2
#define I2C_SCL_PIN          3

#define SERVO_PIN            27
#define SERVO_CLOSED_DEG     0
#define SERVO_OPEN_DEG       180
#define SEPARATION_ALT_M     490.0f

#define LOOP_PERIOD_MS       1000
#define REINIT_PERIOD_MS     5000

// Velocity thresholds (m/s)
#define V_ASCENT_THR         2.0f
#define V_DESCENT_THR       -2.0f
#define V_STILL_ABS          2.0f

// ======== Devices ========
Adafruit_BMP3XX  bmp;
Adafruit_BNO055  bno(55, 0x28, &Wire);
Servo            servo;

// ======== State ========
uint32_t lastTickMs=0, pkt=0;
uint32_t lastBMPtryMs=0, lastBNOtryMs=0;
bool bmpOK=false, bnoOK=false;

bool  baselineSet=false;
float alt0=0.0f;
float lastTempC=0.0f;

bool     altSampleValid=false;
float    altPrev=0.0f;
uint32_t tPrevMs=0;
float    vVert=0.0f;

bool payloadReleased=false;

// ======== Flight state ========
enum FlightState { LAUNCH_READY, ASCENT, SEPARATE, DESCENT, LANDED };
static const char* STATE_NAME[] = { "LAUNCH_READY","ASCENT","SEPARATE","DESCENT","LANDED" };

// ======== Helpers ========
static void formatMissionTime(char* out, size_t n, uint32_t ms) {
  uint32_t s=ms/1000UL, hh=s/3600UL, mm=(s%3600UL)/60UL, ss=s%60UL, hnd=(ms%1000UL)/10UL;
  snprintf(out, n, "%02lu:%02lu:%02lu.%02lu",
           (unsigned long)hh,(unsigned long)mm,(unsigned long)ss,(unsigned long)hnd);
}

static void tryInitBMP() {
  if (bmpOK) return;
  bmpOK = bmp.begin_I2C(0x77,&Wire) || bmp.begin_I2C(0x76,&Wire);
  if (bmpOK) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }
}

static void tryInitBNO() {
  if (bnoOK) return;
  bnoOK = bno.begin();
  if (bnoOK) { bno.setExtCrystalUse(true); bno.setMode(OPERATION_MODE_NDOF); }
}

static FlightState decideState(float altRel, float v, bool sep, bool baseline) {
  if (!baseline) return LAUNCH_READY;

  if (sep || altRel >= SEPARATION_ALT_M) {
    if (v < V_DESCENT_THR)       return DESCENT;
    if (fabsf(v) < V_STILL_ABS)  return LANDED;
    if (v > V_ASCENT_THR)        return ASCENT;
    return SEPARATE;
  }
  if (v > V_ASCENT_THR)  return ASCENT;
  if (v < V_DESCENT_THR) return DESCENT;
  return LAUNCH_READY;
}

static void sendTelemetry(Stream& out,
                          uint32_t team,
                          const char* tStr,
                          uint32_t count,
                          FlightState st,
                          bool payloadR,
                          float altRel, float tempC,
                          float gr, float gp, float gy)
{
  out.print(team); out.print(',');
  out.print(tStr); out.print(',');
  out.print(count); out.print(',');
  out.print(STATE_NAME[st]); out.print(',');
  out.print(payloadR ? 'R' : 'N'); out.print(',');
  out.print(altRel); out.print(',');
  out.print(tempC); out.print(',');
  out.print(gr); out.print(',');
  out.print(gp); out.print(',');
  out.print(gy);
}

// ======== Setup / Loop ========
void setup() {
  Wire.setSDA(I2C_SDA_PIN); Wire.setSCL(I2C_SCL_PIN); Wire.begin(); Wire.setClock(400000);
  tryInitBMP(); lastBMPtryMs = millis();
  tryInitBNO(); lastBNOtryMs = millis();

  Serial1.setTX(XBEE_TX_PIN); Serial1.setRX(XBEE_RX_PIN); Serial1.begin(XBEE_BAUD);

  servo.attach(SERVO_PIN, 500, 2500);
  servo.write(SERVO_CLOSED_DEG);
}

void loop() {
  uint32_t now = millis();

  if (!bmpOK && (now - lastBMPtryMs >= REINIT_PERIOD_MS)) { lastBMPtryMs = now; tryInitBMP(); }
  if (!bnoOK && (now - lastBNOtryMs >= REINIT_PERIOD_MS)) { lastBNOtryMs = now; tryInitBNO(); }

  if (now - lastTickMs < LOOP_PERIOD_MS) return;
  lastTickMs = now; pkt++;

  // Gyro
  float gr=0.0f, gp=0.0f, gy=0.0f;
  if (bnoOK) {
    sensors_event_t e;
    if (bno.getEvent(&e, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
      gr = e.gyro.x * RAD_TO_DEG; gp = e.gyro.y * RAD_TO_DEG; gy = e.gyro.z * RAD_TO_DEG;
    }
  }

  // BMP temp + relative altitude
  float tempC = lastTempC;
  float altRel = 0.0f;
  bool gotAlt = false;

  if (bmpOK && bmp.performReading()) {
    lastTempC = bmp.temperature; tempC = lastTempC;
    float altAbs = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    if (!baselineSet) { alt0 = altAbs; baselineSet = true; altSampleValid = false; }
    altRel = altAbs - alt0; gotAlt = true;
  }

  // Vertical velocity
  if (gotAlt) {
    if (altSampleValid) {
      float dt = (now - tPrevMs) / 1000.0f; if (dt > 0.0f) vVert = (altRel - altPrev)/dt;
    }
    altPrev = altRel; tPrevMs = now; altSampleValid = true;
  } else {
    vVert = 0.0f;
  }

  // Separation
  if (!payloadReleased && baselineSet && (altRel >= SEPARATION_ALT_M)) {
    servo.write(SERVO_OPEN_DEG);
    payloadReleased = true;
  }

  // State + Telemetry
  FlightState st = decideState(altRel, vVert, payloadReleased, baselineSet);

  char tStr[16]; formatMissionTime(tStr, sizeof(tStr), now);
  sendTelemetry(Serial1, TEAMID, tStr, pkt, st, payloadReleased, altRel, tempC, gr, gp, gy);
  Serial1.println();
}