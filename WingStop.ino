 #include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Adafruit_BMP3XX.h"
#include <Servo.h>
#include <math.h>

#define TEAMID               5
#define SEPARATION_ALT_M     490.0f
#define LOOP_PERIOD_MS       1000
#define XBEE_TX_PIN          0
#define XBEE_RX_PIN          1
#define XBEE_BAUD            9600
#define SERVO_PIN            27
#define SERVO_CLOSED_DEG     0
#define SERVO_OPEN_DEG       180
#define V_ASCENT_THR         2.0f
#define V_DESCENT_THR       -2.0f

Adafruit_BMP3XX  bmp;
Adafruit_BNO055  bno(55, 0x28, &Wire);
Servo            servo;

uint32_t lastTickMs = 0;
uint32_t pkt = 0;

bool     payloadReleased = false;
bool     baselineSet = false;

float    p0_hPa = 0.0f;
float    altRel = 0.0f;
float    lastTempC = 0.0f;

uint32_t tPrevMs = 0;
float    altPrev = 0.0f;
float    vVert   = 0.0f;

int      openReassertsRemaining = 0;

bool     missionStarted = false;
uint32_t t0Ms = 0;

enum FlightState { LAUNCH_READY, ASCENT, SEPARATE, DESCENT, LANDED };
const char* STATE_NAME[] = { "LAUNCH_READY","ASCENT","SEPARATE","DESCENT","LANDED" };
FlightState curState = LAUNCH_READY;

static inline void missionTime(char* out, size_t n, uint32_t ms) {
  const uint32_t s  = ms / 1000UL;
  const uint32_t hh = s / 3600UL;
  const uint32_t mm = (s % 3600UL) / 60UL;
  const uint32_t ss = s % 60UL;
  const uint32_t hs = (ms % 1000UL) / 10UL;
  snprintf(out, n, "%02lu:%02lu:%02lu.%02lu",
           (unsigned long)hh, (unsigned long)mm, (unsigned long)ss, (unsigned long)hs);
}

static FlightState decideState(float alt, float v, bool released, bool haveBaseline) {
  if (!haveBaseline) return LAUNCH_READY;
  const bool near_ground = (alt < 5.0f);
  const bool very_still  = (fabsf(v) < 0.3f);
  if (released || alt >= SEPARATION_ALT_M) {
    if (near_ground && very_still) return LANDED;
    if (v > V_ASCENT_THR)          return ASCENT;
    if (v < -0.2f)                 return DESCENT;
    return SEPARATE;
  }
  if (v > V_ASCENT_THR)  return ASCENT;
  if (v < V_DESCENT_THR) return DESCENT;
  return LAUNCH_READY;
}

static void sendTelemetry(Stream& out,
                          const char* tStr,
                          FlightState st,
                          char payload,
                          float alt, float temp,
                          float gr, float gp, float gy)
{
  out.print(TEAMID); out.print(',');
  out.print(tStr);   out.print(',');
  out.print(pkt);    out.print(',');
  out.print(STATE_NAME[st]); out.print(',');
  out.print(payload); out.print(',');
  out.print((int)roundf(alt));  out.print(',');
  out.print((int)roundf(temp)); out.print(',');
  out.print((int)roundf(gr));   out.print(',');
  out.print((int)roundf(gp));   out.print(',');
  out.print((int)roundf(gy));
}

void setup() {
  Wire.begin();              // GP4 SDA, GP5 SCL on Philhower default
  Wire.setClock(400000);

  bool bmp_ok = bmp.begin_I2C(0x77, &Wire) || bmp.begin_I2C(0x76, &Wire);
  if (bmp_ok) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_6_25_HZ);
  }

  bno.begin();
  bno.setExtCrystalUse(true);
  bno.setMode(OPERATION_MODE_NDOF);

  Serial1.setTX(XBEE_TX_PIN);
  Serial1.setRX(XBEE_RX_PIN);
  Serial1.begin(XBEE_BAUD);

  servo.attach(SERVO_PIN, 500, 2500);
  servo.write(SERVO_CLOSED_DEG);
}

void loop() {
  const uint32_t now = millis();
  if (now - lastTickMs < LOOP_PERIOD_MS) return;
  lastTickMs = now;

  float gr = 0, gp = 0, gy = 0;
  sensors_event_t ge;
  if (bno.getEvent(&ge, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
    gr = ge.gyro.x * RAD_TO_DEG;
    gp = ge.gyro.y * RAD_TO_DEG;
    gy = ge.gyro.z * RAD_TO_DEG;
  }

  bool gotAlt = false;
  if (bmp.performReading()) {
    lastTempC = bmp.temperature;
    const float P_hPa = bmp.pressure;
    if (!baselineSet) {
      p0_hPa = P_hPa;
      baselineSet = true;
      altRel = 0.0f;
    } else {
      const float ratio = P_hPa / p0_hPa;
      altRel = 44330.0f * (1.0f - powf(ratio, 0.190295f));
    }
    gotAlt = true;
  }

  if (gotAlt) {
    if (tPrevMs) {
      const float dt = (now - tPrevMs) / 1000.0f;
      if (dt > 0) vVert = (altRel - altPrev) / dt;
    }
    altPrev = altRel;
    tPrevMs = now;
  } else {
    vVert = 0.0f;
  }

  if (!payloadReleased && baselineSet && (altRel >= SEPARATION_ALT_M)) {
    servo.write(SERVO_OPEN_DEG);
    payloadReleased = true;
    openReassertsRemaining = 5;
  }

  if (payloadReleased && openReassertsRemaining > 0) {
    servo.write(SERVO_OPEN_DEG);
    openReassertsRemaining--;
  }

  curState = decideState(altRel, vVert, payloadReleased, baselineSet);

  if (!missionStarted && baselineSet && (altRel > 3.0f) && (vVert >= 2.0f)) {
    missionStarted = true;
    t0Ms = now;
    pkt = 0;
  }

  if (missionStarted) {
    pkt++;
    const uint32_t tMission = now - t0Ms;
    char tStr[16]; missionTime(tStr, sizeof(tStr), tMission);
    sendTelemetry(Serial1, tStr, curState, (payloadReleased ? 'R' : 'N'),
                  altRel, lastTempC, gr, gp, gy);
    Serial1.println();
  }
}
