#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Adafruit_BMP3XX.h"
#include <Servo.h>
#include <math.h>

#define TEAMID                5
#define SEPARATION_ALT_M      490.0f
#define APOGEE_MIN_ALT_M      100.0f
#define LOOP_PERIOD_MS        100
#define BASELINE_READINGS     20

#define XBEE_TX_PIN           0
#define XBEE_RX_PIN           1
#define XBEE_BAUD             9600
#define SERVO_PIN             27
#define SERVO_CLOSED_DEG      0
#define SERVO_OPEN_DEG        180
#define BNO055_SENSOR_ID      55
#define BNO055_I2C_ADDR       0x28

#define V_ASCENT_THR          2.0f
#define V_DESCENT_THR         -2.0f

SFE_UBLOX_GNSS myGNSS;
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno(BNO055_SENSOR_ID, BNO055_I2C_ADDR, &Wire);
Servo servo;

uint32_t lastTickMs = 0;
uint32_t pkt = 0;
bool payloadReleased = false;
bool baselineSet = false;
float pressureSum = 0.0f;
int   pressureReadings = 0;
float p0_Pa = 0.0f;
float altRel = 0.0f;
float lastTempC = 0.0f;
uint32_t tPrevMs = 0;
float altPrev = 0.0f;
float vVert = 0.0f;
int openReassertsRemaining = 0;
bool missionStarted = false;
uint32_t t0Ms = 0;

long gpsLat_e7 = 0;
long gpsLon_e7 = 0;
uint32_t lastGpsMs = 0;
float lastPressurePa = 0.0f;

enum FlightState { LAUNCH_READY, ASCENT, SEPARATE, DESCENT, LANDED };
const char* STATE_NAME[] = { "LAUNCH_READY","ASCENT","SEPARATE","DESCENT","LANDED" };
FlightState curState = LAUNCH_READY;
FlightState prevState = LAUNCH_READY;

bool telemetryHalted = false;
uint32_t launchExitMs = 0;

static inline void missionTime(char* out, size_t n, uint32_t ms) {
  const uint32_t s  = ms / 1000UL;
  const uint32_t hh = s / 3600UL;
  const uint32_t mm = (s % 3600UL) / 60UL;
  const uint32_t ss = s % 60UL;
  const uint32_t hs = (ms % 1000UL) / 10UL;
  snprintf(out, n, "%02lu:%02lu:%02lu.%02lu",
           (unsigned long)hh, (unsigned long)mm, (unsigned long)ss, (unsigned long)hs);
}

static FlightState decideState(FlightState currentState, float alt, float v, bool haveBaseline) {
  if (!haveBaseline) return LAUNCH_READY;
  switch (currentState) {
    case LAUNCH_READY:
      if (v > V_ASCENT_THR && alt > 10.0f) return ASCENT;
      return LAUNCH_READY;
    case ASCENT: {
      bool apogeeDetected = (v < 0.0f && alt > APOGEE_MIN_ALT_M);
      if (alt >= SEPARATION_ALT_M || apogeeDetected) return SEPARATE;
      return ASCENT;
    }
    case SEPARATE:
      if (v < -0.2f) return DESCENT;
      return SEPARATE;
    case DESCENT: {
      bool near_ground = (alt < 5.0f);
      bool very_still = (fabsf(v) < 0.3f);
      if (near_ground && very_still) return LANDED;
      return DESCENT;
    }
    case LANDED:
      return LANDED;
  }
  return LAUNCH_READY;
}

static void sendTelemetry(Stream& out, const char* tStr, FlightState st, char payload,
                          float alt, float temp, long lat_e7, long lon_e7,
                          float gr, float gp, float gy, float pressPa, float v) {
  const float lat_deg = lat_e7 / 1e7f;
  const float lon_deg = lon_e7 / 1e7f;

  out.print(TEAMID); out.print(',');
  out.print(tStr); out.print(',');
  out.print(pkt); out.print(',');
  out.print(STATE_NAME[st]); out.print(',');
  out.print(payload); out.print(',');
  out.print((int)roundf(alt)); out.print(',');
  out.print((int)roundf(temp)); out.print(',');
  out.print(lat_deg, 5); out.print(',');
  out.print(lon_deg, 5); out.print(',');
  out.print((int)roundf(gr)); out.print(',');
  out.print((int)roundf(gp)); out.print(',');
  out.print((int)roundf(gy));
  out.print(',');  // blank field 1
  out.print(',');  // blank field 2
  out.print((int)roundf(pressPa * 0.001f)); out.print(','); // kPa
  out.print((int)roundf(v));
}

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  myGNSS.begin(Wire, 0x42);
  myGNSS.setI2COutput(COM_TYPE_UBX);

  bmp.begin_I2C(0x77, &Wire) || bmp.begin_I2C(0x76, &Wire);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_12_5_HZ);

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

  if (now - lastGpsMs >= 1000) {
    lastGpsMs = now;
    gpsLat_e7 = myGNSS.getLatitude();
    gpsLon_e7 = myGNSS.getLongitude();
  }

  bool gotAlt = false;
  if (bmp.performReading()) {
    lastTempC = bmp.temperature;
    const float P_Pa = bmp.pressure;
    lastPressurePa = P_Pa;

    if (!baselineSet) {
      if (pressureReadings < BASELINE_READINGS) {
        pressureSum += P_Pa;
        pressureReadings++;
      } else {
        p0_Pa = pressureSum / BASELINE_READINGS;
        baselineSet = true;
        altRel = 0.0f;
        altPrev = altRel;
        tPrevMs = now;
      }
    } else {
      altRel = 44330.0f * (1.0f - powf(P_Pa / p0_Pa, 0.190295f));
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

  prevState = curState;
  curState = decideState(curState, altRel, vVert, baselineSet);

  if (prevState == LAUNCH_READY && curState != LAUNCH_READY && launchExitMs == 0) {
    launchExitMs = now; // mark the moment we leave LAUNCH_READY
  }
  if (launchExitMs && (now - launchExitMs >= 5000)) {
    telemetryHalted = true; // stop telemetry 5 seconds after launch
  }

  bool commandServoOpen = false;
  if (curState == SEPARATE && !payloadReleased) {
    payloadReleased = true;
    openReassertsRemaining = 5;
    commandServoOpen = true;
  } else if (payloadReleased && openReassertsRemaining > 0) {
    openReassertsRemaining--;
    commandServoOpen = true;
  }
  if (commandServoOpen) servo.write(SERVO_OPEN_DEG);

  if (!missionStarted && curState == ASCENT) {
    missionStarted = true;
    t0Ms = now;
  }

  if (baselineSet && !telemetryHalted) {
    pkt++;
    const uint32_t tMission = missionStarted ? (now - t0Ms) : 0;
    char tStr[16]; missionTime(tStr, sizeof(tStr), tMission);
    sendTelemetry(Serial1, tStr, curState, (payloadReleased ? 'R' : 'N'),
                  altRel, lastTempC, gpsLat_e7, gpsLon_e7, gr, gp, gy, lastPressurePa, vVert);
    Serial1.println();
  }
}