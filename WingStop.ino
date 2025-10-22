// RP2040 (official core) — BMP3XX + BNO055 (default I2C GP6/GP7) + Servo + XBee
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Adafruit_BMP3XX.h"
#include <Servo.h>
#include <math.h>  // for roundf

// ---------- Config ----------
#define TEAMID               5
#define SEALEVELPRESSURE_HPA 1013.25f

// UART1 -> XBee
#define XBEE_TX_PIN          0   // GP0 -> XBee DIN
#define XBEE_RX_PIN          1   // GP1 <- XBee DOUT
#define XBEE_BAUD            9600

// Servo
#define SERVO_PIN            27
#define SERVO_CLOSED_DEG     0
#define SERVO_OPEN_DEG       180
#define SEPARATION_ALT_M     490.0f

// Loop rate & thresholds
#define LOOP_PERIOD_MS       1000   // 1 Hz
#define V_ASCENT_THR         2.0f   // m/s
#define V_DESCENT_THR       -2.0f   // m/s
#define V_STILL_ABS          2.0f   // m/s (|v|<2 => “still”)

// ---------- Devices ----------
Adafruit_BMP3XX  bmp;
Adafruit_BNO055  bno(55, 0x28, &Wire);
Servo            servo;

// ---------- State ----------
uint32_t lastTickMs = 0;
uint32_t pkt = 0;
bool     payloadReleased = false;
bool     baselineSet = false;

float alt0 = 0.0f;     // absolute altitude baseline
float altRel = 0.0f;   // current relative altitude
float lastTempC = 0.0f;

uint32_t tPrevMs = 0;  // for velocity
float    altPrev = 0.0f;
float    vVert   = 0.0f;

// Flight states
enum FlightState { LAUNCH_READY, ASCENT, SEPARATE, DESCENT, LANDED };
const char* STATE_NAME[] = { "LAUNCH_READY","ASCENT","SEPARATE","DESCENT","LANDED" };
FlightState curState = LAUNCH_READY;

// ---------- Helpers ----------
static void missionTime(char* out, size_t n, uint32_t ms) {
  uint32_t s  = ms / 1000UL;
  uint32_t hh = s / 3600UL;
  uint32_t mm = (s % 3600UL) / 60UL;
  uint32_t ss = s % 60UL;
  uint32_t hs = (ms % 1000UL) / 10UL;  // hundredths
  snprintf(out, n, "%02lu:%02lu:%02lu.%02lu",
           (unsigned long)hh, (unsigned long)mm, (unsigned long)ss, (unsigned long)hs);
}

static FlightState decideState(float alt, float v, bool released, bool haveBaseline) {
  if (!haveBaseline) return LAUNCH_READY;

  // Post-separation band
  if (released || alt >= SEPARATION_ALT_M) {
    if (v < V_DESCENT_THR)        return DESCENT;
    if (fabsf(v) < V_STILL_ABS)   return LANDED;
    if (v > V_ASCENT_THR)         return ASCENT;  // rare edge
    return SEPARATE;
  }

  // Pre-separation
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
  // Round to nearest integer and print with NO decimals
  int alt_i  = (int)roundf(alt);
  int temp_i = (int)roundf(temp);
  int gr_i   = (int)roundf(gr);
  int gp_i   = (int)roundf(gp);
  int gy_i   = (int)roundf(gy);

  out.print(TEAMID); out.print(',');
  out.print(tStr);   out.print(',');
  out.print(pkt);    out.print(',');
  out.print(STATE_NAME[st]); out.print(',');
  out.print(payload); out.print(',');
  out.print(alt_i);  out.print(',');
  out.print(temp_i); out.print(',');
  out.print(gr_i);   out.print(',');
  out.print(gp_i);   out.print(',');
  out.print(gy_i);
}

// ---------- Setup / Loop ----------
void setup() {
  // Default I2C (SDA=GP6, SCL=GP7)
  Wire.begin();
  Wire.setClock(400000);

  // Init sensors (simple one-shot init)
  bmp.begin_I2C(0x77, &Wire) || bmp.begin_I2C(0x76, &Wire);
  if (bmp.initialized()) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  bno.begin();
  bno.setExtCrystalUse(true);
  bno.setMode(OPERATION_MODE_NDOF);

  // Telemetry to XBee
  Serial1.setTX(XBEE_TX_PIN);
  Serial1.setRX(XBEE_RX_PIN);
  Serial1.begin(XBEE_BAUD);

  // Servo
  servo.attach(SERVO_PIN, 500, 2500);
  servo.write(SERVO_CLOSED_DEG);
}

void loop() {
  uint32_t now = millis();
  if (now - lastTickMs < LOOP_PERIOD_MS) return;  // 1 Hz
  lastTickMs = now;
  pkt++;

  // --- Gyro (deg/s) ---
  float gr = 0, gp = 0, gy = 0;
  sensors_event_t ge;
  if (bno.getEvent(&ge, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
    gr = ge.gyro.x * RAD_TO_DEG;
    gp = ge.gyro.y * RAD_TO_DEG;
    gy = ge.gyro.z * RAD_TO_DEG;
  }

  // --- Baro: temp + relative altitude (zeroed at first good read) ---
  bool gotAlt = false;
  if (bmp.performReading()) {
    lastTempC = bmp.temperature;
    float altAbs = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    if (!baselineSet) { alt0 = altAbs; baselineSet = true; }
    altRel = altAbs - alt0;
    gotAlt = true;
  }

  // --- Vertical velocity (simple Δalt/Δt) ---
  if (gotAlt) {
    if (tPrevMs) {
      float dt = (now - tPrevMs) / 1000.0f;
      if (dt > 0) vVert = (altRel - altPrev) / dt;
    }
    altPrev = altRel;
    tPrevMs = now;
  } else {
    vVert = 0.0f;  // no new sample
  }

  // --- Payload release at ≥ 490 m ---
  if (!payloadReleased && baselineSet && (altRel >= SEPARATION_ALT_M)) {
    servo.write(SERVO_OPEN_DEG);
    payloadReleased = true;
  }

  // --- State & Telemetry ---
  curState = decideState(altRel, vVert, payloadReleased, baselineSet);

  char tStr[16]; missionTime(tStr, sizeof(tStr), now);
  sendTelemetry(Serial1, tStr, curState, (payloadReleased ? 'R' : 'N'),
                altRel, lastTempC, gr, gp, gy);
  Serial1.println();
}
