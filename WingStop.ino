#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#define XBEE_BAUD 9600
#define TEAMID    5
#define SEALEVELPRESSURE_HPA 1013.25f

#ifndef LED_BUILTIN
#define LED_BUILTIN 25
#endif

#define PIN_VBAT_ADC 26
static const float ADC_REF_V = 3.3f;
static const int   ADC_MAX   = 4095;
static const float R1 = 200000.0f;
static const float R2 = 100000.0f;
static const float VBAT_SCALE = (R1 + R2) / R2;

Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno(55, 0x28);
SFE_UBLOX_GNSS  gnss;

bool bmp_ok=false, bno_ok=false, gnss_ok=false;

uint32_t packetCount = 0, lastSend = 0, bootMillis = 0, lastGps = 0;

enum SwState { LAUNCH_READY, ASCENT, DESCENT, LANDED };
const char* SW_NAME[] = {"LAUNCH_READY","ASCENT","DESCENT","LANDED"};
SwState sw_state = LAUNCH_READY;
char    pl_state = 'N';

int lastAltBaro_m_i = 0;
int vVert_i = 0;

static inline void missionTime(char* out, size_t n, uint32_t ms){
  uint32_t s = ms / 1000UL;
  uint32_t hh = s / 3600UL;
  uint32_t mm = (s % 3600UL) / 60UL;
  uint32_t ss = s % 60UL;
  uint32_t hs = (ms % 1000UL) / 10UL;
  snprintf(out, n, "%02lu:%02lu:%02lu.%02lu",
           (unsigned long)hh, (unsigned long)mm, (unsigned long)ss, (unsigned long)hs);
}

static inline int readVBat_V_i(){
  uint32_t sum = 0;
  const int N = 8;
  for(int i=0;i<N;++i){ sum += analogRead(PIN_VBAT_ADC); delayMicroseconds(200); }
  float adc = (float)sum / (float)N;
  float v_adc = (adc / ADC_MAX) * ADC_REF_V;
  float v_bat = v_adc * VBAT_SCALE;
  return (int)lroundf(v_bat);
}

bool beginBMP() {
  if (!bmp.begin_I2C(0x77, &Wire)) return false;
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_12_5_HZ);
  return true;
}
bool beginBNO() {
  if (!bno.begin()) return false;
  bno.setExtCrystalUse(true);
  bno.setMode(OPERATION_MODE_NDOF);
  return true;
}
bool beginGNSS() {
  if (!gnss.begin(Wire, 0x42)) return false;
  gnss.setI2COutput(COM_TYPE_UBX);
  return true;
}

// ---- NEW: raw state inference (no debouncing) ----
static inline SwState inferStateRaw(int altBaro_m_i, int vVert_i){
  if (vVert_i >  2) return ASCENT;
  if (vVert_i < -2) return DESCENT;
  if (abs(vVert_i) <= 0 && abs(altBaro_m_i) <= 2) return LANDED;
  return LAUNCH_READY;
}

void setup() {
  Serial1.begin(XBEE_BAUD);
  Wire.begin();
  Wire.setClock(400000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(PIN_VBAT_ADC, INPUT);
  bmp_ok  = beginBMP();
  bno_ok  = beginBNO();
  gnss_ok = beginGNSS();
  bootMillis = millis();
}

void loop() {
  uint32_t now = millis();
  if (now - lastSend < 1000) return;
  uint32_t dt_ms = (lastSend==0) ? 1000 : (now - lastSend);
  lastSend = now;
  packetCount++;

  int tempC_i = 0;
  int altBaro_m_i = lastAltBaro_m_i;
  int press_kPa_i = 0;
  if (bmp_ok && bmp.performReading()) {
    float tempC = bmp.temperature;
    float alt_m = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    tempC_i     = (int)lroundf(tempC);
    altBaro_m_i = (int)lroundf(alt_m);
    press_kPa_i = (int)lroundf(bmp.pressure * 0.001f);
  }

  if (dt_ms > 0) {
    float v = ((float)(altBaro_m_i - lastAltBaro_m_i)) / ((float)dt_ms / 1000.0f);
    vVert_i = (int)lroundf(v);
  }
  lastAltBaro_m_i = altBaro_m_i;

  int gx_i=0, gy_i=0, gz_i=0;
  if (bno_ok) {
    sensors_event_t gyr;
    if (bno.getEvent(&gyr, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
      gx_i = (int)lroundf(gyr.gyro.x * RAD_TO_DEG);
      gy_i = (int)lroundf(gyr.gyro.y * RAD_TO_DEG);
      gz_i = (int)lroundf(gyr.gyro.z * RAD_TO_DEG);
    }
  }

  static long lat_e7_last=0, lon_e7_last=0;
  if (gnss_ok && now - lastGps >= 1000) {
    lastGps = now;
    if (gnss.getFixType() >= 2) {
      lat_e7_last = gnss.getLatitude();
      lon_e7_last = gnss.getLongitude();
    }
  }
  long lat_deg_i = (long)(lat_e7_last / 10000000L);
  long lon_deg_i = (long)(lon_e7_last / 10000000L);

  int vbat_V_i = readVBat_V_i();

  // ---- NEW: Debounced (3-checks) state changes ----
  static SwState cand_state = sw_state;
  static uint8_t cand_count = 0;
  SwState proposed = inferStateRaw(altBaro_m_i, vVert_i);

  if (proposed == sw_state) {
    cand_state = sw_state;
    cand_count = 0;
  } else {
    if (proposed == cand_state) {
      if (++cand_count >= 3) {
        sw_state = proposed;
        cand_count = 0;
      }
    } else {
      cand_state = proposed;
      cand_count = 1;
    }
  }

  char tStr[16];
  missionTime(tStr, sizeof(tStr), now - bootMillis);

  char line[192];
  int n = snprintf(
    line, sizeof(line),
    "%d,%s,%lu,%s,%c,%d,%d,%d,%ld,%ld,%d,%d,%d,,%d,%d",
    TEAMID,
    tStr,
    (unsigned long)packetCount,
    SW_NAME[sw_state],
    pl_state,
    altBaro_m_i,
    tempC_i,
    vbat_V_i,
    lat_deg_i,
    lon_deg_i,
    gx_i, gy_i, gz_i,
    press_kPa_i,
    vVert_i
  );
  if (n > 0) {
    Serial1.write(line, (size_t)n);
    Serial1.write('\n');
  }
}

