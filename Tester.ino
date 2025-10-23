// Board: Raspberry Pi Pico (rp2040:rp2040:rpipico)
// UART0 (Serial1): TX=GP0, RX=GP1 (defaults)
// I2C0 (Wire): SDA=GP4, SCL=GP5 (defaults)

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// ===== Config =====
#define XBEE_BAUD 9600
#define TEAMID    5
#define SEALEVELPRESSURE_HPA 1013.25

#ifndef LED_BUILTIN
#define LED_BUILTIN 25 // Pico onboard LED (GP25)
#endif

// ===== Sensors =====
Adafruit_BMP3XX bmp;                  // BMP3xx @ 0x77
Adafruit_BNO055 bno(55, 0x28);        // BNO055 @ 0x28 (ADR low) on Wire
SFE_UBLOX_GNSS  gnss;                 // u-blox ZOE @ 0x42 (I2C)

bool bmp_ok=false, bno_ok=false, gnss_ok=false;
uint32_t packetCount = 0, lastSend = 0;

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
  if (!gnss.begin(Wire, 0x42)) return false; // I2C (Wire) @ 0x42
  gnss.setI2COutput(COM_TYPE_UBX);           // quieter bus
  return true;
}

void setup() {
  // No USB Serial usage

  // XBee on UART0 (GP0/GP1)
  Serial1.begin(XBEE_BAUD);

  // I2C0 (Wire) for all sensors
  Wire.begin();            // SDA=GP4, SCL=GP5
  Wire.setClock(400000);   // use 100000 if you have long wires

  // Keep Pico LED ON
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Init sensors (on Wire only)
  bmp_ok  = beginBMP();
  bno_ok  = beginBNO();
  gnss_ok = beginGNSS();
}

void loop() {
  // 1 Hz telemetry
  uint32_t now = millis();
  if (now - lastSend < 1000) return;
  lastSend = now;
  packetCount++;

  // --- BMP3XX (integers only) ---
  int tempC_i = 0;
  int altBaro_m_i = 0;
  if (bmp_ok && bmp.performReading()) {
    float tempC = bmp.temperature;                        // °C
    float alt_m = bmp.readAltitude(SEALEVELPRESSURE_HPA); // m
    tempC_i     = (int)lroundf(tempC);
    altBaro_m_i = (int)lroundf(alt_m);
  }

  // --- BNO055 gyro (deg/s, integers) ---
  int gx_i=0, gy_i=0, gz_i=0;
  if (bno_ok) {
    sensors_event_t gyr;
    if (bno.getEvent(&gyr, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
      gx_i = (int)lroundf(gyr.gyro.x * RAD_TO_DEG);
      gy_i = (int)lroundf(gyr.gyro.y * RAD_TO_DEG);
      gz_i = (int)lroundf(gyr.gyro.z * RAD_TO_DEG);
    }
  }

  // --- GNSS (integers) ---
  uint8_t fixType = 0, siv = 0;
  long lat_e7 = 0, lon_e7 = 0;
  int altGNSS_m_i = 0;
  if (gnss_ok) {
    fixType = gnss.getFixType();  // 0=no fix, 2=2D, 3=3D, etc.
    siv     = gnss.getSIV();
    if (fixType >= 2) {
      lat_e7      = gnss.getLatitude();    // 1e-7 deg
      lon_e7      = gnss.getLongitude();   // 1e-7 deg
      long altmm  = gnss.getAltitude();    // mm
      altGNSS_m_i = (int)lroundf(altmm / 1000.0f);
    }
  }

  // --- Format with printf-style, then send over XBee (UART0) ---
  // CSV layout: <TEAM,PACKET,TcC,AltB_m,Gx,Gy,Gz,Fix,SIV,LatE7,LonE7,AltG_m>
  char line[160];
  int n = snprintf(
    line, sizeof(line),
    "<%d,%lu,%d,%d,%d,%d,%d,%u,%u,%ld,%ld,%d>",
    TEAMID,
    (unsigned long)packetCount,
    tempC_i,
    altBaro_m_i,
    gx_i, gy_i, gz_i,
    (unsigned)fixType,
    (unsigned)siv,
    lat_e7, lon_e7,
    altGNSS_m_i
  );
  if (n > 0) {
    Serial1.write(line, (size_t)n);
    Serial1.write('\n'); // end-of-line
  }
}
