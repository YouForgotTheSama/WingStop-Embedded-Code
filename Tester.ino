#include <Arduino.h>
#include <Wire.h>

// BMP3XX
#include "Adafruit_BMP3XX.h"

// BNO055
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// u-blox GNSS (ZOE)
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// ===== I2C addresses (defaults) =====
constexpr uint8_t I2C_ADDR_BMP3XX = 0x77; // default for Adafruit BMP3xx
constexpr uint8_t I2C_ADDR_BNO055 = 0x28; // ADR low (default)
constexpr uint8_t I2C_ADDR_GNSS   = 0x42; // u-blox default

Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno(55, I2C_ADDR_BNO055, &Wire);
SFE_UBLOX_GNSS  gnss;

bool bmp_ok=false, bno_ok=false, gnss_ok=false;
uint32_t lastPrint=0;

bool beginBMP() {
  if (!bmp.begin_I2C(I2C_ADDR_BMP3XX, &Wire)) return false;
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
  if (!gnss.begin(Wire, I2C_ADDR_GNSS)) return false;
  gnss.setI2COutput(COM_TYPE_UBX);
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("CanSat Sensor Test (BMP3XX default addr, BNO055, u-blox ZOE)");

  Wire.begin();
  Wire.setClock(400000);

  bmp_ok = beginBMP();
  bno_ok = beginBNO();
  gnss_ok = beginGNSS();

  Serial.print("BMP3XX: "); Serial.println(bmp_ok ? "OK" : "NOT FOUND");
  Serial.print("BNO055 : "); Serial.println(bno_ok ? "OK" : "NOT FOUND");
  Serial.print("GNSS   : "); Serial.println(gnss_ok ? "OK" : "NOT FOUND");
  Serial.println("---- Reading at 1 Hz ----");
}

void loop() {
  const uint32_t now = millis();
  if (now - lastPrint < 1000) return;
  lastPrint = now;

  float tempC = NAN, press_kPa = NAN;
  if (bmp_ok && bmp.performReading()) {
    tempC = bmp.temperature;
    press_kPa = bmp.pressure * 0.001f; // Pa -> kPa
  }

  float euler_h=NAN, euler_r=NAN, euler_p=NAN;
  float gyro_x=NAN, gyro_y=NAN, gyro_z=NAN;
  if (bno_ok) {
    sensors_event_t orient;
    if (bno.getEvent(&orient, Adafruit_BNO055::VECTOR_EULER)) {
      euler_h = orient.orientation.x;
      euler_r = orient.orientation.z;
      euler_p = orient.orientation.y;
    }
    sensors_event_t ge;
    if (bno.getEvent(&ge, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
      gyro_x = ge.gyro.x * RAD_TO_DEG;
      gyro_y = ge.gyro.y * RAD_TO_DEG;
      gyro_z = ge.gyro.z * RAD_TO_DEG;
    }
  }

  bool hasFix=false;
  uint8_t fixType=0, siv=0;
  long lat_e7=0, lon_e7=0, alt_mm=0;
  uint16_t year=0; uint8_t mon=0, day=0, hr=0, min=0, sec=0;
  if (gnss_ok) {
    fixType = gnss.getFixType();
    hasFix = (fixType >= 2);
    siv = gnss.getSIV();
    if (hasFix) {
      lat_e7 = gnss.getLatitude();
      lon_e7 = gnss.getLongitude();
      alt_mm = gnss.getAltitude();
    }
    year = gnss.getYear(); mon = gnss.getMonth(); day = gnss.getDay();
    hr = gnss.getHour();   min = gnss.getMinute(); sec = gnss.getSecond();
  }

  Serial.println("===== SENSOR READ =====");
  if (bmp_ok) {
    Serial.print("BMP3XX  | Temp(C): ");
    Serial.print(isnan(tempC) ? NAN : tempC, 2);
    Serial.print("  Press(kPa): ");
    Serial.println(isnan(press_kPa) ? NAN : press_kPa, 2);
  } else {
    Serial.println("BMP3XX  | not detected");
  }

  if (bno_ok) {
    Serial.print("BNO055  | Euler(deg) H/R/P: ");
    if (isnan(euler_h)) Serial.print("nan"); else Serial.print(euler_h, 1);
    Serial.print("/");
    if (isnan(euler_r)) Serial.print("nan"); else Serial.print(euler_r, 1);
    Serial.print("/");
    if (isnan(euler_p)) Serial.print("nan"); else Serial.print(euler_p, 1);
    Serial.print("  Gyro(deg/s) X/Y/Z: ");
    if (isnan(gyro_x)) Serial.print("nan"); else Serial.print(gyro_x, 1);
    Serial.print("/");
    if (isnan(gyro_y)) Serial.print("nan"); else Serial.print(gyro_y, 1);
    Serial.print("/");
    if (isnan(gyro_z)) Serial.print("nan"); else Serial.print(gyro_z, 1);
    Serial.println();
  } else {
    Serial.println("BNO055  | not detected");
  }

  if (gnss_ok) {
    Serial.print("GNSS    | FixType:");
    Serial.print(fixType);
    Serial.print("  SIV:");
    Serial.print(siv);
    Serial.print("  Time(UTC): ");
    if (year) {
      char tbuf[24];
      snprintf(tbuf, sizeof(tbuf), "%04u-%02u-%02u %02u:%02u:%02u",
               (unsigned)year,(unsigned)mon,(unsigned)day,(unsigned)hr,(unsigned)min,(unsigned)sec);
      Serial.print(tbuf);
    } else {
      Serial.print("n/a");
    }
    Serial.print("  Pos: ");
    if (hasFix) {
      Serial.print(lat_e7 / 1e7, 5);
      Serial.print(", ");
      Serial.print(lon_e7 / 1e7, 5);
      Serial.print("  Alt(m): ");
      Serial.print(alt_mm / 1000.0, 2);
    } else {
      Serial.print("no fix");
    }
    Serial.println();
  } else {
    Serial.println("GNSS    | not detected");
  }
}

