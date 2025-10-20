#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_BMP3XX.h"

// ----------- Config -----------
#define TEAMID                 5
#define XBEE_BAUD              9600
#define XBEE_TX_PIN            0   // GP0 -> XBee DIN
#define XBEE_RX_PIN            1   // GP1 <- XBee DOUT
#define SEALEVELPRESSURE_HPA   (1013.25)
const uint32_t PRINT_INTERVAL_MS   = 1000;   // 1 Hz
const uint32_t REINIT_INTERVAL_MS  = 5000;   // try reinit every 5 s if a sensor is down

// ----------- Globals ----------
Adafruit_BMP3XX bmp;                       // On Wire (I2C0)
Adafruit_BNO055 bno(55, 0x28, &Wire1);     // On Wire1 (I2C1); use 0x29 if ADR high

uint32_t lastPrint        = 0;
uint32_t packetCount      = 0;
bool bno_ok               = false;
bool bmp_ok               = false;
uint32_t lastBNOAttemptMs = 0;
uint32_t lastBMPAttemptMs = 0;

// Values-only CSV emitter in the requested order
static void sendTelemetryValues(Stream &out,
                                uint32_t teamId,
                                uint32_t count,
                                float bmp_alt_m,
                                float bmpTempC,
                                float gyro_r_dps, float gyro_p_dps, float gyro_y_dps)
{
  out.print(teamId);     out.print(',');
  out.print(count);      out.print(',');
  out.print(bmp_alt_m);  out.print(',');
  out.print(bmpTempC);   out.print(',');
  out.print(gyro_r_dps); out.print(',');
  out.print(gyro_p_dps); out.print(',');
  out.print(gyro_y_dps);
}

void tryInitBMP() {
  // Try both I2C addresses
  bmp_ok = bmp.begin_I2C(0x77, &Wire) || bmp.begin_I2C(0x76, &Wire);
  if (bmp_ok) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println("BMP3XX: initialized");
  } else {
    Serial.println("WARN: BMP3XX not found (GP12/GP13). Will retry...");
  }
}

void tryInitBNO() {
  bno_ok = bno.begin();
  if (bno_ok) {
    bno.setExtCrystalUse(true);
    bno.setMode(OPERATION_MODE_NDOF); // Fusion mode
    Serial.println("BNO055: initialized");
  } else {
    Serial.println("WARN: BNO055 not detected (GP4/GP5). Will retry...");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("RP2040 BNO055 + BMP390 -> XBee @ 1 Hz (values-only CSV)");

  // ----------- I2C setup -----------
  // I2C0 -> BMP3XX on GP12/GP13
  Wire.setSDA(12);
  Wire.setSCL(13);
  Wire.begin();
  Wire.setClock(400000);

  // I2C1 -> BNO055 on GP4/GP5
  Wire1.setSDA(4);
  Wire1.setSCL(5);
  Wire1.begin();
  Wire1.setClock(400000);

  // Initial attempts (non-blocking)
  tryInitBMP();
  lastBMPAttemptMs = millis();

  tryInitBNO();
  lastBNOAttemptMs = millis();

  // ----------- XBee UART1 -----------
  Serial1.setTX(XBEE_TX_PIN);
  Serial1.setRX(XBEE_RX_PIN);
  Serial1.begin(XBEE_BAUD);

  // Legend (USB only, once)
  Serial.println("ORDER: TEAMID,COUNT,BMP_ALT_M,BMP_TEMP_C,GYRO_R_DPS,GYRO_P_DPS,GYRO_Y_DPS");
}

void loop() {
  uint32_t now = millis();

  // Periodic re-init attempts if needed (non-blocking)
  if (!bmp_ok && (now - lastBMPAttemptMs >= REINIT_INTERVAL_MS)) {
    lastBMPAttemptMs = now;
    tryInitBMP();
  }
  if (!bno_ok && (now - lastBNOAttemptMs >= REINIT_INTERVAL_MS)) {
    lastBNOAttemptMs = now;
    tryInitBNO();
  }

  // 1 Hz throttle
  if (now - lastPrint < PRINT_INTERVAL_MS) return;
  lastPrint = now;
  packetCount++;

  // ----------- Gyro values -----------
  float gyro_r_dps = 0.0f, gyro_p_dps = 0.0f, gyro_y_dps = 0.0f;
  if (bno_ok) {
    sensors_event_t gyro;
    if (bno.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
      gyro_r_dps = gyro.gyro.x * RAD_TO_DEG;
      gyro_p_dps = gyro.gyro.y * RAD_TO_DEG;
      gyro_y_dps = gyro.gyro.z * RAD_TO_DEG;
    } else {
      Serial.println("WARN: BNO055 read failed (sending 0.0 for gyro).");
    }
  }

  // ----------- BMP values -----------
  float bmpTempC = 0.0f;
  float bmp_alt  = 0.0f;
  if (bmp_ok) {
    if (bmp.performReading()) {
      bmpTempC = bmp.temperature;
      bmp_alt  = bmp.readAltitude(SEALEVELPRESSURE_HPA); // set QNH for accuracy
    } else {
      Serial.println("WARN: BMP3XX read failed (sending 0.0 for temp/alt).");
    }
  }

  // ----------- Send telemetry (values-only CSV) -----------
  sendTelemetryValues(Serial1, TEAMID, packetCount,
                      bmp_alt, bmpTempC,
                      gyro_r_dps, gyro_p_dps, gyro_y_dps);
  Serial1.println();

  // Echo to USB Serial
  sendTelemetryValues(Serial, TEAMID, packetCount,
                      bmp_alt, bmpTempC,
                      gyro_r_dps, gyro_p_dps, gyro_y_dps);
  Serial.println();
}
