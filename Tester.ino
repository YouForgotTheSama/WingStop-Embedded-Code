// RP2040 (Earle Philhower core recommended) — BMP3XX + BNO055 on shared I2C (GP2/GP3) + Servo + XBee

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_BMP3XX.h"
#include <Servo.h>

// ----------- Config -----------
#define TEAMID               5
#define XBEE_BAUD            9600
#define XBEE_TX_PIN          0   // GP0 -> XBee DIN
#define XBEE_RX_PIN          1   // GP1 <- XBee DOUT
#define SEALEVELPRESSURE_HPA (1013.25)

const uint32_t PRINT_INTERVAL_MS  = 1000;  // 1 Hz
const uint32_t REINIT_INTERVAL_MS = 5000;  // periodic re-init attempts

// ---- Servo config ----
const int   SERVO_PIN     = 27;     // GP27
const int   CLOSED_ANGLE  = 0;
const int   OPEN_ANGLE    = 180;
const float TRIGGER_ALT_M = 490.0f;

Servo s;
bool  servo_attached  = false;
bool  servo_triggered = false;

// ----------- Globals (shared I2C on Wire) ----------
Adafruit_BMP3XX bmp;                    // BMP388/390 (0x77 default, 0x76 if SDO=GND)
Adafruit_BNO055 bno(55, 0x28, &Wire);   // BNO055 on same bus (0x29 if ADR high)

uint32_t lastPrint        = 0;
uint32_t packetCount      = 0;
bool     bno_ok           = false;
bool     bmp_ok           = false;
uint32_t lastBNOAttemptMs = 0;
uint32_t lastBMPAttemptMs = 0;

// Altitude baseline (set once at first good BMP read)
bool  bmp_baseline_set = false;
float bmp_alt0         = 0.0f;

// Keep last good BMP temperature so we don't zero it on failures
float bmpTempC_last    = 0.0f;

// CSV emitter (requested order)
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
  bmp_ok = bmp.begin_I2C(0x77, &Wire) || bmp.begin_I2C(0x76, &Wire);
  if (bmp_ok) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }
}

void tryInitBNO() {
  bno_ok = bno.begin();   // uses &Wire (shared bus)
  if (bno_ok) {
    bno.setExtCrystalUse(true);
    bno.setMode(OPERATION_MODE_NDOF);
  }
}

void setup() {
  // Shared I2C bus on GP2/GP3
  Wire.setSDA(2);
  Wire.setSCL(3);
  Wire.begin();
  Wire.setClock(400000);

  // Initial attempts (non-blocking)
  tryInitBMP();  lastBMPAttemptMs = millis();
  tryInitBNO();  lastBNOAttemptMs = millis();

  // UART1 -> XBee
  Serial1.setTX(XBEE_TX_PIN);
  Serial1.setRX(XBEE_RX_PIN);
  Serial1.begin(XBEE_BAUD);

  // Servo
  s.attach(SERVO_PIN, 500, 2500);
  s.write(CLOSED_ANGLE);
  servo_attached = true;
}

void loop() {
  uint32_t now = millis();

  // Periodic re-init if down
  if (!bmp_ok && (now - lastBMPAttemptMs >= REINIT_INTERVAL_MS)) {
    lastBMPAttemptMs = now;
    tryInitBMP();
  }
  if (!bno_ok && (now - lastBNOAttemptMs >= REINIT_INTERVAL_MS)) {
    lastBNOAttemptMs = now;
    tryInitBNO();
  }

  // 1 Hz pacing
  if (now - lastPrint < PRINT_INTERVAL_MS) return;
  lastPrint = now;
  packetCount++;

  // --- Gyro (deg/s) ---
  float gyro_r_dps = 0.0f, gyro_p_dps = 0.0f, gyro_y_dps = 0.0f;
  if (bno_ok) {
    sensors_event_t gyro;
    if (bno.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
      gyro_r_dps = gyro.gyro.x * RAD_TO_DEG;
      gyro_p_dps = gyro.gyro.y * RAD_TO_DEG;
      gyro_y_dps = gyro.gyro.z * RAD_TO_DEG;
    }
  }

  // --- BMP temp + relative altitude (zeroed at first good read) ---
  float bmpTempC    = bmpTempC_last;  // keep last good temp by default
  float bmp_alt_rel = 0.0f;           // altitude defaults to 0.0 if down/fail

  if (bmp_ok) {
    if (bmp.performReading()) {
      bmpTempC_last = bmp.temperature;      // update temp only on success
      bmpTempC      = bmpTempC_last;

      float alt_abs = bmp.readAltitude(SEALEVELPRESSURE_HPA);

      if (!bmp_baseline_set) {
        bmp_alt0 = alt_abs;
        bmp_baseline_set = true;
      }

      bmp_alt_rel = alt_abs - bmp_alt0;     // relative altitude (0 at startup)
    }
  }

  // --- Servo trigger: fire once at/above TRIGGER_ALT_M ---
  if (!servo_triggered && bmp_baseline_set && (bmp_alt_rel >= TRIGGER_ALT_M)) {
    if (!servo_attached) {
      s.attach(SERVO_PIN, 500, 2500);
      servo_attached = true;
    }
    s.write(OPEN_ANGLE);        // deploy
    servo_triggered = true;
  }

  // --- Telemetry out to XBee only (values-only CSV) ---
  sendTelemetryValues(Serial1, TEAMID, packetCount,
                      bmp_alt_rel, bmpTempC,
                      gyro_r_dps, gyro_p_dps, gyro_y_dps);
  Serial1.println();
}
