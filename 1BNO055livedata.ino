/*
  RAK11300 (RP2040, Earle Philhower core)
  BNO055 live Euler angles over I2C
  I2C pins: SDA=GP2, SCL=GP3
  Serial: 115200 baud
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define SDA_PIN   2
#define SCL_PIN   3
#define BNO_ADDR  0x29   // use 0x29 if ADR is pulled HIGH on the BNO055

Adafruit_BNO055 bno(55, BNO_ADDR, &Wire);

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nBNO055 init...");

  // I2C on GP2/GP3 @ 400kHz
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected at 0x28/0x29. Check 3V3, GND, SDA=GP2, SCL=GP3, ADR.");
    while (1) delay(10);
  }

  bno.setExtCrystalUse(true);           // use external crystal if present
  bno.setMode(OPERATION_MODE_NDOF);     // full sensor fusion

  // Header
  Serial.println("time_ms,heading_deg,roll_deg,pitch_deg,tempC,calSYS,calG,calA,calM");
}

void loop() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < 200) return;  // ~5 Hz
  last = now;

  // Euler angles (degrees): X=heading (yaw), Y=roll, Z=pitch (per Adafruit lib)
  sensors_event_t euler;
  bno.getEvent(&euler, Adafruit_BNO055::VECTOR_EULER);

  // Temperature and calibration
  int8_t tempC = bno.getTemp();
  uint8_t calSYS, calG, calA, calM;
  bno.getCalibration(&calSYS, &calG, &calA, &calM);

  // CSV line
  Serial.print(now);                    Serial.print(',');
  Serial.print(euler.orientation.x);    Serial.print(','); // heading (yaw)
  Serial.print(euler.orientation.y);    Serial.print(','); // roll
  Serial.print(euler.orientation.z);    Serial.print(','); // pitch
  Serial.print(tempC);                  Serial.print(',');
  Serial.print(calSYS);                 Serial.print(',');
  Serial.print(calG);                   Serial.print(',');
  Serial.print(calA);                   Serial.print(',');
  Serial.println(calM);
}
