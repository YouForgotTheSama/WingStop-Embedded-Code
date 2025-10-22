/***************************************************************************
  BMP3XX (BMP388/BMP390) I2C test on RP2040
  I2C pins set explicitly to:
    SDA = GP12
    SCL = GP13
***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }
  Serial.println("Adafruit BMP388 / BMP390 test (I2C on GP12/GP13)");

  // --- I2C on GP12/GP13 ---
  Wire.setSDA(12);      // SDA -> GP12
  Wire.setSCL(13);      // SCL -> GP13
  Wire.begin();
  Wire.setClock(400000);  // optional: 400 kHz fast mode

  // Use I2C; pass address or alternate Wire if needed (default addr 0x77)
  if (!bmp.begin_I2C()) {   // e.g., bmp.begin_I2C(0x77, &Wire)
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1) { delay(10); }
  }

  // Oversampling & filter setup
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop() {
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    delay(200);
    return;
  }

  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  delay(2000);
}
