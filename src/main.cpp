// http://www.instructables.com/id/How-to-Interface-With-Optical-Dust-Sensor/

#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <MHZ19_uart.h>
#include <Wire.h>
#include "model.h"

// MHZ19 set-up
const int rx_pin = 7; // Serial rx pin no for co2
const int tx_pin = 6; // Serial tx pin no for co2

MHZ19_uart mhz19;

// SHARP GP2Y10 14 dust sensor set-up
const int measurePin = A3;
const int ledPower = 12;

const unsigned int samplingTime = 280;
const unsigned int deltaTime = 40;
const unsigned int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDens = 0;

// BOSCH BME280 set-up
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

void setup() {
  // MH-Z19 CO2 sensor  setup
  Serial.begin(9600);
  Serial.println("Initting MH-Z19B");
  mhz19.begin(rx_pin, tx_pin);
  mhz19.setAutoCalibration(false);
  // while( mhz19.isWarming() ) {
  Serial.print("MH-Z19 now warming up...  status:");
  Serial.println(mhz19.getStatus());
  delay(1000);
  //  }
  Serial.println("Initted MH-Z19B");

  // Sharp dust sensor setup
  Serial.println("Initting SHARP GP2Y10");
  pinMode(ledPower, OUTPUT);
  Serial.println("Initted SHARP GP2Y10");

  // BME 280
  Serial.println("Initting BME 280");
  bool st = bme.begin(0x76);
  Serial.println(st);
  if (!st) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  Serial.println("Initted BME 280");

}

Sensor printBmeValues() {
  Sensor bmeDTO;

  bmeDTO.sensorName = "BOSCH_BME280";
  bmeDTO.measurements={}//= Measurement[3];
  bmeDTO.measurements[1].measure = Measure.temperature;

  Serial.print("BOSCH BME280: Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.print(" *C");

  Serial.print("; Pressure = ");

  Serial.print(bme.readPressure() / 100.0F);
  Serial.print(" hPa");

  Serial.print("; Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.print(" m");

  Serial.print("; Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println();

  return bmeDTO;
}

void loop() {
  Serial.println("------------------------------");
  // MH-Z19 CO2 sensor  loop
  int co2ppm = mhz19.getPPM();
  int temp = mhz19.getTemperature();

  Serial.print("MHZ19: co2: ");
  Serial.print(co2ppm);
  Serial.print("; tempMH: ");
  Serial.println(temp);

  // dust measuring
  digitalWrite(ledPower, LOW);
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(measurePin);

  delayMicroseconds(deltaTime);
  digitalWrite(ledPower, HIGH);
  delayMicroseconds(sleepTime);

  calcVoltage = voMeasured * (5.0 / 1024);
  dustDens = 0.17 * calcVoltage - 0.1;

  if (dustDens < 0) {
    dustDens = 0.00;
  }

  Serial.print("SHARP GP2Y10: Raw Signal Value (0-1023): ");
  Serial.print(voMeasured);

  // Serial.println("Voltage:");
  // Serial.println(calcVoltage);

  Serial.print("; Dust Density: ");
  Serial.println(dustDensity);

  bme.takeForcedMeasurement(); // has no effect in normal mode

  printBmeValues();
  Serial.println("------------------------------");

  delay(1000);
}
