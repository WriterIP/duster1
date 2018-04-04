// http://www.instructables.com/id/How-to-Interface-With-Optical-Dust-Sensor/

#include "model.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <MHZ19_uart.h>
#include <Wire.h>
#include <SoftwareSerial.h>

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

// BOSCH BME280 set-up
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

//BT
SoftwareSerial BT(3, 4);

void setup() {
  // MH-Z19 CO2 sensor  setup
  Serial.begin(9600);
  BT.begin(9600);

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
    while (1);//todo:wait 10 seconds
  }
  Serial.println("Initted BME 280");
}

const char *mhz19Name = "MHZ19B";
Sensor readMHZ19Values() {
  Serial.println("reading from mhz19 sensor...");
  Sensor mhDTO;
  mhDTO.sensorName = mhz19Name;
  mhDTO.measurements = new Measurement[2];

  mhDTO.measurements[0].measure = co2;
  mhDTO.measurements[0].value = mhz19.getPPM();

  mhDTO.measurements[1].measure = temperature;
  mhDTO.measurements[1].value = mhz19.getTemperature();

  return mhDTO;
}

const char *bmeName = "BME";
Sensor readBmeValues() {
  Serial.println("reading from bme sensor...");

  bme.takeForcedMeasurement(); // has no effect in normal mode

  Sensor bmeDTO;
  bmeDTO.sensorName = bmeName;
  bmeDTO.measurements = new Measurement[4];

  bmeDTO.measurements[0].measure = temperature;
  bmeDTO.measurements[0].value = bme.readTemperature();

  bmeDTO.measurements[1].measure = pressure;
  bmeDTO.measurements[1].value = (bme.readPressure() / 100.0F);

  bmeDTO.measurements[2].measure = altitude;
  bmeDTO.measurements[2].value = bme.readAltitude(SEALEVELPRESSURE_HPA);

  bmeDTO.measurements[3].measure = humidity;
  bmeDTO.measurements[3].value = bme.readHumidity();

  return bmeDTO;
}

const char *dsharpName = "GP2Y10";
Sensor readDustValues() {
  Serial.println("reading from dust sensor...");

  float voMeasured = 0;
  float calcVoltage = 0;
  float dustDens = 0;

  digitalWrite(ledPower, LOW);
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(measurePin);
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower, HIGH);
  delayMicroseconds(sleepTime);

  calcVoltage = voMeasured * (5.0 / 1024);
  dustDens = 0.17 * calcVoltage - 0.1;

  Sensor sharpDTO;
  sharpDTO.sensorName = dsharpName;
  sharpDTO.measurements = new Measurement[2];
  if (dustDens < 0) {
    dustDens = 0.00;
  }

  sharpDTO.measurements[0].measure = dustRaw;
  sharpDTO.measurements[0].value = voMeasured;

  sharpDTO.measurements[1].measure = dustDensity;
  sharpDTO.measurements[1].value = dustDens;

  return sharpDTO;
}

// todo: refactor it to printsensorvalues
void printBmeValues(Sensor bmeDTO) {
  Serial.print("BOSCH BME280: Temperature = ");
  Serial.print(bmeDTO.measurements[0].value);
  Serial.print(" *C");

  Serial.print("; Pressure = ");

  Serial.print(bmeDTO.measurements[1].value);
  Serial.print(" hPa");

  Serial.print("; Approx. Altitude = ");
  Serial.print(bmeDTO.measurements[2].value);
  Serial.print(" m");

  Serial.print("; Humidity = ");
  Serial.print(bmeDTO.measurements[3].value);
  Serial.println();
}

void printMHZ19Values(Sensor mhDTO) {
  Serial.print("MHZ19: co2: ");
  Serial.print(mhDTO.measurements[0].value);
  Serial.print("; tempMH: ");
  Serial.println(mhDTO.measurements[1].value);
}

void printGP2Y10Values(Sensor sharpDTO) {
  Serial.print("SHARP GP2Y10: Raw Signal Value (0-1023): ");
  Serial.print(sharpDTO.measurements[0].value);

  // Serial.println("Voltage:");
  // Serial.println(calcVoltage);

  Serial.print("; Dust Density: ");
  Serial.println(sharpDTO.measurements[1].value);
}

// debug
extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

uint16_t getFreeSram() {
  uint8_t newVariable;
  // heap is empty, use bss as start memory address
  if ((uint16_t)__brkval == 0)
    return (((uint16_t)&newVariable) - ((uint16_t)&__bss_end));
  // use heap end as the start of the memory address
  else
    return (((uint16_t)&newVariable) - ((uint16_t)__brkval));
};
// end debug section

void printBT(const char *message) {
      BT.print(BT.available());
    // if (BT.available()){
      BT.println(message);
    // } else{
    //   Serial.println("BT unavailable");
    // }
}

void loop() {
  Serial.print("Time: ");
  Serial.println(millis());
  Serial.print("Free SRAM: ");
  Serial.println(getFreeSram());

  Serial.println("------------------------------");
  // MH-Z19 CO2 sensor  loop
  Sensor m = readMHZ19Values();
  printMHZ19Values(m);
  delete[] m.measurements;

  // dust measuring
  Sensor d = readDustValues();
  printGP2Y10Values(d);
  delete[] d.measurements;//somewhy can't move it to struct's destructor
  // bme measuring
  Sensor b = readBmeValues();
  printBmeValues(b);
  delete[] b.measurements;

  Serial.println("------------------------------");

  printBT("made measure");
  delay(2000);
}
