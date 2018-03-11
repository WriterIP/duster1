// http://www.instructables.com/id/How-to-Interface-With-Optical-Dust-Sensor/

#include "model.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <MHZ19_uart.h>
#include <Wire.h>

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
    while (1)
      ;
  }
  Serial.println("Initted BME 280");
}

Sensor readMHZ19Values() {
  Sensor mhDTO;
  Measurement ms[2];

  mhDTO.sensorName = "MHZ19B";
  mhDTO.measurements = ms;

  mhDTO.measurements[0].measure = co2;
  mhDTO.measurements[0].value = mhz19.getPPM();

  mhDTO.measurements[1].measure = temperature;
  mhDTO.measurements[1].value = mhz19.getTemperature();

  return mhDTO;
}

Sensor readBmeValues() {
  Sensor bmeDTO;
  Measurement ms[4];

  bmeDTO.sensorName = "BME";
  bmeDTO.measurements = ms;

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

Sensor readDustValues(){
  Sensor sharpDTO;

  return sharpDTO;
}

//todo: refactor it to printsensorvalues
void printBmeValues(Sensor bmeDTO){
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

void printMHZ19Values(Sensor mhDTO){
  Serial.print("MHZ19: co2: ");
  Serial.print(mhDTO.measurements[0].value);
  Serial.print("; tempMH: ");
  Serial.println(mhDTO.measurements[1].value);
}

//debug
int i = 0;

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
//end debug section
void loop() {
  Serial.println(i++);
  Serial.println("------------------------------");
  // MH-Z19 CO2 sensor  loop
  Sensor m =  readMHZ19Values();
  printMHZ19Values(m);

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
  Serial.println(dustDens);

  // bme.takeForcedMeasurement(); // has no effect in normal mode

  Sensor b =  readBmeValues();
  printBmeValues(b);
  Serial.println("------------------------------");
  Serial.println(getFreeSram());
  delay(2000);
}
