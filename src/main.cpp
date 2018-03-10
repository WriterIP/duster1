#include <Arduino.h>
// http://www.instructables.com/id/How-to-Interface-With-Optical-Dust-Sensor/
#include <MHZ19_uart.h>

// MHZ19 set-up
const int rx_pin = 7; // Serial rx pin no for co2
const int tx_pin = 6; // Serial tx pin no for co2

MHZ19_uart mhz19;

// SHARP dust sensor set-up
const int measurePin = A5;
const int ledPower = 12;

const unsigned int samplingTime = 280;
const unsigned int deltaTime = 40;
const unsigned int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

void setup() {
  // MH-Z19 CO2 sensor  setup
  Serial.begin(9600);
  mhz19.begin(rx_pin, tx_pin);
  mhz19.setAutoCalibration(false);
  // while( mhz19.isWarming() ) {
  Serial.print("MH-Z19 now warming up...  status:");
  Serial.println(mhz19.getStatus());
  delay(1000);
  //  }
  // Sharp dust sensor setup
  pinMode(ledPower, OUTPUT);
}

void loop() {
  //    MH-Z19 CO2 sensor  loop
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
  dustDensity = 0.17 * calcVoltage - 0.1;

  if (dustDensity < 0) {
    dustDensity = 0.00;
  }

  Serial.print("SHARP GP2Y10: Raw Signal Value (0-1023): ");
  Serial.print(voMeasured);

  // Serial.println("Voltage:");
  // Serial.println(calcVoltage);

  Serial.print("; Dust Density: ");
  Serial.println(dustDensity);
  delay(5000);
}
