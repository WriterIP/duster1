// http://www.instructables.com/id/How-to-Interface-With-Optical-Dust-Sensor/

#include "model.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <MHZ19_uart.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// MHZ19 set-up
const int rx_pin = 7; // Serial rx pin no for co2
const int tx_pin = 6; // Serial tx pin no for co2

MHZ19_uart mhz19;

// BOSCH BME280 set-up
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

// BT
SoftwareSerial BT(3, 4);

void sendCommand(const char *command) {
  Serial.print("Command send :");
  Serial.println(command);
  BT.println(command);
  // wait some time
  delay(100);

  char reply[100];
  int i = 0;
  while (BT.available()) {
    reply[i] = BT.read();
    i += 1;
  }
  // end the string
  reply[i] = '\0';
  Serial.print(reply);
  Serial.println("Reply end");
  delay(50);
}

void setup() {
  Serial.begin(9600);

  Serial.println("Initting BT");
  BT.begin(9600);
  sendCommand("AT");
  sendCommand("AT+ROLE0");
  sendCommand("AT+UUID0xFFE0");
  sendCommand("AT+CHAR0xFFE1");
  sendCommand("AT+NAMEAirmonitor");

  Serial.println("Initting MH-Z19B");
  mhz19.begin(rx_pin, tx_pin);
  mhz19.setAutoCalibration(false);
  // while( mhz19.isWarming() ) {
  Serial.print("MH-Z19 now warming up...  status:");
  Serial.println(mhz19.getStatus());
  delay(1000);
  //  }
  Serial.println("Initted MH-Z19B");

  // BME 280
  Serial.println("Initting BME 280");
  bool st = bme.begin(0x76);
  Serial.println(st);
  int jj = 0;
  while (jj < 10) {
    if (!st) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      jj++;
      delay(500);
    } else {
      Serial.println("Initted BME 280");
      break;
    }
  }
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

void printDataBt(Sensor mhDTO, Sensor bmeDTO) {
  String buf;
  buf += String(mhDTO.measurements[0].value, 0);
  buf += F(",");
  buf += String(bmeDTO.measurements[0].value, 1);
  buf += F(",");
  buf += String(bmeDTO.measurements[1].value, 0);
  buf += F(",");
  buf += String(bmeDTO.measurements[3].value, 2);
  char * cstr = new char [buf.length()+1];
  strcpy(cstr, buf.c_str());
  Serial.print(buf.length()+1);
  Serial.println(cstr);
  BT.write(cstr);
  delete[] cstr;

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

void loop() {
  Serial.print("Time: ");
  Serial.println(millis());
  Serial.print("Free SRAM: ");
  Serial.println(getFreeSram());

  Serial.println("------------------------------");
  // MH-Z19 CO2 sensor  loop
  Sensor m = readMHZ19Values();
  printMHZ19Values(m);

  // bme measuring
  Sensor b = readBmeValues();
  printBmeValues(b);

  printDataBt(m, b);

  delete[] m.measurements;
  delete[] b.measurements;

  Serial.println("------------------------------");

  delay(2000);
}
