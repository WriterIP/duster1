#include <Arduino.h>

enum Measure {
  temperature,
  co2,
  dustRaw,
  dustDensity,
  pressure,
  altitude,
  humidity
};

struct Measurement {
  Measure measure;
  double value;
};

struct Sensor {
  const char *sensorName;
  Measurement *measurements;
  ~Sensor() {
    Serial.print("freeing memory " );
    Serial.println(sensorName);
    // delete[] sensorName;
    // delete[] measurements;
    // malloc/free, new/delete, new[]/delete[]
  }
};
