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
  float value;
};

struct Sensor {
  const char *sensorName;
  Measurement *measurements;
  // Sensor(){}
  // ~Sensor() {
  //   // Serial.print("freeing memory " );
  //   // Serial.println(sensorName);
  //   // delete sensorName;
  // if(measurements)
  //   delete[] measurements;
  // //   //
  //   // Serial.print("freed memory " );
  //   // Serial.println(sensorName);
  //
  //   // malloc/free, new/delete, new[]/delete[]
  // }
};
