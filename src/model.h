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
  Measurement measurements[];
  ~Sensor() {
    delete sensorName;
    // delete measurements;
  }
};
