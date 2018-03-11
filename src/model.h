enum measure {
  temperature,
  co2,
  dustRaw,
  dustDensity,
  pressure,
  altitude,
  humidity
};

struct sensor{
  char* sensorName;
  measure* measures;
};
