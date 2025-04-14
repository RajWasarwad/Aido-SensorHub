// aqi.h
#ifndef AQI_H
#define AQI_H

#include <stdint.h>
#include "sensor_logger.h" // Include your sensor log definitions here

void update_aqi(void);
float get_current_aqi(void);
const char* get_aqi_category(float aqi);

#endif // AQI_H
