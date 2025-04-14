// sensor_logger.h

#ifndef SENSOR_LOGGER_H
#define SENSOR_LOGGER_H

#include "main.h"
#include <stdint.h>

#define MAX_LOG_ENTRIES 288  // For 24 hours at 5-minute intervals (288 = 24*60/5)
#define MAX_CO_LOG_ENTRIES 96  // For 8 hours at 5-minute intervals

typedef struct {
    uint32_t timestamp;  // System time or RTC
    float temperature;
    float humidity;
} DHT22_LogEntry;

typedef struct {
    uint32_t timestamp;
    float co;     // Carbon monoxide level
    float no2;    // Nitrogen dioxide level
} MQ135_LogEntry;

typedef struct {
    uint32_t timestamp;
    float pm25;
    float co;
    float no2;
    float temperature;
    float humidity;
} BME688_LogEntry;

void init_sensor_loggers(void);
void log_dht22_data(float temperature, float humidity, uint32_t timestamp);
void log_mq135_data(float co, float no2, uint32_t timestamp);
void log_bme688_data(float pm25, float co, float no2, float temp, float hum, uint32_t timestamp);

#endif
