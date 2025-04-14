// sensor_logger.c

#include "sensor_logger.h"

static DHT22_LogEntry dht22_log[MAX_CO_LOG_ENTRIES];
static MQ135_LogEntry mq135_log[MAX_CO_LOG_ENTRIES];
static BME688_LogEntry bme688_log[MAX_LOG_ENTRIES];

static uint16_t dht22_index = 0;
static uint16_t mq135_index = 0;
static uint16_t bme688_index = 0;

void init_sensor_loggers(void) {
    dht22_index = mq135_index = bme688_index = 0;
}

void log_dht22_data(float temperature, float humidity, uint32_t timestamp) {
    dht22_log[dht22_index] = (DHT22_LogEntry){timestamp, temperature, humidity};
    dht22_index = (dht22_index + 1) % MAX_CO_LOG_ENTRIES;
}

void log_mq135_data(float co, float no2, uint32_t timestamp) {
    mq135_log[mq135_index] = (MQ135_LogEntry){timestamp, co, no2};
    mq135_index = (mq135_index + 1) % MAX_CO_LOG_ENTRIES;
}

void log_bme688_data(float pm25, float co, float no2, float temp, float hum, uint32_t timestamp) {
    bme688_log[bme688_index] = (BME688_LogEntry){timestamp, pm25, co, no2, temp, hum};
    bme688_index = (bme688_index + 1) % MAX_LOG_ENTRIES;
}
