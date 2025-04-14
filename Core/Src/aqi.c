// aqi.c
#include "aqi.h"

static float current_aqi = 0;

// --- AQI Breakpoint Struct ---
typedef struct {
    float conc_low;
    float conc_high;
    int aqi_low;
    int aqi_high;
} aqi_range_t;

// --- PM2.5 Breakpoints (µg/m³) ---
static const aqi_range_t pm25_ranges[] = {
    {0, 30, 0, 50},
    {31, 60, 51, 100},
    {61, 90, 101, 200},
    {91, 120, 201, 300},
    {121, 250, 301, 400},
    {251, 500, 401, 500}
};

// --- CO Breakpoints (mg/m³) ---
static const aqi_range_t co_ranges[] = {
    {0, 1, 0, 50},
    {1.1, 2, 51, 100},
    {2.1, 10, 101, 200},
    {10.1, 17, 201, 300},
    {17.1, 34, 301, 400},
    {34.1, 50, 401, 500}
};

// --- NO2 Breakpoints (µg/m³) ---
static const aqi_range_t no2_ranges[] = {
    {0, 40, 0, 50},
    {41, 80, 51, 100},
    {81, 180, 101, 200},
    {181, 280, 201, 300},
    {281, 400, 301, 400},
    {401, 500, 401, 500}
};

static float calc_sub_index(float conc, const aqi_range_t *table, int size) {
    for (int i = 0; i < size; i++) {
        if (conc >= table[i].conc_low && conc <= table[i].conc_high) {
            float sub_aqi = ((table[i].aqi_high - table[i].aqi_low) /
                            (table[i].conc_high - table[i].conc_low)) *
                            (conc - table[i].conc_low) + table[i].aqi_low;
            return sub_aqi;
        }
    }
    return 500; // Max AQI if beyond all ranges
}

// --- Main AQI Update Function ---
void update_aqi(void) {
    float pm25 = get_average_recent(SENSOR_PM25, 60);  // 60 = last 60 mins (24h = 1440 max)
    float co   = get_average_recent(SENSOR_CO, 30);    // last 8h (if logging every 16 mins)
    float no2  = get_average_recent(SENSOR_NO2, 30);

    float aqi_pm25 = calc_sub_index(pm25, pm25_ranges, sizeof(pm25_ranges)/sizeof(aqi_range_t));
    float aqi_co   = calc_sub_index(co,   co_ranges,   sizeof(co_ranges)/sizeof(aqi_range_t));
    float aqi_no2  = calc_sub_index(no2,  no2_ranges,  sizeof(no2_ranges)/sizeof(aqi_range_t));

    current_aqi = fmaxf(aqi_pm25, fmaxf(aqi_co, aqi_no2));
}

float get_current_aqi(void) {
    return current_aqi;
}

const char* get_aqi_category(float aqi) {
    if (aqi <= 50)   return "Good";
    if (aqi <= 100)  return "Satisfactory";
    if (aqi <= 200)  return "Moderate";
    if (aqi <= 300)  return "Poor";
    if (aqi <= 400)  return "Very Poor";
    return "Severe";
}
