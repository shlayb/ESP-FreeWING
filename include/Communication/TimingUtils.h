#ifndef TIMING_UTILS_H
#define TIMING_UTILS_H

#include <Arduino.h>

// FreeRTOS compatible timing function
inline uint32_t getMillis() {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

// Alternative functions for timing
inline uint32_t getSeconds() {
    return getMillis() / 1000;
}

inline uint32_t getMicros() {
    return xTaskGetTickCount() * portTICK_PERIOD_MS * 1000;
}

#endif // TIMING_UTILS_H
