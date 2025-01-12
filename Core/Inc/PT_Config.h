#ifndef SENSORCONTROLLERFIRMWARE_PT_CONFIG_H
#define SENSORCONTROLLERFIRMWARE_PT_CONFIG_H

#include <stdint-gcc.h>

typedef struct {
  // Uncalibrated
  int16_t max_pressure;
  int16_t min_pressure;
  float max_voltage;
  float min_voltage;
  float conversion;

  // Calibrated
  float m;
  float b;
} PT_Config;

typedef struct {
  float max_load;
  float sensitivity;
  float excitation;
} LC_Config;

#endif //SENSORCONTROLLERFIRMWARE_PT_CONFIG_H
