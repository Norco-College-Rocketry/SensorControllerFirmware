#ifndef SENSORCONTROLLERFIRMWARE_CONVERSIONS_H
#define SENSORCONTROLLERFIRMWARE_CONVERSIONS_H

/* Voltage to sensor conversion functions
 *
 */

#include <stdint-gcc.h>
#include "PT_Config.h"

// Convert load cell voltage to load in units given by lc.max_load and lc.sensitivity
float convert_weight(float voltage, LC_Config* lc) {
  return lc->max_load * voltage / (lc->sensitivity * lc->excitation);
}

float convert_pressure(float voltage, PT_Config *pt) {
  return (voltage - pt->min_voltage) * (pt->max_pressure - pt->min_pressure) / (pt->max_voltage - pt->min_voltage); // Convert to pressure
}

float convert_pressure_calibrated(float voltage, PT_Config *pt) { return pt->m * voltage + pt->b; }

// Converts K Type thermocouple voltage to temperature in Celsius.
float convert_thermocouple_K(float voltage) {
  return voltage * (100 / (4.096f/1000) );
}

#endif //SENSORCONTROLLERFIRMWARE_CONVERSIONS_H
