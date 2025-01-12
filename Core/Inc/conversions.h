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
  float res = 0.0f;

// See https://its90.nist.gov/InvFunctions
  float coeff[] = {
    0.0f,
    2.5173462e01f,
    -1.1662878f,
    -1.0833638f,
    -8.9773540e-01f,
    -3.7342377e-01f,
    -8.6632643e-02f,
    -1.0450598e-02f,
    -5.1920577e-04f,
    0.0f
  };
  if (voltage >= 0 && voltage < 20.644e-3) {
    coeff[0] = 0.0f;
    coeff[1] = 2.508355e01f;
    coeff[2] = 7.860106e-02f;
    coeff[3] = -2.503131e-01f;
    coeff[4] = 8.315270e-02f;
    coeff[5] = -1.228034e-02f;
    coeff[6] = 9.804036e-04f;
    coeff[7] = -4.413030e-5f;
    coeff[8] = 1.057734e-06f;
    coeff[9] = -1.052755e-08f;
  } else if (voltage >= 20.644e-3 && voltage < 54.886e-3) {
    coeff[0] = -1.318058e02f;
    coeff[1] = 4.830222e01f;
    coeff[2] = -1.646031e00f;
    coeff[3] = 5.464731e-02f;
    coeff[4] = -9.650715e-04f;
    coeff[5] = 8.802193e-06f;
    coeff[6] = -3.110810e-08f;
    coeff[7] = 0.0f;
    coeff[8] = 0.0f;
    coeff[9] = 0.0f;
  }

  // Calculate polynomial
  res = coeff[0];
  for (uint8_t i=1; i<9; i++) {
    res += coeff[i] * voltage;
    voltage *= voltage;
  }

  return res;
}

#endif //SENSORCONTROLLERFIRMWARE_CONVERSIONS_H
