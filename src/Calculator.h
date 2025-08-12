#pragma once
#ifndef CALCULATOR_H
#define CALCULATOR_H

#include <array>
#include <cmath>
#include <iostream>
#include <numeric>

class Calculator {
#define MAX_DUTY_CYCLE_SCALE 511  // range is 0-511
 public:
  static float getFanRunningInterval(float current, uint16_t target);
  static float getFanRunningIntervalV2(float current, uint16_t target,
                                       uint16_t fanSpeedInPercent);
  static uint16_t getFanRunSpeed(float current, uint16_t target); // Returns PM generator speed
  static uint16_t scaleDutyCycle(const uint16_t dutyCycle);
  static int calculateInletConcentration(int targetConcentration);
  static int convertPercentageToRPM(int percent);
  static void resetPID(); // Reset PID controller state for PM generation

 private:
  constexpr static double ROOM_VALUE_M3 = 48;
  constexpr static double ROOM_AIR_LEAK_M3H = 32;
  constexpr static double FAN_MAX_AIR_FLOW_M3H =
      183 / 2;  // Max generation capacity (reality may have only half)
  constexpr static double INLET_PM25_CONCENTRATION_UGM3 = 2000;
  // Static PID Variables for PM Generation Control
  static float current_error, previous_error, integral, derivative;
  static float Kp, Ki, Kd;
  static float dt, lastTime;  // Time tracking for PID
  constexpr static double ductDiameter =
      0.1016;  // Diameter of duct in meters (4 inches)
  constexpr static double k =
      0.2516;  // Calculated k value based on the generator specs

  // Static function for calculating PID for PM generation
  static float calculatePID(float current, uint16_t target);
  static float controlFanRPM(double targetPM25FlowRate,
                             double inletPMConcentration);
  static float calculateAirFlowRate(int rpm);
};

#endif