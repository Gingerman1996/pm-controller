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
  static uint16_t getFanRunSpeed(float current, uint16_t target);
  static uint16_t scaleDutyCycle(const uint16_t dutyCycle);
  static float determineFanRPMToAchieveTarget(double averagePMInRoom,
                                              double targetPM,
                                              double inletPMConcentration);
  static int convertRPMToPercentage(double rpm);

 private:
  constexpr static double ROOM_VALUE_M3 = 48;
  constexpr static double ROOM_AIR_LEAK_M3H = 24;
  constexpr static double FAN_MAX_AIR_FLOW_M3H =
      183 / 2;  // reality may have only half
  constexpr static double INLET_PM25_CONCENTRATION_UGM3 = 2000;
  // Static PID Variables
  static float current_error, previous_error, integral, derivative;
  static float Kp, Ki, Kd;

  // Static function for calculating PID
  static float calculatePID(float current, uint16_t target);
  static float determineTargetPM25FlowRate(double averagePMInRoom,
                                           double targetPM, double roomVolume,
                                           double roomAirLeak);
  static float controlFanRPM(double targetPM25FlowRate,
                             double inletPMConcentration);
};

#endif