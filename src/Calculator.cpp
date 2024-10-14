#include "Arduino.h"
#include "Calculator.h"

// Initialize static PID variables in Calculator.cpp
float Calculator::current_error = 0;
float Calculator::previous_error = 0;
float Calculator::integral = 0;
float Calculator::derivative = 0;
float Calculator::Kp = 1.0;   // Proportional gain
float Calculator::Ki = 0.1;   // Integral gain
float Calculator::Kd = 0.01;  // Derivative gain


float Calculator::getFanRunningInterval(float current, uint16_t target) {
  // Serial.println("Use Calculation V1");
  return current < target ? pow(target - current, 0.7) * 1350 * 2 : 0;
}

float Calculator::getFanRunningIntervalV2(float startValue, uint16_t targetValue, uint16_t fanSpeedInPercent) {
  // Serial.println("Use Calculation V2");

  //  ln((C_in - C0) / (C_in - C_target)) = (Q / V) * t
  //  t = (V / Q) * ln((C_in - C0) / (C_in - C_target))

  float fanAirFlowBySpeed = FAN_MAX_AIR_FLOW_M3H * fanSpeedInPercent / 100;
  return ((ROOM_VALUE_M3 / (fanAirFlowBySpeed - ROOM_AIR_LEAK_M3H)) * log((INLET_PM25_CONCENTRATION_UGM3 - startValue) / (INLET_PM25_CONCENTRATION_UGM3 - targetValue))) * 60 * 60 * 1000;
}


uint16_t Calculator::getFanRunSpeed(float current, uint16_t target) {
  // Calculate the PID control output for adjusting the fan speed
  float fanSpeedPID = calculatePID(current, target);
  // Serial.print("Fan Speed PID: ");
  // Serial.println(fanSpeedPID);

  // Calculate the minimum fan speed percentage based on air leakage and maximum airflow
  float minSpeedPercent = (ROOM_AIR_LEAK_M3H / FAN_MAX_AIR_FLOW_M3H) * 100 * 2;

  // Constrain the fan speed within a suitable range (between minSpeedPercent and 60%)
  return fanSpeedPID;
}

uint16_t Calculator::scaleDutyCycle(const uint16_t dutyCycle) {
  return ((float)dutyCycle / 100) * MAX_DUTY_CYCLE_SCALE;
}

// Function to calculate the PID output
float Calculator::calculatePID(float current, uint16_t target) {
  // Calculate the error (difference between target and current value)
  // Serial.print("Target PM: ");
  // Serial.print(target);
  // Serial.println("μg/m³");
  // Serial.print("Cruent PM: ");
  // Serial.print(current);
  // Serial.println("μg/m³");

  // If the current dust level is greater than or equal to the target, set fan speed to 0
  if (current >= target) {
    return 0.0f;  // Turn off the fan
  }

  current_error = target - current;

  // Accumulate the error over time (Integral) and apply windup prevention
  // integral += current_error;

  // Limit the integral term to prevent it from growing too large (Anti-windup)
  // float integralMax = 50.0f;   // Adjust this value as necessary based on your system
  // float integralMin = -50.0f;  // Set a minimum value for integral
  // if (integral > integralMax) integral = integralMax;
  // if (integral < integralMin) integral = integralMin;

  // Calculate the rate of change of the error (Derivative)
  derivative = current_error - previous_error;

  // Compute the PID output using the Proportional, Integral, and Derivative components
  // float pidOutput = (Kp * current_error) + (Ki * integral) + (Kd * derivative);

  // Compute the PD output using only Proportional and Derivative components
  float pdOutput = (Kp * current_error) + (Kd * derivative);


  // Store the current error for the next calculation (for the next cycle)
  previous_error = current_error;

  // Set the fan speed limits between 20% (minimum) and 100% (maximum)
  float maxFanSpeed = 100.0f;
  float minFanSpeed = 18.0f;

  // Normalize the PID output to a range between 0 and 1
  // This assumes that the maximum error is equal to the target value
  // float normalizedOutput = pidOutput / (Kp * target);

  // Normalize the PD output to a range between 0 and 1
  float normalizedOutput = pdOutput / (Kp * target);

  // Scale the normalized output to fit within the range from minFanSpeed to maxFanSpeed
  float fanSpeed = minFanSpeed + (normalizedOutput * (maxFanSpeed - minFanSpeed));

  // Ensure the fan speed is not lower than the minimum speed and not higher than the maximum speed
  if (fanSpeed < minFanSpeed) fanSpeed = minFanSpeed;
  if (fanSpeed > maxFanSpeed) fanSpeed = maxFanSpeed;

  // Return the calculated fan speed
  // Serial.print("fanSpeed from PID functuion: ");
  // Serial.println(fanSpeed);
  return fanSpeed;
}
