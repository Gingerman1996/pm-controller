#include "Calculator.h"

#include "Arduino.h"

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

float Calculator::getFanRunningIntervalV2(float startValue,
                                          uint16_t targetValue,
                                          uint16_t fanSpeedInPercent) {
  // Serial.println("Use Calculation V2");

  //  ln((C_in - C0) / (C_in - C_target)) = (Q / V) * t
  //  t = (V / Q) * ln((C_in - C0) / (C_in - C_target))

  float fanAirFlowBySpeed = FAN_MAX_AIR_FLOW_M3H * fanSpeedInPercent / 100;
  return ((ROOM_VALUE_M3 / (fanAirFlowBySpeed - ROOM_AIR_LEAK_M3H)) *
          log((INLET_PM25_CONCENTRATION_UGM3 - startValue) /
              (INLET_PM25_CONCENTRATION_UGM3 - targetValue))) *
         60 * 60 * 1000;
}

uint16_t Calculator::getFanRunSpeed(float current, uint16_t target) {
  // Calculate the PID control output for adjusting the fan speed
  float fanSpeedPID = calculatePID(current, target);
  // Serial.print("Fan Speed PID: ");
  // Serial.println(fanSpeedPID);

  // Calculate the minimum fan speed percentage based on air leakage and maximum
  // airflow
  float minSpeedPercent = (ROOM_AIR_LEAK_M3H / FAN_MAX_AIR_FLOW_M3H) * 100 * 2;

  // Constrain the fan speed within a suitable range (between minSpeedPercent
  // and 60%)
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

  // If the current dust level is greater than or equal to the target, set fan
  // speed to 0
  if (current >= target) {
    return 0.0f;  // Turn off the fan
  }

  current_error = target - current;

  // Accumulate the error over time (Integral) and apply windup prevention
  // integral += current_error;

  // Limit the integral term to prevent it from growing too large (Anti-windup)
  // float integralMax = 50.0f;   // Adjust this value as necessary based on
  // your system float integralMin = -50.0f;  // Set a minimum value for
  // integral if (integral > integralMax) integral = integralMax; if (integral <
  // integralMin) integral = integralMin;

  // Calculate the rate of change of the error (Derivative)
  derivative = current_error - previous_error;

  // Compute the PID output using the Proportional, Integral, and Derivative
  // components float pidOutput = (Kp * current_error) + (Ki * integral) + (Kd *
  // derivative);

  // Compute the PD output using only Proportional and Derivative components
  float pdOutput = (Kp * current_error) + (Kd * derivative);

  // Store the current error for the next calculation (for the next cycle)
  previous_error = current_error;

  // Set the fan speed limits between 20% (minimum) and 100% (maximum)
  float maxFanSpeed = 70.0f;
  float minFanSpeed = 20.0f;

  // Normalize the PID output to a range between 0 and 1
  // This assumes that the maximum error is equal to the target value
  // float normalizedOutput = pidOutput / (Kp * target);

  // Normalize the PD output to a range between 0 and 1
  float normalizedOutput = pdOutput / (Kp * target);

  // Scale the normalized output to fit within the range from minFanSpeed to
  // maxFanSpeed
  float fanSpeed =
      minFanSpeed + (normalizedOutput * (maxFanSpeed - minFanSpeed));

  // Ensure the fan speed is not lower than the minimum speed and not higher
  // than the maximum speed
  if (fanSpeed < minFanSpeed) fanSpeed = minFanSpeed;
  if (fanSpeed > maxFanSpeed) fanSpeed = maxFanSpeed;

  // Return the calculated fan speed
  // Serial.print("fanSpeed from PID functuion: ");
  // Serial.println(fanSpeed);
  return fanSpeed;
}

// Function to control fan RPM based on target PM2.5 flow rate
float Calculator::controlFanRPM(double targetPM25FlowRate,
                                double inletPMConcentration) {
  // Constants
  constexpr double ductDiameter =
      0.1016;  // Diameter of duct in meters (4 inches)
  constexpr double roomAirLeak =
      ROOM_AIR_LEAK_M3H /
      60;  // Room air leak rate in m3 per minute (converted from m3/h)

  // Step 1: Calculate the cross-sectional area of the duct (A = π * (D/2)^2)
  double area = M_PI * std::pow(ductDiameter / 2.0, 2);

  // Step 2: Calculate the required air flow rate to meet the target PM2.5 flow
  // rate
  double requiredAirFlowRate =
      targetPM25FlowRate /
      inletPMConcentration;  // Required air flow rate in m3/min

  // Step 3: Subtract room air leakage from the required air flow rate
  double netAirFlowRate = requiredAirFlowRate - roomAirLeak;

  // Step 4: Calculate the required air velocity (V = Q / A)
  double requiredVelocity =
      netAirFlowRate / area;  // Required velocity in meters per minute

  // Step 5: Calculate the required RPM (RPM = V / k)
  constexpr double k = 0.2516;  // Calculated k value based on the fan specs
  double requiredRPM = requiredVelocity / k;  // Required RPM for the fan

  return requiredRPM;
}

// Function to determine the PM2.5 flow rate needed to achieve target PM2.5 in
// the room, considering room size and air leakage
float Calculator::determineTargetPM25FlowRate(double averagePMInRoom,
                                              double targetPM,
                                              double roomVolume,
                                              double roomAirLeak) {
  if (averagePMInRoom < targetPM) {
    double requiredIncrease =
        targetPM -
        averagePMInRoom;  // Amount of PM2.5 needed to reach target (µg/m3)
    double requiredFlowRate =
        (requiredIncrease * roomVolume) +
        roomAirLeak;  // Calculate flow rate needed considering room volume and
                      // air leakage
    return requiredFlowRate;  // Required PM2.5 flow rate in µg/min
  }
  return 0.0;  // No additional PM2.5 needed if already at or above target
}

// Updated function to determine fan RPM based on target PM2.5 flow rate
float Calculator::determineFanRPMToAchieveTarget(double averagePMInRoom,
                                                 double targetPM,
                                                 double inletPMConcentration) {
  // Step 1: Determine the target PM2.5 flow rate needed to reach target PM2.5
  // in the room
  double targetPM25FlowRate = determineTargetPM25FlowRate(
      averagePMInRoom, targetPM, ROOM_VALUE_M3, ROOM_AIR_LEAK_M3H / 60);

  // Step 2: If target PM2.5 flow rate is needed, calculate the required fan RPM
  double requiredRPM = 0.0;
  if (targetPM25FlowRate > 0.0) {
    requiredRPM = controlFanRPM(targetPM25FlowRate, inletPMConcentration);
  }

  if (requiredRPM > 3000) {
    return 3000;
  } else {
    return requiredRPM;
  }
}

// Function to calculate the average PM2.5 concentration in the room from
// multiple sensors
int Calculator::convertRPMToPercentage(double rpm) {
  constexpr double maxRPM = 3000.0;  // Maximum RPM of the fan
  int percent = (rpm / maxRPM) * 100.0;
  if (percent > 60) {
    return 60;
  } else {
    return percent;
  }
}

int Calculator::convertPercentageToRPM(int percent) {
  constexpr double maxRPM = 3000.0;
  int rpm = (percent / 100.0) * maxRPM;
  return rpm;
}

float Calculator::calculateAirFlowRate(int rpm) {
  // Step 1: Calculate the cross-sectional area of the duct (A = π * (D/2)^2)
  double area = M_PI * std::pow(ductDiameter / 2.0, 2);

  // Step 2: Calculate air velocity (V = k * RPM)
  double velocity = k * rpm;  // Velocity in meters per minute

  // Step 3: Calculate air flow rate (Q_air = A * V)
  double airFlowRate = area * velocity;  // Air flow rate in m3/min

  return airFlowRate;  // Return the calculated air flow rate in m3/min
}

int Calculator::calculateInletConcentration(int targetConcentration, int rpm) {
  // Step 1: Convert room air leak rate from m3/h to m3/min
  double roomAirLeakRateMin = ROOM_AIR_LEAK_M3H / 60.0;

  float airFlowRate = calculateAirFlowRate(rpm);

  // Step 2: Calculate the required inlet concentration (C_inlet =
  // (targetConcentration * roomAirLeakRate) / airFlowRate)
  if (airFlowRate <= 0) {
    Serial.println("Error: Air flow rate must be greater than zero.");
    return -1;  // Return an error value if air flow rate is invalid
  }
  double inletConcentration =
      (targetConcentration * roomAirLeakRateMin) / airFlowRate;

  return inletConcentration;  // Return the calculated inlet concentration in
                              // µg/m³
}