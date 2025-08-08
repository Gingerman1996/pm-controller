#include "Calculator.h"

#include "Arduino.h"

// Initialize static PID variables in Calculator.cpp
float Calculator::current_error = 0;
float Calculator::previous_error = 0;
float Calculator::integral = 0;
float Calculator::derivative = 0;
float Calculator::Kp = 0.15; // Proportional gain - reduced for stability
float Calculator::Ki = 0.005; // Integral gain - reduced to prevent windup
float Calculator::Kd = 0.25;  // Derivative gain - increased for better damping
float Calculator::dt = 1.0;   // Time step in seconds
float Calculator::lastTime = 0; // Last time PID was calculated

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
  // Get current time in milliseconds
  float currentTime = millis() / 1000.0; // Convert to seconds
  
  // Calculate time difference since last PID calculation
  if (lastTime == 0) {
    lastTime = currentTime;
    dt = 1.0; // Default time step for first calculation
  } else {
    dt = currentTime - lastTime;
    lastTime = currentTime;
  }
  
  // Ensure minimum time step to prevent division by zero
  if (dt <= 0.01) dt = 0.01;
  
  // Calculate the error (difference between target and current value)
  // If the current dust level is greater than or equal to the target, set fan speed to 0
  if (current >= target) {
    // Reset PID terms when target is reached to prevent integral windup
    integral = 0;
    previous_error = 0;
    return 0.0f; // Turn off the fan
  }

  current_error = target - current;

  // Proportional term with deadband to reduce noise
  float proportionalTerm = Kp * current_error;
  
  // Integral term with anti-windup protection
  integral += current_error * dt;
  
  float maxFanSpeed;
  float minFanSpeed;
  float integralMax;
  float integralMin;
  
  // Set the fan speed limits and integral limits based on target concentration
  if (target > 20) {
    maxFanSpeed = 50.0f;
    minFanSpeed = 25.0f; // Reduced minimum for better control
    integralMax = 10.0f; // Increased for better steady-state accuracy
    integralMin = -10.0f;
  } else {
    maxFanSpeed = 40.0f;
    minFanSpeed = 20.0f; // Reduced minimum for better control
    integralMax = 8.0f;
    integralMin = -8.0f;
  }

  // Anti-windup: Limit the integral term
  if (integral > integralMax) integral = integralMax;
  if (integral < integralMin) integral = integralMin;

  float integralTerm = Ki * integral;

  // Derivative term with filtering to reduce noise
  derivative = (current_error - previous_error) / dt;
  
  // Apply low-pass filter to derivative to reduce noise
  static float filtered_derivative = 0;
  float alpha = 0.1; // Low-pass filter coefficient (0-1, lower = more filtering)
  filtered_derivative = alpha * derivative + (1 - alpha) * filtered_derivative;
  
  float derivativeTerm = Kd * filtered_derivative;

  // Compute the PID output
  float pidOutput = proportionalTerm + integralTerm + derivativeTerm;

  // Store the current error for the next calculation (for the next cycle)
  previous_error = current_error;

  // Improved output scaling with better normalization
  // Use a more conservative approach to avoid overshooting
  float outputScale = 1.0f / (Kp * target * 0.5f); // More conservative scaling
  float normalizedOutput = pidOutput * outputScale;
  
  // Apply saturation limits
  if (normalizedOutput < 0.0f) normalizedOutput = 0.0f;
  if (normalizedOutput > 1.0f) normalizedOutput = 1.0f;

  // Scale the normalized output to fit within the range from minFanSpeed to maxFanSpeed
  // Use a smooth transition curve for better control
  float fanSpeed = minFanSpeed + (normalizedOutput * (maxFanSpeed - minFanSpeed));

  // Apply deadband near the target to reduce oscillation
  float errorPercent = abs(current_error) / target;
  if (errorPercent < 0.05) { // 5% deadband
    // Reduce fan speed gradually when close to target
    fanSpeed *= (errorPercent / 0.05);
  }

  // Ensure the fan speed is within limits
  if (fanSpeed < minFanSpeed) fanSpeed = minFanSpeed;
  if (fanSpeed > maxFanSpeed) fanSpeed = maxFanSpeed;

  // Add rate limiting to prevent sudden changes
  static float lastFanSpeed = 0;
  float maxChangeRate = 5.0f; // Maximum change per second (%)
  float maxChange = maxChangeRate * dt;
  
  if (abs(fanSpeed - lastFanSpeed) > maxChange) {
    if (fanSpeed > lastFanSpeed) {
      fanSpeed = lastFanSpeed + maxChange;
    } else {
      fanSpeed = lastFanSpeed - maxChange;
    }
  }
  lastFanSpeed = fanSpeed;

  // Return the calculated fan speed
  return fanSpeed;
}

// Function to control fan RPM based on target PM2.5 flow rate
float Calculator::controlFanRPM(double targetPM25FlowRate,
                                double inletPMConcentration) {
  // Constants
  constexpr double ductDiameter =
      0.1016; // Diameter of duct in meters (4 inches)
  constexpr double roomAirLeak =
      ROOM_AIR_LEAK_M3H /
      60; // Room air leak rate in m3 per minute (converted from m3/h)

  // Step 1: Calculate the cross-sectional area of the duct (A = π * (D/2)^2)
  double area = M_PI * std::pow(ductDiameter / 2.0, 2);

  // Step 2: Calculate the required air flow rate to meet the target PM2.5 flow
  // rate
  double requiredAirFlowRate =
      targetPM25FlowRate /
      inletPMConcentration; // Required air flow rate in m3/min

  // Step 3: Subtract room air leakage from the required air flow rate
  double netAirFlowRate = requiredAirFlowRate - roomAirLeak;

  // Step 4: Calculate the required air velocity (V = Q / A)
  double requiredVelocity =
      netAirFlowRate / area; // Required velocity in meters per minute

  // Step 5: Calculate the required RPM (RPM = V / k)
  constexpr double k = 0.2516; // Calculated k value based on the fan specs
  double requiredRPM = requiredVelocity / k; // Required RPM for the fan

  return requiredRPM;
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
  double velocity = k * rpm; // Velocity in meters per minute

  // Step 3: Calculate air flow rate (Q_air = A * V)
  double airFlowRate = area * velocity; // Air flow rate in m3/min

  return airFlowRate; // Return the calculated air flow rate in m3/min
}

int Calculator::calculateInletConcentration(int targetConcentration) {
  // Step 1: Convert room air leak rate from m3/h to m3/min
  double roomAirLeakRateMin =
      ROOM_AIR_LEAK_M3H / 60.0; // Convert from m³/h to m³/min

  // Step 2: Calculate the airflow rate using the RPM value
  // if (rpm < 0) {
  //   Serial.println("Error: RPM must be non-negative.");
  //   return -1;  // Return an error value if rpm is invalid
  // }
  float airFlowRate = calculateAirFlowRate(convertPercentageToRPM(1));

  // Check if airflow rate is greater than zero to prevent division by zero
  if (airFlowRate <= 0) {
    Serial.println("Error: Air flow rate must be greater than zero.");
    return -1; // Return an error value if air flow rate is invalid
  }

  // Step 3: Calculate air exchange rate based on room volume
  double airExchangeRate =
      airFlowRate / ROOM_VALUE_M3; // Number of air exchanges per minute

  // Step 4: Calculate the required inlet concentration considering dilution
  if (targetConcentration < 0) {
    Serial.println("Error: Target concentration must be non-negative.");
    return -1; // Return an error value if target concentration is invalid
  }

  // Use room volume and air exchange rate to determine the dilution factor
  double dilutionFactor =
      1.0 + airExchangeRate; // More exchange results in more dilution
  double inletConcentration = (targetConcentration * roomAirLeakRateMin) /
                              (airFlowRate * dilutionFactor);

  return inletConcentration; // Return the calculated inlet concentration in
                             // µg/m³
}

// Reset PID controller state - useful when changing targets or restarting
void Calculator::resetPID() {
  current_error = 0;
  previous_error = 0;
  integral = 0;
  derivative = 0;
  lastTime = 0;
}