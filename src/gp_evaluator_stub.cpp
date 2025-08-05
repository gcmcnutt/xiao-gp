#include <GP/autoc/gp_evaluator.h>
#include <GP/autoc/aircraft_state.h>
#include <math.h>

// Stub implementation of GP evaluator for compilation
// This will be replaced by the actual evolved GP program
double evaluateGPSimple(AircraftState& aircraftState, const Path& currentPath, double arg) {
  
  // Simple stub controller that just tries to point toward the target
  // This demonstrates the interface - the actual GP program will be more sophisticated
  
  Eigen::Vector3d current_pos = aircraftState.getPosition();
  Eigen::Vector3d target_pos = currentPath.start;
  
  // Calculate direction to target
  Eigen::Vector3d direction = target_pos - current_pos;
  double distance = direction.norm();
  
  if (distance < 0.1) {
    // Very close to target, maintain current attitude
    return 0.0;
  }
  
  direction.normalize();
  
  // Simple proportional control toward target
  double roll_error = direction[1];  // East component for roll
  double pitch_error = -direction[0]; // North component for pitch (negative because positive pitch is up)
  
  // Set control commands (clamped to [-1, 1])
  double roll_cmd = CLAMP_DEF(roll_error * 2.0, -1.0, 1.0);
  double pitch_cmd = CLAMP_DEF(pitch_error * 2.0, -1.0, 1.0);
  double throttle_cmd = CLAMP_DEF(0.3 + distance * 0.01, -1.0, 1.0); // Base throttle + distance compensation
  
  aircraftState.setRollCommand(roll_cmd);
  aircraftState.setPitchCommand(pitch_cmd);
  aircraftState.setThrottleCommand(throttle_cmd);
  
  // Return fitness-like value (lower is better)
  return distance;
}