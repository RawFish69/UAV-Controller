#ifndef JOYSTICK_HANDLER_H
#define JOYSTICK_HANDLER_H

#ifdef BUILD_TX  // Only include for transmitter

#include <Arduino.h>

/**
 * Joystick Handler for Throttle and Yaw Control
 * 
 * This module handles analog joystick reading for throttle and yaw control.
 * Provides normalized values with deadband, smoothing, and calibration support.
 */

class JoystickHandler {
public:
  /**
   * Initialize the joystick handler
   * @param throttlePin Analog pin for throttle axis (typically up/down on left stick)
   * @param yawPin Analog pin for yaw axis (typically left/right on left stick)
   * @param buttonPin Digital pin for joystick button (SW pin, optional, use 255 to disable)
   */
  void begin(uint8_t throttlePin, uint8_t yawPin, uint8_t buttonPin = 255);
  
  /**
   * Update joystick readings
   * Call this regularly (e.g., at your RC send frequency)
   */
  void update();
  
  /**
   * Get normalized throttle value (0.0 to 1.0)
   * 0.0 = minimum throttle (joystick at center/rest or pushed down)
   * 1.0 = maximum throttle (joystick pushed up)
   * Note: Only uses upper half of joystick travel (like drone throttle stick)
   */
  float getThrottle();
  
  /**
   * Get normalized yaw value (-1.0 to 1.0)
   * -1.0 = full left, 0.0 = center, 1.0 = full right
   */
  float getYaw();
  
  /**
   * Get button state (true = pressed, false = not pressed)
   * Returns false if button pin is not configured
   */
  bool getButton();
  
  /**
   * Check if button was just pressed (rising edge detection)
   * Useful for toggle actions
   */
  bool getButtonPressed();
  
  /**
   * Check if button was just released (falling edge detection)
   */
  bool getButtonReleased();
  
  /**
   * Get raw ADC readings (for debugging/calibration)
   */
  void getRawValues(uint16_t &throttleRaw, uint16_t &yawRaw);
  
  /**
   * Set deadband for yaw axis (prevents drift near center)
   * @param deadband Value from 0.0 to 1.0 (default: 0.05 = 5%)
   */
  void setYawDeadband(float deadband) { yawDeadband = constrain(deadband, 0.0, 0.5); }
  
  /**
   * Set throttle deadband (prevents jitter at min/max)
   * @param deadband Value from 0.0 to 1.0 (default: 0.02 = 2%)
   */
  void setThrottleDeadband(float deadband) { throttleDeadband = constrain(deadband, 0.0, 0.1); }
  
  /**
   * Enable/disable smoothing filter
   * @param enable true to enable exponential smoothing
   * @param alpha Smoothing factor (0.0-1.0, lower = more smoothing, default: 0.3)
   */
  void setSmoothing(bool enable, float alpha = 0.3);
  
  /**
   * Calibrate joystick center and endpoints
   * Call this when joystick is at center position
   */
  void calibrateCenter();
  
  /**
   * Manually set throttle range (use ADC values from serial monitor)
   * @param minValue ADC value when throttle is pushed UP fully (typically 100-500)
   * @param centerValue ADC value when throttle is at rest/center (typically 1900-2200)
   */
  void setThrottleRange(uint16_t minValue, uint16_t centerValue) {
    throttleMin = minValue;
    throttleCenter = centerValue;
    Serial.printf("[Joystick] Throttle range set: min=%d center=%d\n", throttleMin, throttleCenter);
  }
  
  /**
   * Set whether throttle stick is spring-loaded (returns to center)
   * If true, treats throttle like yaw (centered at mid-point)
   * If false (default), treats throttle as 0-100% (typical for drones)
   * @param springLoaded true for spring-loaded stick, false for ratcheted/stays-put stick
   */
  void setThrottleSpringLoaded(bool springLoaded) { throttleIsSpringLoaded = springLoaded; }

private:
  uint8_t throttlePin;
  uint8_t yawPin;
  uint8_t buttonPin;
  bool buttonEnabled;
  
  // Button state tracking
  bool buttonState = false;          // Current button state
  bool buttonLastState = false;      // Previous button state
  bool buttonPressed = false;        // Rising edge (just pressed)
  bool buttonReleased = false;       // Falling edge (just released)
  unsigned long buttonDebounceTime = 0;  // For debouncing
  static const unsigned long DEBOUNCE_DELAY = 50;  // 50ms debounce
  
  // ADC configuration
  static const uint16_t ADC_MIN = 0;
  static const uint16_t ADC_MAX = 4095;  // 12-bit ADC on ESP32
  static const uint16_t ADC_CENTER = 2048;
  
  // Calibration values (can be updated via calibrateCenter())
  uint16_t throttleCenter = ADC_CENTER;
  uint16_t yawCenter = ADC_CENTER;
  uint16_t throttleMin = ADC_MIN;
  uint16_t throttleMax = ADC_MAX;
  uint16_t yawMin = ADC_MIN;
  uint16_t yawMax = ADC_MAX;
  
  // Raw readings
  uint16_t throttleRaw = ADC_CENTER;
  uint16_t yawRaw = ADC_CENTER;
  
  // Normalized values
  float throttleNormalized = 0.0;
  float yawNormalized = 0.0;
  
  // Smoothed values (if smoothing enabled)
  float throttleSmoothed = 0.0;
  float yawSmoothed = 0.0;
  
  // Configuration
  float yawDeadband = 0.05;        // 5% deadband around center
  float throttleDeadband = 0.02;   // 2% deadband at min/max
  bool smoothingEnabled = true;
  float smoothingAlpha = 0.3;      // Exponential smoothing factor
  bool throttleIsSpringLoaded = false;  // false = normal throttle (0-100%)
  
  /**
   * Apply deadband to a normalized value around center (0.0)
   */
  float applyDeadband(float value, float deadband);
  
  /**
   * Map ADC value to normalized range with calibration
   */
  float mapToNormalized(uint16_t raw, uint16_t minVal, uint16_t center, uint16_t maxVal, bool isCentered);
};

#endif // BUILD_TX

#endif // JOYSTICK_HANDLER_H

