#include "joystick_handler.h"

#ifdef BUILD_TX  // Only compile for transmitter

void JoystickHandler::begin(uint8_t throttlePin, uint8_t yawPin, uint8_t buttonPin) {
  this->throttlePin = throttlePin;
  this->yawPin = yawPin;
  this->buttonPin = buttonPin;
  this->buttonEnabled = (buttonPin != 255);
  
  // Configure ADC pins
  pinMode(throttlePin, INPUT);
  pinMode(yawPin, INPUT);
  
  // Configure button pin if enabled
  if (buttonEnabled) {
    pinMode(buttonPin, INPUT_PULLUP);  // Use internal pull-up resistor
    buttonState = digitalRead(buttonPin);
    buttonLastState = buttonState;
  }
  
  // Set ADC resolution (ESP32 supports 9-12 bits)
  analogReadResolution(12);  // 12-bit = 0-4095
  
  // Set ADC attenuation for full 0-3.3V range
  analogSetAttenuation(ADC_11db);  // 0-3.3V range
  
  Serial.println("\n[Joystick] Initialized");
  Serial.printf("     Throttle Pin: GPIO%d\n", throttlePin);
  Serial.printf("     Yaw Pin: GPIO%d\n", yawPin);
  if (buttonEnabled) {
    Serial.printf("     Button Pin: GPIO%d (active LOW)\n", buttonPin);
  } else {
    Serial.println("     Button Pin: Disabled");
  }
  Serial.printf("     ADC Resolution: 12-bit (0-%d)\n", ADC_MAX);
  
  // Read initial values
  update();
  
  Serial.printf("     Initial readings - Throttle: %d, Yaw: %d", throttleRaw, yawRaw);
  if (buttonEnabled) {
    Serial.printf(", Button: %s\n", buttonState ? "Released" : "Pressed");
  } else {
    Serial.println();
  }
  
  // Suggest calibration to find actual joystick range
  Serial.println("\n[Joystick] For best performance, move throttle to extremes:");
  Serial.println("     1. Push throttle UP fully and check ADC value");
  Serial.println("     2. Leave at CENTER/REST and check ADC value");
  Serial.println("     These will be shown in the stats output");
  Serial.println();
}

void JoystickHandler::update() {
  // Read analog values
  throttleRaw = analogRead(throttlePin);
  yawRaw = analogRead(yawPin);
  
  // Read button state with debouncing
  if (buttonEnabled) {
    bool reading = digitalRead(buttonPin);
    
    // Reset edge detection flags
    buttonPressed = false;
    buttonReleased = false;
    
    // Check if button state changed
    if (reading != buttonLastState) {
      buttonDebounceTime = millis();
    }
    
    // If enough time has passed, accept the new state
    if ((millis() - buttonDebounceTime) > DEBOUNCE_DELAY) {
      // If state changed and is stable
      if (reading != buttonState) {
        buttonState = reading;
        
        // Detect edges (button is active LOW, so LOW = pressed)
        if (buttonState == LOW) {
          buttonPressed = true;   // Just pressed
        } else {
          buttonReleased = true;  // Just released
        }
      }
    }
    
    buttonLastState = reading;
  }
  
  // Map to normalized values
  if (throttleIsSpringLoaded) {
    // Spring-loaded throttle: treat like yaw (centered)
    throttleNormalized = mapToNormalized(throttleRaw, throttleMin, throttleCenter, throttleMax, true);
    throttleNormalized = applyDeadband(throttleNormalized, throttleDeadband);
  } else {
    // Normal throttle: only use upper half of joystick travel
    // Center position (rest) = 0% throttle
    // Push UP (towards min ADC) = 100% throttle
    // Push DOWN (towards max ADC) = 0% throttle (ignore)
    
    if (throttleRaw <= throttleCenter) {
      // Upper half: center to min (pushed up) -> 0.0 to 1.0
      throttleNormalized = map((float)throttleRaw, (float)throttleMin, (float)throttleCenter, 1.0, 0.0);
    } else {
      // Lower half: center to max (pushed down or at rest) -> 0.0
      throttleNormalized = 0.0;
    }
    
    // Clamp to valid range
    throttleNormalized = constrain(throttleNormalized, 0.0, 1.0);
    
    // Apply small deadband at endpoints to prevent jitter
    if (throttleNormalized < throttleDeadband) throttleNormalized = 0.0;
    if (throttleNormalized > (1.0 - throttleDeadband)) throttleNormalized = 1.0;
  }
  
  // Yaw: centered axis (-1 to 1)
  yawNormalized = mapToNormalized(yawRaw, yawMin, yawCenter, yawMax, true);
  yawNormalized = applyDeadband(yawNormalized, yawDeadband);
  
  // Apply smoothing if enabled
  if (smoothingEnabled) {
    // Exponential moving average: output = alpha * new + (1-alpha) * old
    throttleSmoothed = smoothingAlpha * throttleNormalized + (1.0 - smoothingAlpha) * throttleSmoothed;
    yawSmoothed = smoothingAlpha * yawNormalized + (1.0 - smoothingAlpha) * yawSmoothed;
  } else {
    throttleSmoothed = throttleNormalized;
    yawSmoothed = yawNormalized;
  }
}

float JoystickHandler::getThrottle() {
  return smoothingEnabled ? throttleSmoothed : throttleNormalized;
}

float JoystickHandler::getYaw() {
  return smoothingEnabled ? yawSmoothed : yawNormalized;
}

bool JoystickHandler::getButton() {
  if (!buttonEnabled) return false;
  return (buttonState == LOW);  // Active LOW, so LOW = pressed
}

bool JoystickHandler::getButtonPressed() {
  return buttonPressed;
}

bool JoystickHandler::getButtonReleased() {
  return buttonReleased;
}

void JoystickHandler::getRawValues(uint16_t &throttleRaw, uint16_t &yawRaw) {
  throttleRaw = this->throttleRaw;
  yawRaw = this->yawRaw;
}

void JoystickHandler::setSmoothing(bool enable, float alpha) {
  smoothingEnabled = enable;
  smoothingAlpha = constrain(alpha, 0.0, 1.0);
  
  if (enable) {
    Serial.printf("[Joystick] Smoothing enabled (alpha=%.2f)\n", smoothingAlpha);
  } else {
    Serial.println("[Joystick] Smoothing disabled");
  }
}

void JoystickHandler::calibrateCenter() {
  Serial.println("\n[Joystick] Calibrating center position...");
  Serial.println("     Center the sticks and wait...");
  delay(1000);
  
  // Read multiple samples for accuracy
  uint32_t throttleSum = 0;
  uint32_t yawSum = 0;
  const int numSamples = 50;
  
  for (int i = 0; i < numSamples; i++) {
    throttleSum += analogRead(throttlePin);
    yawSum += analogRead(yawPin);
    delay(20);
  }
  
  throttleCenter = throttleSum / numSamples;
  yawCenter = yawSum / numSamples;
  
  Serial.println("[Joystick] Calibration complete!");
  Serial.printf("     Throttle center: %d\n", throttleCenter);
  Serial.printf("     Yaw center: %d\n", yawCenter);
  Serial.println();
}

float JoystickHandler::applyDeadband(float value, float deadband) {
  if (fabs(value) < deadband) {
    return 0.0;
  }
  
  // Scale the remaining range to maintain smooth transition
  if (value > 0) {
    return (value - deadband) / (1.0 - deadband);
  } else {
    return (value + deadband) / (1.0 - deadband);
  }
}

float JoystickHandler::mapToNormalized(uint16_t raw, uint16_t minVal, uint16_t center, uint16_t maxVal, bool isCentered) {
  if (isCentered) {
    // Map to -1.0 to 1.0 with center at 0.0
    if (raw >= center) {
      // Upper half: center to max -> 0.0 to 1.0
      return map(raw, center, maxVal, 0.0, 1.0);
    } else {
      // Lower half: min to center -> -1.0 to 0.0
      return map(raw, minVal, center, -1.0, 0.0);
    }
  } else {
    // Map to 0.0 to 1.0 (throttle)
    return map(raw, minVal, maxVal, 0.0, 1.0);
  }
}

// Helper function for float mapping
float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif // BUILD_TX

