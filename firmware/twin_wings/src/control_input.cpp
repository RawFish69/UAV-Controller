#include "control_input.h"

#include <Arduino.h>

namespace {

String g_inputLine;

void parseLine(const String& line, ControlInput& out) {
  if (line.length() == 0) {
    return;
  }

  // Bench/test input: "roll,pitch,yaw,throttle", each -1..1 except throttle 0..1.
  // Replace with CRSF/SBUS/PPM or an autopilot command parser for flight.
  float values[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  int parsed = sscanf(line.c_str(), "%f,%f,%f,%f", &values[0], &values[1],
                      &values[2], &values[3]);
  if (parsed >= 1) out.roll = values[0];
  if (parsed >= 2) out.pitch = values[1];
  if (parsed >= 3) out.yaw = values[2];
  if (parsed >= 4) out.throttle = values[3];
}

}  // namespace

void readControlInput(ControlInput& out) {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      parseLine(g_inputLine, out);
      g_inputLine = "";
      if (c == '\r') {
        continue;
      }
    } else {
      g_inputLine += c;
    }
  }
}
