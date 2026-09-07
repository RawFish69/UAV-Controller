#pragma once

#include <Arduino.h>

#include "control_input.h"

void rcInputInit(HardwareSerial& serial, uint32_t baud, bool invert = false);
bool rcInputReadCrsf(Stream& stream, ControlInput& out);
bool rcInputReadSbus(Stream& stream, ControlInput& out);
