#pragma once

#include <Arduino.h>

float l1BankCommandRad(float crossTrackErrorM, float groundspeedMps, float l1DistanceM);
void tecsEnergyCommand(float altitudeErrorM, float airspeedErrorMps,
                       float& pitchCmd, float& throttleCmd);
