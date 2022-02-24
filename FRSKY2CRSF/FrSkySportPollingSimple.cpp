/*
  FrSky sensor simple data polling class for Teensy LC/3.x/4.x, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20210108
  Not for commercial use
*/

#include "FrSkySportPollingSimple.h"

FrSkySportPollingSimple::FrSkySportPollingSimple() : FrSkySportPolling()
{
  nextPollIdIdx = 0;
}

FrSkySportSensor::SensorId FrSkySportPollingSimple::getNextId()
{
  if(nextPollIdIdx >= FRSKY_POLLED_ID_COUNT) nextPollIdIdx = 0;
  return POLLED_ID_TABLE[nextPollIdIdx++];
}
