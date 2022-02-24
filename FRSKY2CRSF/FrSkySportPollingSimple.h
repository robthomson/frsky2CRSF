/*
  FrSky sensor data polling class for Teensy LC/3.x/4.x, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20210108
  Not for commercial use
*/

#ifndef _FRSKY_SPORT_POLLING_SIMPLE_H_
#define _FRSKY_SPORT_POLLING_SIMPLE_H_

#include "FrSkySportPolling.h"

class FrSkySportPollingSimple : public FrSkySportPolling
{
  public:
    FrSkySportPollingSimple();

  private:
    virtual FrSkySportSensor::SensorId getNextId();
    uint8_t  nextPollIdIdx;
};

#endif // _FRSKY_SPORT_POLLING_SIMPLE_H_

