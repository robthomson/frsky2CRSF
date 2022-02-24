/*
  FrSky ASS-70/ASS-100 airspeed sensor class for Teensy LC/3.x/4.x, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20210509
  Not for commercial use
*/

#include "FrSkySportSensorAss.h" 

FrSkySportSensorAss::FrSkySportSensorAss(SensorId id) : FrSkySportSensor(id) { }

void FrSkySportSensorAss::setData(float speed)
{
  speedData = (uint32_t)(speed * 10.0);
}

uint16_t FrSkySportSensorAss::send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if(sensorId == id) sendSingleData(serial, ASS_SPEED_DATA_ID, dataId, speedData, ASS_SPEED_DATA_PERIOD, speedTime, now);
  return dataId;
}

uint16_t FrSkySportSensorAss::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  if((sensorId == id) || (sensorId == FrSkySportSensor::ID_IGNORE))
  {
    if(appId == ASS_SPEED_DATA_ID)
    {
      speed = data / 10.0;
      return appId;
    }
  }
  return SENSOR_NO_DATA_ID;
}

float FrSkySportSensorAss::getSpeed() { return speed; }
