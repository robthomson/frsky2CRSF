/*
  FrSky RPM sensor class for Teensy LC/3.x/4.x, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20210509
  Not for commercial use
*/

#include "FrSkySportSensorRpm.h" 

FrSkySportSensorRpm::FrSkySportSensorRpm(SensorId id) : FrSkySportSensor(id) { }

void FrSkySportSensorRpm::setData(uint32_t rpm, float t1, float t2)
{
  rpmData = rpm * 2;
  t1Data = (int32_t)round(t1);
  t2Data = (int32_t)round(t2);
}

uint16_t FrSkySportSensorRpm::send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if(sensorId == id)
  {
    switch(sensorDataIdx)
    {
      case 0:
        sendSingleData(serial, RPM_T1_DATA_ID, dataId, t1Data, RPM_T1_DATA_PERIOD, t1Time, now);
        break;
      case 1:
        sendSingleData(serial, RPM_T2_DATA_ID, dataId, t2Data, RPM_T2_DATA_PERIOD, t2Time, now);
        break;
      case 2:
        sendSingleData(serial, RPM_ROT_DATA_ID, dataId, rpmData, RPM_ROT_DATA_PERIOD, rpmTime, now);
        break;
    }
    sensorDataIdx++;
    if(sensorDataIdx >= RPM_DATA_COUNT) sensorDataIdx = 0;
  }
  return dataId;
}

uint16_t FrSkySportSensorRpm::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  if((sensorId == id) || (sensorId == FrSkySportSensor::ID_IGNORE))
  {
    switch(appId)
    {
      case RPM_T1_DATA_ID:
        t1 = (int32_t)data;
        return appId;
      case RPM_T2_DATA_ID:
        t2 = (int32_t)data;
        return appId;
      case RPM_ROT_DATA_ID:
        rpm = (uint32_t)(data / 2);
        return appId;
    }
  }
  return SENSOR_NO_DATA_ID;
}

uint32_t FrSkySportSensorRpm::getRpm() { return rpm; }
int32_t FrSkySportSensorRpm::getT1() { return t1; }
int32_t FrSkySportSensorRpm::getT2() { return t2; }
