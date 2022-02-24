/*
  FrSky FCS-40A/FCS-150A current sensor class for Teensy LC/3.x/4.x, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20210509
  Not for commercial use
*/

#include "FrSkySportSensorFcs.h" 

FrSkySportSensorFcs::FrSkySportSensorFcs(SensorId id) : FrSkySportSensor(id) { }

void FrSkySportSensorFcs::setData(float current, float voltage)
{
  currentData = (uint32_t)(current * 10);
  voltageData = (uint32_t)(voltage * 100);
}

uint16_t FrSkySportSensorFcs::send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if(sensorId == id)
  {
    switch(sensorDataIdx)
    {
      case 0:
        sendSingleData(serial, FCS_CURR_DATA_ID, dataId, currentData, FCS_CURR_DATA_PERIOD, currentTime, now);
        break;
      case 1:
        sendSingleData(serial, FCS_VOLT_DATA_ID, dataId, voltageData, FCS_VOLT_DATA_PERIOD, voltageTime, now);
        break;
    }
    sensorDataIdx++;
    if(sensorDataIdx >= FCS_DATA_COUNT) sensorDataIdx = 0;
  }
  return dataId;
}

uint16_t FrSkySportSensorFcs::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  if((sensorId == id) || (sensorId == FrSkySportSensor::ID_IGNORE))
  {
    switch(appId)
    {
      case FCS_CURR_DATA_ID:
        current = data / 10.0;
        return appId;
      case FCS_VOLT_DATA_ID:
        voltage = data / 100.0;
        return appId;
    }
  }
  return SENSOR_NO_DATA_ID;
}

float FrSkySportSensorFcs::getCurrent() { return current; }
float FrSkySportSensorFcs::getVoltage() { return voltage; }
