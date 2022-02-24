/*
  FrSky ESC sensor class for Teensy LC/3.x/4.x, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20210509
  Not for commercial use
*/

#include "FrSkySportSensorEsc.h" 

FrSkySportSensorEsc::FrSkySportSensorEsc(SensorId id) : FrSkySportSensor(id) { }

void FrSkySportSensorEsc::setData(float volt, float curr, uint32_t rpm, uint16_t cons, float temp, float sbecVolt, float sbecCurr)
{
  powerData = (((uint32_t)round(curr * 100)) << 16) + (uint32_t)round(volt * 100);
  rpmConsData = (((uint32_t)cons) << 16) + (rpm / 100);
  tempData = (int32_t)round(temp);
  sbecData = (((uint32_t)round(sbecCurr * 1000)) << 16) + (uint32_t)round(sbecVolt * 1000);
}

uint16_t FrSkySportSensorEsc::send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if(sensorId == id)
  {
    switch(sensorDataIdx)
    {
      case 0:
        sendSingleData(serial, ESC_POWER_DATA_ID, dataId, powerData, ESC_POWER_DATA_PERIOD, powerTime, now);
        break;
      case 1:
        sendSingleData(serial, ESC_RPM_CONS_DATA_ID, dataId, rpmConsData, ESC_RPM_CONS_DATA_PERIOD, rpmConsTime, now);
        break;
      case 2:
        sendSingleData(serial, ESC_TEMP_DATA_ID, dataId, tempData, ESC_TEMP_DATA_PERIOD, tempTime, now);
        break;
      case 3:
        sendSingleData(serial, ESC_SBEC_DATA_ID, dataId, sbecData, ESC_SBEC_DATA_PERIOD, sbecTime, now);
        break;
    }
    sensorDataIdx++;
    if(sensorDataIdx >= ESC_DATA_COUNT) sensorDataIdx = 0;
  }
  return dataId;
}

uint16_t FrSkySportSensorEsc::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  if((sensorId == id) || (sensorId == FrSkySportSensor::ID_IGNORE))
  {
    switch(appId)
    {
      case ESC_POWER_DATA_ID:
        volt = (data & 0xFFFF) / 100.0;
        curr = (data >> 16) / 100.0;
        return appId;
      case ESC_RPM_CONS_DATA_ID:
        rpm = (data & 0xFFFF) * 100;
        cons = data >> 16;
        return appId;
      case ESC_TEMP_DATA_ID:
        temp = (int32_t)data;
        return appId;
      case ESC_SBEC_DATA_ID:
        sbecVolt = (data & 0xFFFF) / 1000.0;
        sbecCurr = (data >> 16) / 1000.0;
        return appId;
    }
  }
  return SENSOR_NO_DATA_ID;
}

float FrSkySportSensorEsc::getVoltage() { return volt; }
float FrSkySportSensorEsc::getCurrent() { return curr; }
uint32_t FrSkySportSensorEsc::getRpm() { return rpm; }
uint16_t FrSkySportSensorEsc::getConsumption() { return cons; }
int32_t FrSkySportSensorEsc::getTemp() { return temp; }
float FrSkySportSensorEsc::getSbecVoltage() { return sbecVolt; }
float FrSkySportSensorEsc::getSbecCurrent() { return sbecCurr; }
