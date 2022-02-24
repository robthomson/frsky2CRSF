/*
  FrSky Gas Suite sensor class for Teensy LC/3.x/4.x, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20210509
  Not for commercial use
*/

#include "FrSkySportSensorGasSuite.h" 

FrSkySportSensorGasSuite::FrSkySportSensorGasSuite(SensorId id) : FrSkySportSensor(id) { }

void FrSkySportSensorGasSuite::setData(float t1, float t2, uint32_t rpm, uint16_t resVol, uint8_t resPercent, uint16_t flow, uint16_t flowMax, uint16_t flowAvg)
{
  t1Data = (int32_t)round(t1); // DEVIATION FROM SPEC: FrSky protocol spec uses only bits 0-15 to store the data
  t2Data = (int32_t)round(t2); // DEVIATION FROM SPEC: FrSky protocol spec uses only bits 0-15 to store the data
  rpmData = rpm;
  resVolData = resVol;
  resPercentData = resPercent; // DEVIATION FROM SPEC: FrSky protocol spec uses only bits 0-15 to store the data
  flowData = flow;             // DEVIATION FROM SPEC: FrSky protocol spec uses only bits 0-15 to store the data 
  flowMaxData = flowMax;       // DEVIATION FROM SPEC: FrSky protocol spec uses only bits 0-15 to store the data
  flowAvgData = flowAvg;       // DEVIATION FROM SPEC: FrSky protocol spec uses only bits 0-15 to store the data
}

uint16_t FrSkySportSensorGasSuite::send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if(sensorId == id)
  {
    switch(sensorDataIdx)
    {
      case 0:
        sendSingleData(serial, GAS_SUITE_T1_DATA_ID, dataId, t1Data, GAS_SUITE_T1_DATA_PERIOD, t1Time, now);
        break;
      case 1:
        sendSingleData(serial, GAS_SUITE_T2_DATA_ID, dataId, t2Data, GAS_SUITE_T2_DATA_PERIOD, t2Time, now);
        break;
      case 2:
        sendSingleData(serial, GAS_SUITE_RPM_DATA_ID, dataId, rpmData, GAS_SUITE_RPM_DATA_PERIOD, rpmTime, now);
        break;
      case 3:
        sendSingleData(serial, GAS_SUITE_RES_VOLUME_DATA_ID, dataId, resVolData, GAS_SUITE_RES_VOLUME_DATA_PERIOD, resVolTime, now);
        break;
      case 4:
        sendSingleData(serial, GAS_SUITE_RES_PERCENT_DATA_ID, dataId, resPercentData, GAS_SUITE_RES_PERCENT_DATA_PERIOD, resPercentTime, now);
        break;
      case 5:
        sendSingleData(serial, GAS_SUITE_FLOW_DATA_ID, dataId, flowData, GAS_SUITE_FLOW_DATA_PERIOD, flowTime, now);
        break;
      case 6:
        sendSingleData(serial, GAS_SUITE_FLOW_MAX_DATA_ID, dataId, flowMaxData, GAS_SUITE_FLOW_MAX_DATA_PERIOD, flowMaxTime, now);
        break;
      case 7:
        sendSingleData(serial, GAS_SUITE_FLOW_AVG_DATA_ID, dataId, flowAvgData, GAS_SUITE_FLOW_AVG_DATA_PERIOD, flowAvgTime, now);
        break;
    }
    sensorDataIdx++;
    if(sensorDataIdx >= GAS_SUITE_DATA_COUNT) sensorDataIdx = 0;
  }
  return dataId;
}

uint16_t FrSkySportSensorGasSuite::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  if((sensorId == id) || (sensorId == FrSkySportSensor::ID_IGNORE))
  {
    switch(appId)
    {
      case GAS_SUITE_T1_DATA_ID:
        t1 = (int16_t)data;
        return appId;
      case GAS_SUITE_T2_DATA_ID:
        t2 = (int16_t)data;
        return appId;
      case GAS_SUITE_RPM_DATA_ID:
        rpm = (uint32_t)data;
        return appId;
      case GAS_SUITE_RES_VOLUME_DATA_ID:
        resVol = (uint16_t)data;
        return appId;
      case GAS_SUITE_RES_PERCENT_DATA_ID:
        resPercent = (uint8_t)data;
        return appId;
      case GAS_SUITE_FLOW_DATA_ID:
        flow = (uint16_t)data;
        return appId;
      case GAS_SUITE_FLOW_MAX_DATA_ID:
        flowMax = (uint16_t)data;
        return appId;
      case GAS_SUITE_FLOW_AVG_DATA_ID:
        flowAvg = (uint16_t)data;
        return appId;
    }
  }
  return SENSOR_NO_DATA_ID;
}

int16_t FrSkySportSensorGasSuite::getT1() { return t1; }
int16_t FrSkySportSensorGasSuite::getT2() { return t2; }
uint32_t FrSkySportSensorGasSuite::getRpm() { return rpm; }
uint16_t FrSkySportSensorGasSuite::getResVol() { return resVol; }
uint8_t FrSkySportSensorGasSuite::getResPercent() { return resPercent; }
uint16_t FrSkySportSensorGasSuite::getFlow() { return flow; }
uint16_t FrSkySportSensorGasSuite::getFlowMax() { return flowMax; }
uint16_t FrSkySportSensorGasSuite::getFlowAvg() { return flowAvg; }
