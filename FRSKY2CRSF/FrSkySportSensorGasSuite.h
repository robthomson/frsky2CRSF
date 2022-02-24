/*
  FrSky Gas Suite sensor class for Teensy LC/3.x/4.x, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20210227
  Not for commercial use
*/

#ifndef _FRSKY_SPORT_SENSOR_GAS_SUITE_H_
#define _FRSKY_SPORT_SENSOR_GAS_SUITE_H_

#include "FrSkySportSensor.h"

#define GAS_SUITE_DEFAULT_ID ID23
#define GAS_SUITE_DATA_COUNT 8
#define GAS_SUITE_T1_DATA_ID          0x0D00
#define GAS_SUITE_T2_DATA_ID          0x0D10
#define GAS_SUITE_RPM_DATA_ID         0x0D20
#define GAS_SUITE_RES_VOLUME_DATA_ID  0x0D30
#define GAS_SUITE_RES_PERCENT_DATA_ID 0x0D40
#define GAS_SUITE_FLOW_DATA_ID        0x0D50
#define GAS_SUITE_FLOW_MAX_DATA_ID    0x0D60
#define GAS_SUITE_FLOW_AVG_DATA_ID    0x0D70

#define GAS_SUITE_T1_DATA_PERIOD          100
#define GAS_SUITE_T2_DATA_PERIOD          100
#define GAS_SUITE_RPM_DATA_PERIOD         100
#define GAS_SUITE_RES_VOLUME_DATA_PERIOD  100
#define GAS_SUITE_RES_PERCENT_DATA_PERIOD 100
#define GAS_SUITE_FLOW_DATA_PERIOD        100
#define GAS_SUITE_FLOW_MAX_DATA_PERIOD    100
#define GAS_SUITE_FLOW_AVG_DATA_PERIOD    100

class FrSkySportSensorGasSuite : public FrSkySportSensor
{
  public:
    FrSkySportSensorGasSuite(SensorId id = GAS_SUITE_DEFAULT_ID);
    void setData(float t1, float t2, uint32_t rpm, uint16_t resVol, uint8_t resPercent, uint16_t flow, uint16_t flowMax, uint16_t flowAvg);
    virtual uint16_t send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now);
    virtual uint16_t decodeData(uint8_t id, uint16_t appId, uint32_t data);
    int16_t getT1();
    int16_t getT2();
    uint32_t getRpm();
    uint16_t getResVol();
    uint8_t getResPercent();
    uint16_t getFlow();
    uint16_t getFlowMax();
    uint16_t getFlowAvg();

  private:
    int32_t t1Data;
    int32_t t2Data;
    uint32_t rpmData;
    uint32_t resVolData;
    uint32_t resPercentData;
    uint32_t flowData;
    uint32_t flowMaxData;
    uint32_t flowAvgData;
    uint32_t t1Time;
    uint32_t t2Time;
    uint32_t rpmTime;
    uint32_t resVolTime;
    uint32_t resPercentTime;
    uint32_t flowTime;
    uint32_t flowMaxTime;
    uint32_t flowAvgTime;
    int16_t t1;
    int16_t t2;
    uint32_t rpm;
    uint16_t resVol;
    uint8_t resPercent;
    uint16_t flow;
    uint16_t flowMax;
    uint16_t flowAvg;
};

#endif // _FRSKY_SPORT_SENSOR_GAS_SUITE_H_
