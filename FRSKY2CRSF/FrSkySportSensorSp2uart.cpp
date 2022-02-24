/*
  FrSky S.Port to UART Remote (Type B) converter class for Teensy LC/3.x/4.x, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20210509
  Not for commercial use
  
  Note that only analog ports ADC3 and ADC4 are implemented, not the UART part.
*/

#include "FrSkySportSensorSp2uart.h" 

FrSkySportSensorSp2uart::FrSkySportSensorSp2uart(SensorId id) : FrSkySportSensor(id) { }

void FrSkySportSensorSp2uart::setData(float adc3, float adc4)
{
  adc3Data = (uint32_t)(adc3 * 100);
  adc4Data = (uint32_t)(adc4 * 100);
}

uint16_t FrSkySportSensorSp2uart::send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if(sensorId == id)
  {
    switch(sensorDataIdx)
    {
      case 0:
        sendSingleData(serial, SP2UARTB_ADC3_DATA_ID, dataId, adc3Data, SP2UARTB_ADC3_DATA_PERIOD, adc3Time, now);
        break;
      case 1:
        sendSingleData(serial, SP2UARTB_ADC4_DATA_ID, dataId, adc4Data, SP2UARTB_ADC4_DATA_PERIOD, adc4Time, now);
        break;
    }
    sensorDataIdx++;
    if(sensorDataIdx >= SP2UARTB_DATA_COUNT) sensorDataIdx = 0;
  }
  return dataId;
}

uint16_t FrSkySportSensorSp2uart::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  if((sensorId == id) || (sensorId == FrSkySportSensor::ID_IGNORE))
  {
    switch(appId)
    {
      case SP2UARTB_ADC3_DATA_ID:
        adc3 = data / 100.0;
        return appId;
      case SP2UARTB_ADC4_DATA_ID:
        adc4 = data / 100.0;
        return appId;
    }
  }
  return SENSOR_NO_DATA_ID;
}

float FrSkySportSensorSp2uart::getAdc3() { return adc3; }
float FrSkySportSensorSp2uart::getAdc4() { return adc4; }
