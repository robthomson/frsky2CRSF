/*
  FrSky GPS sensor class for Teensy LC/3.x/4.x, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20210509
  Not for commercial use
*/

#include "FrSkySportSensorGps.h" 

FrSkySportSensorGps::FrSkySportSensorGps(SensorId id) : FrSkySportSensor(id) { }

void FrSkySportSensorGps::setData(float lat, float lon, float alt, float speed, float cog, uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
  latData = setLatLon(lat, true);
  lonData = setLatLon(lon, false);
  cogData = cog * 100;
  speedData = speed * 1944; // Convert m/s to knots
  altData = alt * 100;
  dateData = setDateTime(year, month, day, true);
  timeData = setDateTime(hour, minute, second, false);
}

uint32_t FrSkySportSensorGps::setLatLon(float latLon, bool isLat)
{
  uint32_t data = (uint32_t)((latLon < 0 ? -latLon : latLon) * 60 * 10000) & 0x3FFFFFFF;
  if(isLat == false) data |= 0x80000000;
  if(latLon < 0) data |= 0x40000000;

  return data;
}

uint32_t FrSkySportSensorGps::setDateTime(uint8_t yearOrHour, uint8_t monthOrMinute, uint8_t dayOrSecond, bool isDate)
{
  uint32_t data = yearOrHour;
  data <<= 8;
  data |= monthOrMinute;
  data <<= 8;
  data |= dayOrSecond;
  data <<= 8;
  if(isDate == true) data |= 0xFF;

  return data;
}

uint16_t FrSkySportSensorGps::send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if(sensorId == id)
  {
    switch(sensorDataIdx)
    {
      case 0:
        sendSingleData(serial, GPS_LAT_LON_DATA_ID, dataId, latData, GPS_LAT_LON_DATA_PERIOD, latTime, now);
        break;
      case 1:
        sendSingleData(serial, GPS_LAT_LON_DATA_ID, dataId, lonData, GPS_LAT_LON_DATA_PERIOD, lonTime, now);
        break;
      case 2:
        sendSingleData(serial, GPS_ALT_DATA_ID, dataId, altData, GPS_ALT_DATA_PERIOD, altTime, now);
        break;
      case 3:
        sendSingleData(serial, GPS_SPEED_DATA_ID, dataId, speedData, GPS_SPEED_DATA_PERIOD, speedTime, now);
        break;
      case 4:
       // sendSingleData(serial, GPS_COG_DATA_ID, dataId, cogData, GPS_COG_DATA_PERIOD, cogTime, now);
        break;
      case 5:
        //sendSingleData(serial, GPS_DATE_TIME_DATA_ID, dataId, dateData, GPS_DATE_TIME_DATA_PERIOD, dateTime, now);
        break;
      case 6:
        //sendSingleData(serial, GPS_DATE_TIME_DATA_ID, dataId, timeData, GPS_DATE_TIME_DATA_PERIOD, timeTime, now);
        break;
    }
    sensorDataIdx++;
    if(sensorDataIdx >= GPS_DATA_COUNT) sensorDataIdx = 0;
  }
  return dataId;
}

uint16_t FrSkySportSensorGps::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  if((sensorId == id) || (sensorId == FrSkySportSensor::ID_IGNORE))
  {
    switch(appId)
    {
      case GPS_LAT_LON_DATA_ID:
        {
          float latLonData = (data & 0x3FFFFFFF) / 10000.0 / 60.0;
          if((data & 0x40000000) > 0) latLonData = -latLonData;                 // is negative?
          if((data & 0x80000000) == 0) lat = latLonData; else lon = latLonData; // is latitude?
        }
        return appId;
      case GPS_ALT_DATA_ID:
        altitude = ((int32_t)data) / 100.0;
        return appId;
      case GPS_SPEED_DATA_ID:
        speed = data / 1944.0; // Convert knots to m/s
        return appId;
      case GPS_COG_DATA_ID:
        cog = data / 100.0;
        return appId;
      case GPS_DATE_TIME_DATA_ID:
        if((data & 0xFF) > 0)  // is date?
        {
          data >>= 8; day = data & 0xFF;
          data >>= 8; month = data & 0xFF;
          data >>= 8; year = data & 0xFF;
        }
        else
        {
          data >>= 8; second = data & 0xFF;
          data >>= 8; minute = data & 0xFF;
          data >>= 8; hour = data & 0xFF;
        }
        return appId;
    }
  }
  return SENSOR_NO_DATA_ID;
}

float FrSkySportSensorGps::getLat() { return lat; }
float FrSkySportSensorGps::getLon() { return lon; }
float FrSkySportSensorGps::getAltitude() { return altitude; }
float FrSkySportSensorGps::getSpeed() { return speed; }
float FrSkySportSensorGps::getCog() { return cog; }
uint8_t FrSkySportSensorGps::getYear() { return year; }
uint8_t FrSkySportSensorGps::getMonth() { return month; }
uint8_t FrSkySportSensorGps::getDay() { return day; }
uint8_t FrSkySportSensorGps::getHour() { return hour; }
uint8_t FrSkySportSensorGps::getMinute() { return minute; }
uint8_t FrSkySportSensorGps::getSecond() { return second; }
