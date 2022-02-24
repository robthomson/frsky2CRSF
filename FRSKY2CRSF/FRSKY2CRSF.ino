

/*
  Copyright (C) Rob Thomson
 
  Based on code named fromn the opentx project
    th9x - http://code.google.com/p/th9x
    er9x - http://code.google.com/p/er9x
    gruvin9x - http://code.google.com/p/gruvin9x
 
  License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 
  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License version 2 as
  published by the Free Software Foundation.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  Jeti  EX Bus C++ Library for Teensy 3.x
  -------------------------------------------------------------------
  
  Copyright (C) 2018 Bernd Wokoeck
  
  Version history:
  0.90   02/04/2018  created
  0.91   02/09/2018  Support for AtMega32u4 added
  0.92   02/14/2018  Support for ESP32 added
  0.93   02/16/2018  ESP32 uart initialization changed
  0.94   02/17/2018  Generic arduino HardwareSerial support for AtMega328PB
  0.95   03/17/2018  Synchronization (IsBusReleased) for time consuming operations
  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the "Software"),
  to deal in the Software without restriction, including without limitation
  the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the
  Software is furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  IN THE SOFTWARE.
**************************************************************/

 

#include <stdint.h> 
#include "xfire.h"
#include <limits.h>
#include "RTClib.h"
#include <Time.h>
#include <inttypes.h>
#include "sbus.h"


#include "FrSkySportSensor.h"
#include "FrSkySportSensorAss.h"
#include "FrSkySportSensorEsc.h"
#include "FrSkySportSensorFcs.h"
#include "FrSkySportSensorFlvss.h"
#include "FrSkySportSensorGasSuite.h"
#include "FrSkySportSensorGps.h"
#include "FrSkySportSensorRpm.h"
#include "FrSkySportSensorSp2uart.h"
#include "FrSkySportSensorVario.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportTelemetry.h"


bfs::SbusRx sbus_rx(&Serial2);
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbus_data;

//frsky sensors
FrSkySportSensorAss ass;                               // Create ASS sensor with default ID
FrSkySportSensorEsc esc;                               // Create ESC sensor with default ID
FrSkySportSensorFcs fcs;                               // Create FCS-40A sensor with default ID (use ID8 for FCS-150A)
FrSkySportSensorFlvss flvss1;                          // Create FLVSS sensor with default ID
FrSkySportSensorFlvss flvss2(FrSkySportSensor::ID15);  // Create FLVSS sensor with given ID
FrSkySportSensorGasSuite gas;                          // Create Gas Suite sensor with default ID
FrSkySportSensorGps gps;                               // Create GPS sensor with default ID
FrSkySportSensorRpm rpm;                               // Create RPM sensor with default ID
FrSkySportSensorSp2uart sp2uart;                       // Create SP2UART Type B sensor with default ID
FrSkySportSensorVario vario;                           // Create Variometer sensor with default ID
FrSkySportTelemetry telemetry;                                 // Create telemetry object without polling


//bring in all externals related to crossfire
extern uint32_t crossfireChannels[CROSSFIRE_CHANNELS_COUNT];  //pulses data
extern float sensorVario;
extern double sensorGPSLat;
extern double sensorGPSLong;
extern float sensorAltitude;
extern float sensorHeading;
extern uint32_t sensorSpeed;
extern uint32_t sensorSats;
extern float sensorPitch;
extern float sensorRoll;
extern float sensorYaw;
extern double sensorVoltage;
extern double sensorCurrent;
extern double sensorFuel;
extern uint32_t  sensor1RSS;
extern uint32_t sensor2RSS;
extern uint32_t sensorRXQly;
extern uint32_t sensorRXSNR; 
extern uint32_t sensorAntenna; 
extern uint32_t sensorRFMode;
extern uint32_t sensorTXPWR;
extern uint32_t sensorTXRSSI; 
extern uint32_t sensorTXQly;
extern uint32_t sensorTXSNR;
extern uint32_t sensorCapacity;




enum
{
	ID_GPSLON=1,
	ID_GPSLAT,
	ID_VAL11, 
	ID_VAL12, 
	ID_VAL13, 
	ID_VAL14, 
	ID_VAL15, 
	ID_VAL16, 
	ID_VAL17, 
	ID_VAL18, 
	ID_VAL19, 
	ID_VAL20, 
  ID_VAL32,  //meant to be here  
	ID_VAL21, 
	ID_VAL22,
	ID_VAL23,
	ID_VAL24,
  ID_VAL25,
  ID_VAL26,
  ID_VAL27,
  ID_VAL28,
  ID_VAL29,
  ID_VAL30,
  ID_VAL31

};





void setup()
{
  Serial.begin(9600);

  startCrossfire();


      
  //start s.bus
  sbus_rx.Begin();  

  //telemetry.begin(FrSkySportSingleWireSerial::SERIAL_3, &ass, &esc, &fcs, &flvss1, &flvss2, &gas, &gps, &rpm, &sp2uart, &vario);

telemetry.begin(FrSkySportSingleWireSerial::SERIAL_3, &ass, &gps, &vario, &fcs);
}


void loop()
{



    if (sbus_rx.Read()) {
      /* Grab the received data */
      sbus_data = sbus_rx.ch();
      /* Display the received data */
  
      for (int8_t i = 0; i < bfs::SbusRx::NUM_CH(); i++) { 
         crossfireChannels[i] =  map(sbus_data[i],SBUS_LOW,SBUS_HIGH,CROSSFIRE_LOW,CROSSFIRE_HIGH);
      }
    }

  // Set GPS sensor data
  gps.setData(sensorGPSLat, sensorGPSLong,  // Latitude and longitude in degrees decimal (positive for N/E, negative for S/W)
              sensorAltitude,                // Altitude in m (can be negative)
              sensorSpeed,                // Speed in m/s
              sensorHeading,                // Course over ground in degrees (0-359, 0 = north)
              0, 0, 0,            // Date (year - 2000, month, day)
              00, 00, 00);          // Time (hour, minute, second) - will be affected by timezone setings in your radio

  // Set airspeed sensor (ASS) data
  ass.setData((sensorSpeed)*0.036);  // Airspeed in km/h

  // Set variometer data
  // (set Variometer source to VSpd in menu to use the vertical speed data from this sensor for variometer).
  vario.setData(sensorAltitude,  // Altitude in meters (can be negative)
                sensorVario);  // Vertical speed in m/s (positive - up, negative - down)


  // Set current/voltage sensor (FCS) data
  // (set Voltage source to FAS in menu to use this data for battery voltage,
  //  set Current source to FAS in menu to use this data for current readins)
  fcs.setData((sensorCurrent)/10,   // Current consumption in amps
              (sensorVoltage)/10);  // Battery voltage in volts


  
  telemetry.send();




}
