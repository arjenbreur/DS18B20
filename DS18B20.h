#ifndef _DS18B20_h_
#define _DS18B20_h_

#include <Arduino.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
#include <OneWire.h>

#define TEMP_ERROR -273.15f

class DS18B20
{
public:
  DS18B20(OneWire *oneWire);
  bool begin(uint8_t quality=12, float resolution=0.5F);
  bool request();
  bool available();
  float readTemperature();

private:
  const byte _DS18S20_ID = 0x10;
  const byte _DS18B20_ID = 0x28;

  OneWire *_oneWire;
  uint8_t _quality;
  float _resolution;
  bool _powerType;
  uint32_t _beginConversionTime;
  uint8_t firstSensorAddress[8];

  bool storeFirstSensorAddress();
  bool _sendCommand(uint8_t *address, uint8_t command);
  bool _sendQuality(uint8_t *address);
  bool _receivePowerType(uint8_t *address);
};
#endif