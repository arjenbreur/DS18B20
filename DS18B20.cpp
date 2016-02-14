#include "DS18B20.h"

// Exception handler
// Prints the line number of the exception and block the program, when the function returns false
void __check(bool value, uint16_t line)
{
  if (value)
    return;

  Serial.print(F("EXCEPTION at line: "));
  Serial.println(line);
  while(1);
}

// Constructor
// Argument: Pointer to OneWire object
// Return: New DS18B20 object
DS18B20::DS18B20(OneWire *oneWire)
{
  _oneWire = oneWire;
  _quality = 0;
  _samePowerType = false;
  _powerType = false;
  _beginConversionTime = 0;
}

// Setup for all ds19b20 sensors in 1-Wire bus.
// Argument: quality - measurement resolution in bits (from 9 to 12)
// Return:
// true - if all operations were successful
// false - when the bus is physically damaged
//       - when devices not respond
//       - when device address is not valid
//       - when not detect any device
bool DS18B20::begin(uint8_t quality)
{
  _quality = constrain(quality, 9, 12);
  uint8_t address[8];
  uint8_t devices = 0;
  uint8_t parasiteDevices = 0;

  uint32_t beginResetTimeout = millis();

  while(!_oneWire->reset())
  {
    uint32_t elapsedResetTimeout = millis() - beginResetTimeout;

    if (elapsedResetTimeout > 1000)
      return false;
  }

  _oneWire->reset_search();
  while (_oneWire->search(address))
  {
    if (OneWire::crc8(address, 7) != address[7])
      return false;

    if (address[0] != _DS18B20_ID && address[0] != _DS18S20_ID)
      continue;

    if (!_sendQuality(address))
      return false;

    _powerType = _receivePowerType(address);
    if (!_powerType)
      parasiteDevices++;

    devices++;
  }

  if (parasiteDevices == devices || parasiteDevices == 0)
    _samePowerType = true;

  if (devices == 0)
    return false;
    
  storeFirstSensorAddress();

  return true;
}

// Store address of first device found
// Return:
// - true - if operation were successful
// - false - if no valid device is found
bool DS18B20::storeFirstSensorAddress()
{
    //search for a device
    if (!_oneWire->search(firstSensorAddress)) {
        // not a single device found
        _oneWire->reset_search();
        return false;
    }
    // device found, check the crc
    if (OneWire::crc8( firstSensorAddress, 7) != firstSensorAddress[7]) {
        // incorrect crc, search again recursively for next device on the bus
        if(!storeFirstSensorAddress()) return false;
    }
    if (firstSensorAddress[0] != _DS18S20_ID && firstSensorAddress[0] != _DS18B20_ID) {
        // incorrect device ID, search again recursively for next device on the bus
        if(!storeFirstSensorAddress()) return false;
    }
    // device found and stored.
    return true;
}

// Request for temperature measurements on all devices
// Return:
// - true - if operation were successful
// - false - if devices have different ways of power (combinations of normal and parasite in one bus)
//         - if devices not responding
bool DS18B20::request()
{
  if (!_samePowerType)
    return false;

  if (!_oneWire->reset())
    return false;

  _oneWire->skip();
  _oneWire->write(0x44, !_powerType);

  _beginConversionTime = millis();

  return true;
}

// Request for temperature measurements on device
// Argument: Pointer to an array of device address
// Return:
// - true - if operation were successful
// - false - if device not responding
bool DS18B20::request(uint8_t *address)
{
  if (!_samePowerType)
  {
    _powerType = _receivePowerType(address);
  }

  if (!_oneWire->reset())
    return false;

  _oneWire->select(address);
  _oneWire->write(0x44, !_powerType);

  _beginConversionTime = millis();

  return true;
}

// Request for temperature measurements on device
// Argument: Pointer to an array in flash memory of device address
// Return:
// - true - if operation were successful
// - false - if device not responding
bool DS18B20::request(const __FlashStringHelper *_address)
{
  uint8_t address[8];
  _readFlashAddress(_address, address);

  return request(address);
}

// Checks if devices completed the measurement
// Return:
// - true - the measurement was completed
// - false - the measurement wasn't completed
bool DS18B20::available(void)
{
  uint32_t durationTime[] = {94, 188, 375, 750};
  uint32_t elapsedTime = millis() - _beginConversionTime;
  bool timeout = elapsedTime >= durationTime[_quality-9];

  if (_powerType)
  {
    bool ready = _oneWire->read_bit();

    if (ready || timeout)
      return true;
  }

  if (timeout)
    return true;

  return false;
}

// Read temperature from device
// Argument: Pointer to an array of device address
// Return: temperature in degrees Celsius
// If the temperature is TEMP_ERROR value - measurement failed because:
// - the bus is physically damaged
// - devices not respond
// - when data from the device is not valid
// - when not detect device of thad address
float DS18B20::readTemperature(uint8_t *address)
{
  uint8_t scratchpad[9];

  if (!_sendCommand(address, 0xbe))
    return TEMP_ERROR;

  _oneWire->read_bytes(scratchpad, 9);

  if (OneWire::crc8(scratchpad, 8) != scratchpad[8])
    return TEMP_ERROR;

  uint8_t shift[] = {3, 2, 1, 0};
  int16_t raw = word(scratchpad[1], scratchpad[0]);
  raw >>= shift[_quality-9];

  return raw * _resolutions[_quality-9];
}

// Read temperature from device
// Argument: Pointer to an array in flash memory of device address
// Return: temperature in degrees Celsius
// If the temperature is TEMP_ERROR value - measurement failed because:
// - the bus is physically damaged
// - devices not respond
// - when data from the device is not valid
// - when not detect device of thad address
float DS18B20::readTemperature(const __FlashStringHelper *_address)
{
  uint8_t address[8];
  _readFlashAddress(_address, address);

  return readTemperature(address);
}

// Set the resolution (conversion factor) for current quality
// Argument: resolution (conversion factor)
// Return: void
void DS18B20::setResolution(float resolution)
{
    _resolutions[_quality-9] = resolution;
}

// private methods
bool DS18B20::_sendCommand(uint8_t *address, uint8_t command)
{
  if (!_oneWire->reset())
    return false;

  _oneWire->select(address);
  _oneWire->write(command);

  return true;
}

bool DS18B20::_sendQuality(uint8_t *address)
{
  if (!_sendCommand(address, 0x4e))
    return false;

  _oneWire->write(0);
  _oneWire->write(0);

  uint8_t quality = _quality;
  quality -= 9;
  quality <<= 5;
  quality |= 0b00011111;
  _oneWire->write(quality);

  return true;
}

bool DS18B20::_receivePowerType(uint8_t *address)
{
  _sendCommand(address, 0xb4);

  return _oneWire->read();
}

void DS18B20::_readFlashAddress(const __FlashStringHelper *_address, uint8_t *address)
{
  const uint8_t *pgmAddress PROGMEM = (const uint8_t PROGMEM *) _address;

  for (uint8_t i=0; i<8; i++)
  {
    address[i] = pgm_read_byte(pgmAddress++);
  }
}
