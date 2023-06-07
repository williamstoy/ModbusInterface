#include <Arduino.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

#include <ModbusInterface.h>

// constructor
ModbusInterface::ModbusInterface(HardwareSerial& serial, bool verbose)
: _verbose(verbose), _serial(serial)
{
  _inErrorState = false;
}

void ModbusInterface::begin(int RS485baudrate, int RS485config) {
  // Calculate preDelay and postDelay in microseconds for stable RS-485 transmission
  _bitduration = 1.f / RS485baudrate;
  _wordlen = 10.0f;  // OR 10.0f depending on the channel configuration
  _preDelayBR = _bitduration * _wordlen * 3.5f * 1e6;
  _postDelayBR = _bitduration * _wordlen * 3.5f * 1e6;

  // Start the Modbus RTU client
  RS485.setDelays(_preDelayBR, _postDelayBR);

  if (!ModbusRTUClient.begin(RS485baudrate, RS485config)) {
      if (_verbose) _serial.println("Failed to start Modbus RTU Client!");
      
      _inErrorState = true;

      while (1);
  } else {
    if (_verbose) {
      if (_verbose) _serial.println("Successfully started Modbus RTU Client!");
    }
  }
}



/**
  Writes Holding Register values to the server under specified address.
*/
bool ModbusInterface::writeHoldingRegisterValues(int address, int startingRegisterAddress, uint16_t *data, int dataLength) {
  // Set the Holding Register values to counter
  if (_verbose) _serial.print("Writing Holding Registers values ... ");

  ModbusRTUClient.beginTransmission(address, HOLDING_REGISTERS, startingRegisterAddress, dataLength);

  // Write the data out byte-by-byte
  for (int i = 0; i < dataLength; i++) {
    ModbusRTUClient.write(data[i]);
  }

  if (!ModbusRTUClient.endTransmission()) {
    if (_verbose) {
      _serial.print("WRITE FAILED! ");
      _serial.print("address: ");
      _serial.print(address);
      _serial.print(", startingRegisterAddress: ");
      _serial.print(startingRegisterAddress);
      _serial.print(", dataLength: ");
      _serial.print(dataLength);
      _serial.print(", data: "); 
      for (int i = 0; i < dataLength; i++) {
        _serial.print(data[i]);
        _serial.print(" ");
      }
      _serial.println();
      _serial.println(ModbusRTUClient.lastError());
    }

    _inErrorState = true;

    return false;
  } else {
    if (_verbose) _serial.println("WRITE SUCCESSFUL");

    _inErrorState = false;

    return true;
  }
}



/**
  Write a single Holding Register value to the server under specified address.
*/
bool ModbusInterface::writeHoldingRegisterValue(int address, int registerAddress, uint16_t data) {
  uint16_t dataArray[1] = { data };

  return writeHoldingRegisterValues(address, registerAddress, dataArray, 1);
}



/**
  Reads Holding Register values from the server under specified address.
*/
bool ModbusInterface::readHoldingRegisterValues(int address, int startingRegisterAddress, int nValues, uint16_t *response) {
    if (_verbose) _serial.print("Reading Holding Register values ... ");

    if (!ModbusRTUClient.requestFrom(address, HOLDING_REGISTERS, startingRegisterAddress, nValues)) {
      if (_verbose)  {
        _serial.print("READ FAILED! ");
        _serial.println(ModbusRTUClient.lastError());
      }

      _inErrorState = true;

      return false;
    } else {
      if (_verbose) _serial.println("READ SUCCESSFUL");

      _inErrorState = false;

      // Process the response message
      uint16_t value = 0;
      int responseIndex = 0;
      while (ModbusRTUClient.available()) {
        value = ModbusRTUClient.read(); 

        // only put a value in the response array if it it is less than the expected number of values
        if (responseIndex < nValues) {
          response[responseIndex] = value;
          responseIndex++;
        }
      }

      return true;
    }
}



/**
  Read the Holding Register value at a given register from the server at a specified address.
*/
bool ModbusInterface::readHoldingRegisterValue(int address, int registerAddress, uint16_t *response) {
  return readHoldingRegisterValues(address, registerAddress, 1, response);
}