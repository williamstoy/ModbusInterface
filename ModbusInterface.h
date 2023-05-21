#ifndef ModbusInterface_h
  #define ModbusInterface_h
  #include <Arduino.h>
  #include <ArduinoRS485.h>
  #include <ArduinoModbus.h>

  class ModbusInterface {
    private:
      HardwareSerial& _serial; // Use a reference here, not a value
      bool _verbose;
      float bitduration, wordlen, preDelayBR, postDelayBR;
    public:
            ModbusInterface(HardwareSerial& serial, bool verbose);
      void  begin(int RS485baudrate, int RS485config);
      bool  writeHoldingRegisterValues(int address, int startingRegisterAddress, uint8_t *data, int dataLength);
      bool  writeHoldingRegisterValue(int address, int registerAddress, uint8_t dataByte);
      bool  readHoldingRegisterValues(int address, int startingRegisterAddress, int nValues, int *response);
      bool  readHoldingRegisterValue(int address, int registerAddress, int *response);
  };
#endif