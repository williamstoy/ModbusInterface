#ifndef ModbusInterface_h
  #define ModbusInterface_h
  #include <Arduino.h>
  #include <ArduinoRS485.h>
  #include <ArduinoModbus.h>

  class ModbusInterface {
    private:
      HardwareSerial& _serial; // Use a reference here, not a value
      bool _verbose;
      float _bitduration, _wordlen, _preDelayBR, _postDelayBR;
      bool _inErrorState;
    public:
            ModbusInterface(HardwareSerial& serial, bool verbose);
      void  begin(int RS485baudrate, int RS485config);
      bool  writeHoldingRegisterValues(int address, int startingRegisterAddress, uint16_t *data, int dataLength);
      bool  writeHoldingRegisterValue(int address, int registerAddress, uint16_t dataByte);
      bool  readHoldingRegisterValues(int address, int startingRegisterAddress, int nValues, uint16_t *response);
      bool  readHoldingRegisterValue(int address, int registerAddress, uint16_t *response);
  };
#endif