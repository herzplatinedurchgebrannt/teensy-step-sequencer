#ifndef MCP23017_H
#define MCP23017_H

#include <Arduino.h>
#include <Wire.h>

class MCP23017 {
private:
    int name, loves;
    static MCP23017* instancePtr;

    MCP23017();

public:
    MCP23017(const MCP23017&) = delete;

    static MCP23017& getInstance();

    static const int IODIRA = 0x00;   
    static const int IODIRB = 0x01;
    static const int IOCONA = 0x0A;   
    static const int IOCONB = 0x0B;
    static const int INTCAPA = 0x10;  
    static const int INTCAPB = 0x11;  
    static const int INTCONA = 0x08;  
    static const int INTCONB = 0x09;  
    static const int INTFA = 0x0E;    
    static const int INTFB = 0x0F;
    static const int GPINTENA = 0x04; 
    static const int GPINTENB = 0x05;
    static const int DEFVALA = 0x06;  
    static const int DEFVALB = 0x07;
    static const int IPOLA = 0x02;	 
    static const int GPIOA = 0x12;    
    static const int GPIOB = 0x13;
    static const int INTPOL = 1;	
    static const int INTODR = 2;
    static const int MIRROR = 6;	
    static const int GPPUA = 0x0C;	
    static const int GPPUB = 0x0D;
    static const int ADDRESS_1 = 0x20; // (A2/A1/A0 = LOW) 
    static const int ADDRESS_2 = 0x21;

    void setValues(int name, int loves);

    // void setup(int PIN_INT_A, int PIN_INT_B);

    void begin();

    void write(uint8_t mcpAdress, uint8_t registerAdress, uint8_t registerValues);

    // static void buttonInterrupt0();

    uint8_t read(uint8_t mcpAdress, uint8_t registerAdress);

    void print();
};

#endif // SINGLETON_H






