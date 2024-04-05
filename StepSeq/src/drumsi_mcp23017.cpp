// myFunctions.cpp

#include "drumsi_mcp23017.h"

MCP23017& MCP23017::getInstance() {
  static MCP23017 instance;
  return instance;
}

MCP23017::MCP23017() {
  // Constructor code here
}

void MCP23017::setValues(int a, int b) {
  // Method implementation here
}


uint8_t MCP23017::read (byte mcpAdress, byte registerAdress){
  Wire.beginTransmission(mcpAdress);
  Wire.write(registerAdress);
  Wire.endTransmission();
  Wire.requestFrom(mcpAdress, 1);
  return Wire.read();
}

void MCP23017::begin()
{
  Wire.begin();
}

void MCP23017::write(byte mcpAdress, byte registerAdress, byte registerValues)
{
    Wire.beginTransmission(mcpAdress);
    Wire.write(registerAdress);
    Wire.write(registerValues);
    Wire.endTransmission();
}

// void MCP23017::setup(int PIN_INT_A, int PIN_INT_B){
//   pinMode(PIN_INT_A, INPUT);
//   attachInterrupt(digitalPinToInterrupt(PIN_INT_A), buttonInterrupt0, FALLING);
//   write(ADDRESS_1, IODIRA,   B00000000);    // IO Direction Register, 1=Input, 0=Output, LEDs als Output
//   write(ADDRESS_1, GPIOA,    B11111111);    // LEDs anschalten
//   delay(250); 
//   write(ADDRESS_1, GPIOA,    B00000000);    // LEDs ausschalten
//   delay(250);
//   write(ADDRESS_1, IOCONA,   B00000000);   // set InterruptPinPol Interrupt bei LOW-Signal
//   write(ADDRESS_1, IOCONB,   B00000000);
//   delay(10);
//   write(ADDRESS_1, IODIRB,   B11111111);   // IO Direction Register: 1=Input, 0=Output, Buttons als Input
//   write(ADDRESS_1, GPINTENB, B11111111);   // Interrupt-on-change Control Register: 0=Disable, 1=Enable, alle B-Ports haben für die Buttons Interrupts
//   write(ADDRESS_1, INTCONB,  B11111111);   // Interrupt Control Register: Bedingung mit welcher Interrupt ausgelöst wird, 0=InterruptOnChange, 1=InterruptOnDefValDeviation
//   write(ADDRESS_1, DEFVALB,  B11111111);   // Default Value Register: Wenn der Wert im GPIO-Register von diesem Wert abweicht, wird ein Interrupt ausgelöst. In diesem Fall lösen die Interrupts bei einem LOW Signal aus -> =0
//   write(ADDRESS_1, GPPUB,    B11111111);   // Pull-up Widerstände für Buttons aktivieren

//   pinMode(PIN_INT_B, INPUT);
//   attachInterrupt(digitalPinToInterrupt(PIN_INT_B), buttonInterrupt0, FALLING);

//   write(ADDRESS_2, IODIRA,   B00000000);    // IO Direction Register, 1=Input, 0=Output, LEDs als Output
//   write(ADDRESS_2, GPIOA,    B11111111);    // LEDs anschalten
//   delay(250); 
//   write(ADDRESS_2, GPIOA,    B00000000);    // LEDs ausschalten
//   delay(250);
//   write(ADDRESS_2, IOCONA,   B00000000);   // set InterruptPinPol Interrupt bei LOW-Signal
//   write(ADDRESS_2, IOCONB,   B00000000);
//   delay(10);
//   write(ADDRESS_2, IODIRB,   B11111111);   // IO Direction Register: 1=Input, 0=Output, Buttons als Input
//   write(ADDRESS_2, GPINTENB, B11111111);   // Interrupt-on-change Control Register: 0=Disable, 1=Enable, alle B-Ports haben für die Buttons Interrupts
//   write(ADDRESS_2, INTCONB,  B11111111);   // Interrupt Control Register: Bedingung mit welcher Interrupt ausgelöst wird, 0=InterruptOnChange, 1=InterruptOnDefValDeviation
//   write(ADDRESS_2, DEFVALB,  B11111111);   // Default Value Register: Wenn der Wert im GPIO-Register von diesem Wert abweicht, wird ein Interrupt ausgelöst. In diesem Fall lösen die Interrupts bei einem LOW Signal aus -> =0
//   // mcpWrite(MCP_ADDRESS_2, MCP_GPPUB,    B11111111);   // Pull-up Widerstände für Buttons aktivieren
// }


// void buttonInterrupt0(){
//   if ( ( millis() >= 50))
//   {

//   }
// }

