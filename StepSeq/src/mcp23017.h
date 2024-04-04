// myFunctions.h

#ifndef MY_FUNCTIONS_H
#define MY_FUNCTIONS_H

// MCP23017 
const int MCP_IODIRA = 0x00;   
const int MCP_IODIRB = 0x01;
const int MCP_IOCONA = 0x0A;   
const int MCP_IOCONB = 0x0B;
const int MCP_INTCAPA = 0x10;  
const int MCP_INTCAPB = 0x11;  
const int MCP_INTCONA = 0x08;  
const int MCP_INTCONB = 0x09;  
const int MCP_INTFA = 0x0E;    
const int MCP_INTFB = 0x0F;
const int MCP_GPINTENA = 0x04; 
const int MCP_GPINTENB = 0x05;
const int MCP_DEFVALA = 0x06;  
const int MCP_DEFVALB = 0x07;
const int MCP_IPOLA = 0x02;	 
const int MCP_GPIOA = 0x12;    
const int MCP_GPIOB = 0x13;
const int MCP_INTPOL = 1;	
const int MCP_INTODR = 2;
const int MCP_MIRROR = 6;	
const int MCP_GPPUA = 0x0C;	
const int MCP_GPPUB = 0x0D;
const int MCP_ADDRESS_1 = 0x20; // (A2/A1/A0 = LOW) 
const int MCP_ADDRESS_2 = 0x21;

// Deklaration der Funktionen oder Variablen
void myFunction();

#endif
