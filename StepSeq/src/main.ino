#include <Arduino.h>
#include <Wire.h>
#include <U8glib.h>

byte variable = 0;
byte randNumber;

#define OLED_RESET 13
U8GLIB_SSD1306_ADAFRUIT_128X64 u8g(10, 9, 12, 11, 13);
// SW SPI Com: SCK = 10, MOSI = 9, CS = 12, DC = 11, RST = 13


/************  MCP23017   ***************/
#define IODIRA 0x00   
#define IODIRB 0x01   
#define IOCONA 0x0A   
#define IOCONB 0x0B  
#define INTCAPA 0x10  
#define INTCAPB 0x11  
#define INTCONA 0x08  
#define INTCONB 0x09  
#define INTFA 0x0E    
#define INTFB 0x0F
#define GPINTENA 0x04 
#define GPINTENB 0x05
#define DEFVALA 0x06  
#define DEFVALB 0x07
#define IPOLA 0x02	 
#define GPIOA 0x12    
#define GPIOB 0x13
#define INTPOL 1	
#define INTODR 2
#define MIRROR 6	
#define GPPUA 0x0C	
#define GPPUB 0x0D

#define MCP_ADDRESS_1 0x20 // (A2/A1/A0 = LOW) 
#define MCP_ADDRESS_2 0x21

int interruptPin = 26;
int interruptPin2 = 28;

/************  Display   ***************/

/*
#include <Adafruit_SSD1306.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_GFX.h>

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
*/

enum menuAnwahl {SPUR, TEMPO, SAVE, PATTERN, MIDI_NOTE, MIDI_VELOCITY};
enum patternAuswahl {PATTERN1, PATTERN2, PATTERN3, PATTERN4, PATTERN5, PATTERN6, PATTERN7, PATTERN8};


int patternDisplay = 0;
int menuAktuell = 1;

menuAnwahl menuPosition = SPUR;
unsigned long lastTimeInvertMenu = 0;

bool displayValueChange = false;


/************  DrehEncoder   **************/
int messungPin1 = HIGH;
int messungPin1Alt = HIGH;

volatile int encoderWert = 0;
int encoderWertAlt = 0;
volatile bool switchPressed = false;
unsigned long lastTimeSwitchPressed = 0;

volatile int encoderWertTest = 0;

/************  MIDI IN   **************/
const byte START = 250;
const byte CONTINUE = 251;
const byte STOP = 252;
const byte CLOCK = 248;

byte zaehler = 0;
float zeitAlt = 0;

/************  Timer and Tempo   ************/
float noteLength = 4.000;
float bpm = 120.000;
float offset = 15;
float tempo = (1000.000/(bpm/60*noteLength))-offset;

float bpmClock = 0;

int changeTempo = false;

unsigned long lastTime = 0;
unsigned long lastTimeTrack = 0;
unsigned long lastTimeStartStop = 0;


volatile byte buttonGedrueckt = 0;
unsigned long lastInterrupt = 0;


/* Noten Speicher und Pattern*/
boolean seqSpeicher[8][16] =   {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} 
                                };



/* PresetPattern - diese werden in den seqSpeicher geladen */
boolean seqSpeicherP1[8][16] = {{1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
                                {0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0},
                                {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} 
                                };

boolean seqSpeicherP2[8][16] = {{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0} 
                                };

boolean seqSpeicherP3[8][16] = {{0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0} 
                                };

boolean seqSpeicherP4[8][16] = {{0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0} 
                                };

boolean seqSpeicherP5[8][16] = {{0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0} 
                                };

boolean seqSpeicherP6[8][16] = {{0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0} 
                                };

boolean seqSpeicherP7[8][16] = {{0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0} 
                                };

boolean seqSpeicherP8[8][16] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} 
                                };



/* Velocity Speicher + Pattern1-8 */
int velocitySpeicher[8][16] =  {{127,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50},
                                {50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50},
                                {50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50},
                                {50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50},
                                {50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50},
                                {50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50},
                                {50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50},
                                {50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50} 
                               };

int velocityP1[8][16] = {{120,120,120,120,120,120,120,120,120,120,120,120,120,120,120,120},
                         {120,120,120,120,120,120,120,120,120,120,120,120,120,120,120,120},
                         {120,120,120,120,120,120,120,120,120,120,120,120,120,120,120,120},
                         {120,120,120,120,120,120,120,120,120,120,120,120,120,120,120,120},
                         {120,120,120,120,120,120,120,120,120,120,120,120,120,120,120,120},
                         {120,120,120,120,120,120,120,120,120,120,120,120,120,120,120,120},
                         {120,120,120,120,120,120,120,120,120,120,120,120,120,120,120,120},
                         {120,120,120,120,120,120,120,120,120,120,120,120,120,120,120,120} 
                        };

int velocityP2[8][16] = {{120,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110},
                         {110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110},
                         {115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115},
                         {115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115},
                         {115,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119},
                         {119,119,119,119,119,119,119,119,119,119,119,119,117,117,117,117},
                         {117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117},
                         {110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110} 
                        };

int velocityP3[8][16] = {{110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110},
                         {110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110},
                         {110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110},
                         {110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110},
                         {110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110},
                         {110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110},
                         {110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110},
                         {110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,127} 
                        };

int velocityP4[8][16] = {{127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127} 
                        };

int velocityP5[8][16] = {{127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127} 
                        };

int velocityP6[8][16] = {{127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127} 
                        };

int velocityP7[8][16] = {{127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127} 
                        };

int velocityP8[8][16] = {{127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127},
                         {127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127} 
                        };

int midiNotes [8][3] = {  {36, 127, 1}, //Kick
                          {38, 127, 1}, //Snare
                          {42, 127, 1}, //Hat
                          {49, 127, 1}, //Crash
                          {48, 127, 1}, //Crash
                          {39, 127, 1}, //Crash
                          {44, 127, 1}, //Crash
                          {35, 127, 1}  //Crash
                          };

String spurNamen [8] = { "Kick", 
                         "Snare",
                         "Hihat",
                         "Crash",
                         "Shakr",
                         "Tom 1",
                         "Tom 2",
                         "Tambu"};

int midiVelocityDisplay = 127;
int midiNoteDisplay = midiNotes[0][0];
int midiChannelDisplay = midiNotes[0][2];

byte seqSpurAktiv = 0;
byte seqStepAktuell = 0;

volatile byte statusSeqTrackToLED = 0;
volatile byte statusSeqLauflicht = 0;

bool sendOkay = true;

volatile bool changeTrack = false;
volatile bool startStopInterrupt = false;

bool start = true;

int lastButtonPressed = 0;



#define encoderPinA 6
#define encoderPinB 7

volatile int encoderPos = 0;  // a counter for the dial
int lastReportedPos = 1;   // change management
static boolean rotating=false;      // debounce management

// interrupt service routine vars
boolean A_set = false;            
boolean B_set = false;



/*------------------------SETUP-----------------------*/

void setup() {

  Wire.begin();
  Serial.begin(31250); 
  
  /************  MCP23017 Setup  *************/
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), buttonInterrupt0, FALLING);

  mcpWrite(MCP_ADDRESS_1, IODIRA,   B00000000);    // IO Direction Register, 1=Input, 0=Output, LEDs als Output
  mcpWrite(MCP_ADDRESS_1, GPIOA,    B11111111);    // LEDs anschalten
  delay(250); 
  mcpWrite(MCP_ADDRESS_1, GPIOA,    B00000000);    // LEDs ausschalten
  delay(250);
  mcpWrite(MCP_ADDRESS_1, IOCONA,   B00000000);   // set InterruptPinPol Interrupt bei LOW-Signal
  mcpWrite(MCP_ADDRESS_1, IOCONB,   B00000000);
  delay(10);
  mcpWrite(MCP_ADDRESS_1, IODIRB,   B11111111);   // IO Direction Register: 1=Input, 0=Output, Buttons als Input
  mcpWrite(MCP_ADDRESS_1, GPINTENB, B11111111);   // Interrupt-on-change Control Register: 0=Disable, 1=Enable, alle B-Ports haben für die Buttons Interrupts
  mcpWrite(MCP_ADDRESS_1, INTCONB,  B11111111);   // Interrupt Control Register: Bedingung mit welcher Interrupt ausgelöst wird, 0=InterruptOnChange, 1=InterruptOnDefValDeviation
  mcpWrite(MCP_ADDRESS_1, DEFVALB,  B11111111);   // Default Value Register: Wenn der Wert im GPIO-Register von diesem Wert abweicht, wird ein Interrupt ausgelöst. In diesem Fall lösen die Interrupts bei einem LOW Signal aus -> =0
  mcpWrite(MCP_ADDRESS_1, GPPUB,    B11111111);   // Pull-up Widerstände für Buttons aktivieren

  pinMode(interruptPin2, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), buttonInterrupt1, FALLING);

  mcpWrite(MCP_ADDRESS_2, IODIRA,   B00000000);    // IO Direction Register, 1=Input, 0=Output, LEDs als Output
  mcpWrite(MCP_ADDRESS_2, GPIOA,    B11111111);    // LEDs anschalten
  delay(250); 
  mcpWrite(MCP_ADDRESS_2, GPIOA,    B00000000);    // LEDs ausschalten
  delay(250);
  mcpWrite(MCP_ADDRESS_2, IOCONA,   B00000000);   // set InterruptPinPol Interrupt bei LOW-Signal
  mcpWrite(MCP_ADDRESS_2, IOCONB,   B00000000);
  delay(10);
  mcpWrite(MCP_ADDRESS_2, IODIRB,   B11111111);   // IO Direction Register: 1=Input, 0=Output, Buttons als Input
  mcpWrite(MCP_ADDRESS_2, GPINTENB, B11111111);   // Interrupt-on-change Control Register: 0=Disable, 1=Enable, alle B-Ports haben für die Buttons Interrupts
  mcpWrite(MCP_ADDRESS_2, INTCONB,  B11111111);   // Interrupt Control Register: Bedingung mit welcher Interrupt ausgelöst wird, 0=InterruptOnChange, 1=InterruptOnDefValDeviation
  mcpWrite(MCP_ADDRESS_2, DEFVALB,  B11111111);   // Default Value Register: Wenn der Wert im GPIO-Register von diesem Wert abweicht, wird ein Interrupt ausgelöst. In diesem Fall lösen die Interrupts bei einem LOW Signal aus -> =0
  mcpWrite(MCP_ADDRESS_2, GPPUB,    B11111111);   // Pull-up Widerstände für Buttons aktivieren

  /************  Button 1 -STOP- Interrupt  *************/
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), stopInterrupt, FALLING);  
  pinMode(38, OUTPUT); // LED BUTTON 1 -> StopSeq
  digitalWrite(38, LOW);


  pinMode(40, OUTPUT); // LED BUTTON 1 -> StopSeq
  digitalWrite(40, LOW);

  /************  Button 2 -CHANGETRACK- Interrupt  *************/
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), trackInterrupt, FALLING);  
  pinMode(39, OUTPUT); // LED Button 2 -> Trackchange

  /************  SSD1306 Setup  *************/
  /*
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // I2C address = 0x3C
  //delay(250);
    
  display.clearDisplay();
  display.fillRect(0,0,128,17,WHITE);
  display.setTextSize(2);     
  display.setTextColor(BLACK);  
  display.setCursor(1, 1);  
  display.print("DRUM SEQ.");

  display.fillRect(110,0,128,16,BLACK);
  display.fillRect(112,0,126,15,WHITE);

  display.drawLine(0,42,128,42, WHITE);
  display.drawLine(42,20,42,64, WHITE);
  display.drawLine(86,20,86,64, WHITE);

  display.display(); 

  //markMenuInt(0);
  //markMenuInt(menuAktuell);

  delay(250);*/

  /************  Internal LED Setup  *************/
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  /************  Reset PIN Setup  *************/
  pinMode(26, INPUT);

  usbMIDI.setHandleRealTimeSystem(beatClock);
  
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT); 

  //digitalWrite(encoderPinA, HIGH);  // turn on pullup resistors
  //digitalWrite(encoderPinB, HIGH);  // turn on pullup resistors

  //attachInterrupt(4, encoderSwitch, FALLING);
  //attachInterrupt(6, doEncoderA, CHANGE); 
  //attachInterrupt(7, doEncoderB, CHANGE); 





 // set SPI backup if required
 //u8g.setHardwareBackup(u8g_backup_avr_spi);

 // assign default color value
 if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
 u8g.setColorIndex(255); // white
 }
 else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
 u8g.setColorIndex(3); // max intensity
 }
 else if ( u8g.getMode() == U8G_MODE_BW ) {
 u8g.setColorIndex(1); // pixel on
 }
 else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
 u8g.setHiColorByRGB(255,255,255);
 }

} 
bool fuck = true;
bool invertMenu = true;


unsigned long lastTimeEncder = 0;



void loop() {


    
  // picture loop
  u8g.firstPage(); 
  do 
  {
    draw(); 
    if (switchPressed == true){
      markMenu(menuAktuell);
    }
    else
    {
      if (millis() - lastTimeInvertMenu > 500)
      {
        markMenu(menuAktuell);
      }
    }

    
  }
  while (u8g.nextPage());

  
  



  messungPin1 = digitalRead(6);


  if ((messungPin1 == LOW) && (messungPin1Alt == HIGH)) {
    lastReportedPos = encoderPos;
    

    if (digitalRead(7) == HIGH) {
      encoderPos++;
    }
    else {
      encoderPos--;
    }Serial.println(encoderPos);
  } 
  messungPin1Alt = messungPin1;




  if (millis() - lastTimeSwitchPressed > 300 && digitalRead(5) == LOW)
  {
    switchPressed = !switchPressed;
    digitalWrite(13, switchPressed);
    lastTimeSwitchPressed = millis();
  }




  if (displayValueChange == true) { displayValueChange = false; } 

  // Display aktualisieren
  if (switchPressed == true )
  {  
    if (encoderPos != lastReportedPos)
    {
      if (encoderPos-lastReportedPos < 0)
      {
      menuAktuell = menuAktuell - 1;
      if (menuAktuell <= 0) { menuAktuell = 6; }
      }
      else 
      {
      menuAktuell = menuAktuell + 1;
      if (menuAktuell > 6) { menuAktuell = 1; }
      } 
    }
    lastReportedPos = encoderPos;
  }  
  // Display aktualisieren, Button nicht gedrückt, Parameter lassen sich ändern
  else if ((switchPressed == false  && displayValueChange == true) || (switchPressed == false  && millis() - lastTimeInvertMenu >= 1000))
  {
    if (invertMenu == true)
    {
      // unmark Menu BÖSE
      //markMenuInt(0);
    }
    else
    { // BÖSE
      //markMenuInt(menuAktuell);
        markMenu(menuAktuell);
    }
    // Display invertieren nach 500ms
    if (millis() - lastTimeInvertMenu >= 500)
    {
      invertMenu = !invertMenu;
      lastTimeInvertMenu = millis();
    }
    displayValueChange = false;
  }

  // Hier werden die Parameter der einzelnen Funktionen verändert
  if (lastReportedPos != encoderPos && switchPressed == false)
  {
    switch (menuAktuell)
        {
          case 1:
            seqSpurAktiv = seqSpurAktiv + encoderPos - lastReportedPos;
            if (seqSpurAktiv > 200) { seqSpurAktiv = 0; }
            else if (seqSpurAktiv > 7 && seqSpurAktiv <= 200) { seqSpurAktiv = 7; }
            midiNoteDisplay = midiNotes[seqSpurAktiv][0];
            displayValueChange = true;
            break;
          case 2:
            patternDisplay = patternDisplay + encoderPos - lastReportedPos;
            if (patternDisplay < 1){ patternDisplay = 1; }
            else if (patternDisplay > 8){ patternDisplay = 8; }
            loadPreset(patternDisplay+8);
            displayValueChange = true;
            break;
          case 3:
            bpm = bpm + encoderPos - lastReportedPos;
            if (bpm < 60) { bpm = 60; }
            else if (bpm > 240) { bpm = 240; }
            tempo = (1000.000 / (bpm / 60 * noteLength)) - offset;
            displayValueChange = true;          
            break;
          case 4:
            midiChannelDisplay = midiChannelDisplay + encoderPos - lastReportedPos;
            if (midiChannelDisplay <= 0) { midiChannelDisplay = 1; }
            else if (midiChannelDisplay > 16) { midiChannelDisplay = 16; }
            displayValueChange = true;
            break;
          case 5:
            midiNoteDisplay = midiNoteDisplay + encoderPos - lastReportedPos;
            if (midiNoteDisplay < 0) { midiNoteDisplay = 0; }
            else if (midiNoteDisplay > 127) { midiNoteDisplay = 127; }
            midiNotes[seqSpurAktiv][0] = midiNoteDisplay;
            displayValueChange = true;
            break;
          case 6:
            midiVelocityDisplay = midiVelocityDisplay + encoderPos - lastReportedPos;
            if (midiVelocityDisplay < 0) { midiVelocityDisplay = 127; }
            else if (midiVelocityDisplay > 127) { midiVelocityDisplay = 0; }
            displayValueChange = true;
            break;
          default:
            break;
        }   
        lastReportedPos = encoderPos;
  }

  rotating = true;  // reset the debouncer
  /*

      if (changeTempo == true){
          display.fillRect(112,0,126,15,WHITE);
          display.setTextSize(1);     
          display.setTextColor(BLACK);  
          display.setCursor(114, 4);  
          display.print("CL");
          display.display(); 
      }
      else {
          display.fillRect(112,0,126,15,WHITE);
          display.setTextSize(1);     
          display.setTextColor(BLACK);  
          display.setCursor(117, 4);  
          display.print("MA");
          display.display(); 
      }
      displayValueChange = true;

      changeTempo = false;
    }*/
    


  // LED wird angeschaltet damit Shift-Funktion für Benutzer angedeutet wird
  if (changeTrack == true){
    digitalWrite(39, HIGH);
  }

  // ChangeTrack Button2 ist aktiv und ein Auswahlbutton wurde gedrückt. B1-8=> Spurwechselm B9-16=> Patternwechsel
  if (changeTrack == true && lastButtonPressed != 0)
  {
    if (lastButtonPressed <= 8)
    {
      seqSpurAktiv = lastButtonPressed-1;
      midiNoteDisplay = midiNotes[seqSpurAktiv][0];

      //displayValueChange = true;
      //markMenuInt(0);

      lastTimeTrack = millis();
      changeTrack = false;

      sendOkay = false;                // in diesem Durchlauf darf Controller keine Note mehr schicken aufgrund von ChangeTrack
      digitalWrite(39, LOW);           // LED von Button2 wird ausgeschaltet, dadurch wird Trackwechsel signalisiert
      lastButtonPressed = 0;
    }
    
    else if (lastButtonPressed > 8 && lastButtonPressed <= 16)
    {
      loadPreset (lastButtonPressed);  
      patternDisplay = lastButtonPressed -8;

      //displayValueChange = true;
      //markMenuInt(0);

      lastTimeTrack = millis();
      changeTrack = false;

      sendOkay = false;                // in diesem Durchlauf darf Controller keine Note mehr schicken aufgrund von ChangeTrack
      digitalWrite(39, LOW);  
      lastButtonPressed = 0;
    }
  }

    // MCPs müssen ausgelesen werden um die Interrupts zurück zu setzen, ansonsten bleiben MCPs stehen
    mcpRead(MCP_ADDRESS_1,INTCAPB);
    mcpRead(MCP_ADDRESS_2,INTCAPB);

    usbMIDI.read();

    // Hier ist die Zeitschleife
    if (millis()-lastTime >= tempo  && startStopInterrupt == false && fuck == true)
    {    
      // LEDs von der aktuell angewählten Spur werden angezeigt
      seqTrackToLED(seqSpurAktiv);

      // Lauflichteffekt
      seqLauflicht(seqStepAktuell);

      // Midi Noten raus schicken per USB
      sendMidiNotes(seqSpurAktiv, seqStepAktuell);

      // Schritt hochzählen
      seqStepAktuell = seqStepAktuell + 1;
      if (seqStepAktuell == 16){ seqStepAktuell = 0;}

      // lastTime vllt mal an Anfang der Schleife ausprobieren für stabileres Timing?!?!?!
      lastTime = millis();
    }

    if (buttonGedrueckt != 0 && sendOkay == true)
    {
      buttonsAbfragen(buttonGedrueckt);
      buttonGedrueckt = 0;    
    }

    if (digitalRead(26) == 1 && digitalRead(28) == 1)
    {
      sendOkay = true;
    }
  }


/*---------------------------ENDE LOOP---------------------------*/

/*
void markMenuInt(int test){

    switch (test) {
      case 0:
          // unmark everything
          display.fillRect(1,29,39,10, BLACK);
          display.setTextSize(1); 
          display.setTextColor(WHITE);  
          display.setCursor(6, 21); 
          display.print("TRACK");
          display.setCursor(2, 30); 
          display.print(spurNamen[seqSpurAktiv]); 

          display.fillRect(45,29,39,10, BLACK);
          display.setTextSize(1); 
          display.setTextColor(WHITE);  
          display.setCursor(56, 21); 
          display.print("BPM");
          display.setCursor(47, 30);
          display.print(bpm); 

          display.fillRect(89,29,86,10, BLACK);
          display.setTextSize(1); 
          display.setTextColor(WHITE);  
          display.setCursor(95, 21); 
          display.print("CHAN.");
          display.setCursor(105, 30); 
          display.print(midiChannelDisplay); 

          display.fillRect(1,55,39,52, BLACK);
          display.setTextSize(1); 
          display.setTextColor(WHITE);  
          display.setCursor(3, 47);  
          display.print("PRESET"); 
          display.setCursor(17, 56); 
          display.print(patternDisplay);

          display.fillRect(45,55,39,52, BLACK);
          display.setTextSize(1); 
          display.setTextColor(WHITE); 
          display.setCursor(52, 47);  
          display.print("NOTE");
          display.setCursor(58, 56);  
          display.print(midiNoteDisplay); 

          display.fillRect(89,55,86,52, BLACK);
          display.setTextSize(1); 
          display.setTextColor(WHITE); 
          display.setCursor(91, 47);  
          display.print("VELOC.");
          display.setCursor(100, 56); 
          display.print(midiVelocityDisplay);  

          display.display(); 
        break;
      case 1:
          display.fillRect(1,29,39,10, WHITE);
          display.setTextSize(1); 
          display.setTextColor(BLACK);  
          display.setCursor(2, 30); 
          display.print(spurNamen[seqSpurAktiv]); 
          display.display(); 
        break; 
      case 2:
          display.fillRect(45,29,39,10, WHITE);
          display.setTextSize(1); 
          display.setTextColor(BLACK);  
          display.setCursor(47, 30);
          display.print(bpm); 
          display.display(); 
        break;
      case 3:
          display.fillRect(89,29,86,10, WHITE);
          display.setTextSize(1); 
          display.setTextColor(BLACK);  
          display.setCursor(105, 30); 
          display.print(midiChannelDisplay); 
          display.display(); 
        break;
      case 4:
          display.fillRect(1,55,39,52, WHITE);
          display.setTextSize(1); 
          display.setTextColor(BLACK);  
          display.setCursor(17, 56); 
          display.print(patternDisplay);
          display.display(); 
        break;
      case 5:
          display.fillRect(45,55,39,52, WHITE);
          display.setTextSize(1); 
          display.setTextColor(BLACK); 
          display.setCursor(58, 56);  
          display.print(midiNoteDisplay); 
          display.display(); 
        break;
      case 6:
          display.fillRect(89,55,86,52, WHITE);
          display.setTextSize(1); 
          display.setTextColor(BLACK); 
          display.setCursor(100, 56);
          display.print(midiVelocityDisplay);  
          display.display(); 
        break;
    }
}*/

void encoderSwitch(){
  if (millis() - lastTimeSwitchPressed > 300)
    switchPressed = !switchPressed;
    digitalWrite(40, switchPressed);
    lastTimeSwitchPressed = millis();
}

// sendet MIDI Noten aus dem aktuellen S
void sendMidiNotes(byte spur, byte schritt)
{
  for (int i=0; i<=7; i++)
  {
    if (seqSpeicher[i][schritt] == 1) 
    {
    usbMIDI.sendNoteOn(midiNotes[i][0], velocitySpeicher[i][schritt], midiChannelDisplay);
    usbMIDI.sendNoteOff(midiNotes[i][0], velocitySpeicher[i][schritt], midiChannelDisplay);

    }
  }
}

void loadPreset (int whichPreset){
  switch (whichPreset)
    {
    case 9:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          seqSpeicher[i][j] = seqSpeicherP1[i][j];
          velocitySpeicher[i][j] = velocityP1[i][j];
        }   
      }
      break;
    case 10:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          seqSpeicher[i][j] = seqSpeicherP2[i][j];
          velocitySpeicher[i][j] = velocityP2[i][j];
        }   
      }
      break;
    case 11:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          seqSpeicher[i][j] = seqSpeicherP3[i][j];
          velocitySpeicher[i][j] = velocityP3[i][j];
        }   
      }
      break;
    case 12:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          seqSpeicher[i][j] = seqSpeicherP4[i][j];
          velocitySpeicher[i][j] = velocityP4[i][j];
        }   
      }
      break;
    case 13:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          seqSpeicher[i][j] = seqSpeicherP5[i][j];
          velocitySpeicher[i][j] = velocityP5[i][j];
        }   
      }
      break;
    case 14:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          seqSpeicher[i][j] = seqSpeicherP6[i][j];
          velocitySpeicher[i][j] = velocityP6[i][j];
        }   
      }
      break;
    case 15:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          seqSpeicher[i][j] = seqSpeicherP7[i][j];
          velocitySpeicher[i][j] = velocityP7[i][j];
        }   
      }
      break;
    case 16:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          seqSpeicher[i][j] = seqSpeicherP8[i][j];
          velocitySpeicher[i][j] = velocityP8[i][j];
        }   
      }
      break;
    default:
      break;
    }
}

void doEncoderA()
{
  if ( rotating ) /*delay (1)*/;  // wait a little until the bouncing is done
  if( digitalRead(encoderPinA) != A_set ) 
  {  // debounce once more
    A_set = !A_set;
    // adjust counter + if A leads B
    if ( A_set && !B_set ) 
      encoderPos += 1;
    rotating = false;  // no more debouncing until loop() hits again
  }
}

// Interrupt on B changing state, same as A above
void doEncoderB()
{
  if ( rotating ) /*delay*/ (1);
  if( digitalRead(encoderPinB) != B_set ) 
  {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if( B_set && !A_set ) 
      encoderPos -= 1;
    rotating = false;
  }
}

// Interrupt, welcher die aktuelle Spur ändert
void trackInterrupt(){
  if (( millis()-lastTimeTrack >= 50) && changeTrack == false)
  {
    changeTrack = true;
    lastTimeTrack = millis();
  }
}

// Interrupt, welcher den Sequencer startet+stoppt
void stopInterrupt(){
  if ((millis() - lastTimeStartStop >= 300))
  {
    //startStopInterrupt = true;
    startStopInterrupt = !startStopInterrupt;
    digitalWrite(38, startStopInterrupt);
    lastTimeStartStop = millis();
  }
}

// Buttons werden abgefragt
void buttonsAbfragen(byte woGedrueckt) 
{
  byte statusICR = 0;
  byte mcpWahl = 0;

  if (woGedrueckt == 1 ) {  mcpWahl = MCP_ADDRESS_1; }
  if (woGedrueckt == 2 ) {  mcpWahl = MCP_ADDRESS_2; }

  statusICR = mcpRead(mcpWahl,INTFB); 
  lastButtonPressed = statusICR;

  if (statusICR != 0) { seqNoteSchreiben(statusICR, mcpWahl); }
}

// Schreibt Werte in den SeqSpeicher
void seqNoteSchreiben(byte noteInBits, int mcpNummer)
{
  byte x = 0;

  if (mcpNummer == 0x21)
  {
    mcpNummer = 8;
  }
  else 
  {
    mcpNummer = 0;
  }

  while ( bitRead(noteInBits, x) == 0) 
  {
    x++;
  }

  lastButtonPressed = x + 1 + mcpNummer;

  if(changeTrack == false)
  {
    if (seqSpeicher[seqSpurAktiv][x + mcpNummer] ==  1) 
    { 
      seqSpeicher[seqSpurAktiv][x + mcpNummer] = 0; 
      sendOkay = false;
      Serial.println("Note 0");
      lastButtonPressed = 0;
    }
    else 
    {
      seqSpeicher[seqSpurAktiv][x + mcpNummer] = 1; 
      velocitySpeicher[seqSpurAktiv][x + mcpNummer] = midiVelocityDisplay;
      sendOkay = false;
      Serial.println("Note 1");
      lastButtonPressed = 0;
    }
  }
}

// Button von MPC23017-1 wird gedrückt
void buttonInterrupt0(){
  if ( ( millis()-lastInterrupt >= 50) && (buttonGedrueckt == 0))
  {
    buttonGedrueckt = 1;
    lastInterrupt = millis();
  }
}

void buttonInterrupt1(){
  if ( ( millis()-lastInterrupt >= 50) && (buttonGedrueckt == 0))
  {
    buttonGedrueckt = 2;
    lastInterrupt = millis();
  }
}

// Invertiert den aktuellen LED-Status, damit ein Lauflichteffekt entsteht
void seqLauflicht (byte schrittNr){
  if (seqSpeicher[seqSpurAktiv][schrittNr] == 1){ digitalWriteMCP(schrittNr,0);}
  else {digitalWriteMCP(schrittNr,1);}
}

// Schaltet die LEDs der aktiven Spur so wie im SeqSpeicher an/aus
void seqTrackToLED(byte trackNr) {
  for (int i=0; i<=15; i++) {
    digitalWriteMCP(i,seqSpeicher[trackNr][i]);
    }
  }

// Schreibt in die Register des MCP23017 
void digitalWriteMCP(byte stepNummer, boolean anOderAus){
  byte statusGP = 0;
  byte pinNumber = 0;
  byte mcpWahl = 0;

  if ( stepNummer >= 8){ mcpWahl = MCP_ADDRESS_2; }
  else { mcpWahl = MCP_ADDRESS_1; }
  
  statusGP = mcpRead(mcpWahl, GPIOA);

  if (stepNummer >= 8) { pinNumber = stepNummer-8; }
  else { pinNumber = stepNummer;}

  if (anOderAus == 0) { statusGP &= ~(B00000001 << (pinNumber));
  }
  else if (anOderAus == 1) { statusGP |= (B00000001 << (pinNumber));
  }

  mcpWrite(mcpWahl, GPIOA, statusGP);
}


void beatClock(byte realtimebyte) {

  if(realtimebyte == START) { zaehler = 0; zeitAlt = millis(); }
  if(realtimebyte == CONTINUE) { zeitAlt = millis(); }
  if(realtimebyte == STOP) { digitalWrite(13, LOW); }
  
  if(realtimebyte == CLOCK) {
    
    zaehler++;
    if (zaehler == 97) {zaehler = 1;}
    if(zaehler == 1 || zaehler == 24 || zaehler == 48 || zaehler == 72) { 
      digitalWrite(13, HIGH);
    } 
    if(zaehler == 5 || zaehler == 25 || zaehler == 49 || zaehler == 73) { 
      digitalWrite(13, LOW);
    }
  }

  if (zaehler == 24 || zaehler == 48 || zaehler == 72 || zaehler == 96 ) {
    bpm = round((60000 / (millis() - zeitAlt)));

    changeTempo = true;

    Serial.print(bpm);
    Serial.println(" BPM");
    zeitAlt = millis();
  }
}


uint8_t mcpRead (byte mcpAdress, byte registerAdress){
  Wire.beginTransmission(mcpAdress);
  Wire.write(registerAdress);
  Wire.endTransmission();
  Wire.requestFrom(mcpAdress, 1);
  return Wire.read();
}

void mcpWrite (byte mcpAdress, byte registerAdress, byte registerValues){
  Wire.beginTransmission(mcpAdress);
  Wire.write(registerAdress); 
  Wire.write(registerValues); 
  Wire.endTransmission();
}


void draw(void) {



 //Presetname
 u8g.setColorIndex(1);
 u8g.drawBox(0,0,128,16);
 u8g.setColorIndex(0);
 u8g.setFont(u8g_font_helvB10);
 u8g.drawStr( 2, 13, "DRUMSI");
 //u8g.drawBitmapP( 116, 4, 1, 8, MidiIn);
 u8g.setColorIndex(1);


 //Parameternamen
 u8g.setFont(u8g_font_profont12);
 u8g.drawStr( 0, 28, "Trck");
 u8g.setPrintPos(38, 28); 
 u8g.print(spurNamen[seqSpurAktiv]);

 u8g.drawStr( 0, 45, "Patt");
 u8g.setPrintPos(38, 45); 
 u8g.print(patternDisplay);

 //int bpmCut = bpm;

 u8g.drawStr( 0, 62, "BPM");
 u8g.setPrintPos(38, 62); 
 u8g.print(bpm);
 
 u8g.drawStr( 72, 28, "Chan");
 u8g.setPrintPos(110, 28); 
 u8g.print(midiChannelDisplay);
 
 u8g.drawStr( 72, 45, "Note");
 u8g.setPrintPos(110, 45); 
 u8g.print(midiNoteDisplay);

 u8g.drawStr( 72, 62, "Velo");
 u8g.setPrintPos(110, 62); 
 u8g.print(midiVelocityDisplay);


 //Menü
 u8g.setColorIndex(1);
 u8g.setColorIndex(0);
 u8g.setColorIndex(1);
 
 //Parameterwerte


/*
 u8g.drawLine(62, 17, 62, 48);
 u8g.drawLine(50, 48, 128, 48);
 u8g.drawLine(50, 48, 50, 64);
 */
}


void markMenu(int test){
  
  u8g.setFont(u8g_font_profont12);
  u8g.setColorIndex(1);

    switch (test) {
      case 1:
        u8g.drawBox(36,17,32,14);
        u8g.setColorIndex(0);
        u8g.setPrintPos(38, 28); 
        u8g.print(spurNamen[seqSpurAktiv]);
        break; 
      case 2:
        u8g.drawBox(36,34,32,14);
        u8g.setColorIndex(0);
        u8g.setPrintPos(38, 45); 
        u8g.print(patternDisplay);
        break;
      case 3:
        u8g.drawBox(36,51,32,14);
        u8g.setColorIndex(0);
        u8g.setPrintPos(38, 62); 
        u8g.print(bpm);
        break;
      case 4:
        u8g.drawBox(98,17,32,14);
        u8g.setColorIndex(0);
        u8g.setPrintPos(110, 28); 
        u8g.print(midiChannelDisplay);
        break;
      case 5:
        u8g.drawBox(98,34,32,14);
        u8g.setColorIndex(0);
        u8g.setPrintPos(110, 45); 
        u8g.print(midiNoteDisplay);
        break;
      case 6:
        u8g.drawBox(98,51,32,14);
        u8g.setColorIndex(0);
        u8g.setPrintPos(110, 62); 
        u8g.print(midiVelocityDisplay);
        break;
      default:
        break;
    }
}