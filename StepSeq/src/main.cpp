#include <main.h>
#include <Arduino.h>
#include <Wire.h>
#include <U8glib.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SSD1306.h>
#include <Drumsi_MCP23017.h>
#include <Drumsi_Pattern.h>
#include <Drumsi_Menu.h>
#include <wiring.h>

// encoder 
int encValue = HIGH;
int encValueOld = HIGH;
volatile bool encButtonIsActive = false;
unsigned long encButtonLastStamp = 0;
volatile int encoderPos = 0;  
int lastReportedPos = 1; 

// buttons
MCP23017& mcp = MCP23017::getInstance();

ButtonState stepButtonStateA = released;
ButtonState stepButtonStateB = released;
ButtonState shiftTrackState = released; 
ButtonState shiftPatternStateB = released;

volatile bool mcpAPressed = false;
volatile bool mcpBPressed = false;
unsigned long mcpAStamp = 0;
unsigned long mcpBStamp = 0;

// playback
IntervalTimer playbackTimer;
ShiftFunction shiftFunction = off;
PlayerState playerState = playing;

int prevNotesPlayed[] = {false, false, false, false, false, false, false, false};

float tempoInMs = 250;
int actTrack = 2;
int actStep = 0;


unsigned long pauseStamp = 0;
unsigned long shiftAStamp = 0;
unsigned long shiftBStamp = 0;
unsigned long lastTimeTrack = 0;
unsigned long tempoStamp = 0;

DisplayMenu& menu = DisplayMenu::getInstance();
U8GLIB_SSD1306_ADAFRUIT_128X64 u8g(10, 9, 12, 11, 13); // SW SPI Com: SCK = 10, MOSI = 9, CS = 12, DC = 11, RST = 13


void setup() {

  // serial
  Serial.begin(115200); 

  // i2c shift registers A & B
  mcp.begin();
  pinMode(MCP_PIN_INT_A, INPUT);
  //attachInterrupt(digitalPinToInterrupt(MCP_PIN_INT_A), buttonPressedMcpA, FALLING);
  attachInterrupt(MCP_PIN_INT_A, mcpAButtonPressedInterrupt, FALLING);
  mcp.write(MCP23017::ADDRESS_1, MCP23017::IODIRA,   B00000000);    // IO Direction Register, 1=Input, 0=Output, LEDs als Output
  mcp.write(MCP23017::ADDRESS_1, MCP23017::GPIOA,    B11111111);    // LEDs anschalten
  delay(250); 
  mcp.write(MCP23017::ADDRESS_1, MCP23017::GPIOA,    B00000000);    // LEDs ausschalten
  delay(250);
  mcp.write(MCP23017::ADDRESS_1, MCP23017::IOCONA,   B00000000);   // set InterruptPinPol Interrupt bei LOW-Signal
  mcp.write(MCP23017::ADDRESS_1, MCP23017::IOCONB,   B00000000);
  delay(10);
  mcp.write(MCP23017::ADDRESS_1, MCP23017::IODIRB,   B11111111);   // IO Direction Register: 1=Input, 0=Output, Buttons als Input
  mcp.write(MCP23017::ADDRESS_1, MCP23017::GPINTENB, B11111111);   // Interrupt-on-change Control Register: 0=Disable, 1=Enable, alle B-Ports haben für die Buttons Interrupts
  mcp.write(MCP23017::ADDRESS_1, MCP23017::INTCONB,  B11111111);   // Interrupt Control Register: Bedingung mit welcher Interrupt ausgelöst wird, 0=InterruptOnChange, 1=InterruptOnDefValDeviation
  mcp.write(MCP23017::ADDRESS_1, MCP23017::DEFVALB,  B11111111);   // Default Value Register: Wenn der Wert im GPIO-Register von diesem Wert abweicht, wird ein Interrupt ausgelöst. In diesem Fall lösen die Interrupts bei einem LOW Signal aus -> =0
  mcp.write(MCP23017::ADDRESS_1, MCP23017::GPPUB,    B11111111);   // Pull-up Widerstände für Buttons aktivieren
  pinMode(MCP_PIN_INT_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(MCP_PIN_INT_B), mcpBButtonPressedInterrupt, FALLING);
  attachInterrupt(MCP_PIN_INT_B, mcpBButtonPressedInterrupt, FALLING);
  mcp.write(MCP23017::ADDRESS_2, MCP23017::IODIRA,   B00000000);    // IO Direction Register, 1=Input, 0=Output, LEDs als Output
  mcp.write(MCP23017::ADDRESS_2, MCP23017::GPIOA,    B11111111);    // LEDs anschalten
  delay(250); 
  mcp.write(MCP23017::ADDRESS_2, MCP23017::GPIOA,    B00000000);    // LEDs ausschalten
  delay(250);
  mcp.write(MCP23017::ADDRESS_2, MCP23017::IOCONA,   B00000000);   // set InterruptPinPol Interrupt bei LOW-Signal
  mcp.write(MCP23017::ADDRESS_2, MCP23017::IOCONB,   B00000000);
  delay(10);
  mcp.write(MCP23017::ADDRESS_2, MCP23017::IODIRB,   B11111111);   // IO Direction Register: 1=Input, 0=Output, Buttons als Input
  mcp.write(MCP23017::ADDRESS_2, MCP23017::GPINTENB, B11111111);   // Interrupt-on-change Control Register: 0=Disable, 1=Enable, alle B-Ports haben für die Buttons Interrupts
  mcp.write(MCP23017::ADDRESS_2, MCP23017::INTCONB,  B11111111);   // Interrupt Control Register: Bedingung mit welcher Interrupt ausgelöst wird, 0=InterruptOnChange, 1=InterruptOnDefValDeviation
  mcp.write(MCP23017::ADDRESS_2, MCP23017::DEFVALB,  B11111111);   // Default Value Register: Wenn der Wert im GPIO-Register von diesem Wert abweicht, wird ein Interrupt ausgelöst. In diesem Fall lösen die Interrupts bei einem LOW Signal aus -> =0
  mcp.write(MCP23017::ADDRESS_2, MCP23017::GPPUB,    B11111111);   // Pull-up Widerstände für Buttons aktivieren

  // play button 
  pinMode(BUTTON_PLAY_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PLAY_PIN), pauseButtonPressed, FALLING);  
  pinMode(BUTTON_PLAY_LED, OUTPUT);
  digitalWrite(BUTTON_PLAY_LED, LOW);

  // shift track button
  pinMode(BUTTON_TRACK_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_TRACK_PIN), shiftTrackInterrupt, FALLING);  
  pinMode(BUTTON_TRACK_LED, OUTPUT);

  // shift pattern button
  //pinMode(BUTTON_PATTERN_PIN, OUTPUT); 
  //digitalWrite(BUTTON_PATTERN_LED, LOW);

  // internal teensy led
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  /************  Reset PIN Setup  *************/
  pinMode(RESET_PIN, INPUT);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT);
  pinMode(ENC_PIN_A, INPUT);
  pinMode(ENC_PIN_B, INPUT); 
} 


void loop() {

  /* TODO
  
  REPLACE MILLIS BY MICROS

  */
  if (millis() - tempoStamp >= tempoInMs)
  {
    tempoStamp = millis();

    if (playerState == playing) 
    {
      // led sequence
      seqTrackToLED(actTrack);
      seqLauflicht(actStep);

      // send midi notes to host
      sendMidiNotes();

      // increment current step
      if (actStep < N_STEPS - 1) actStep++;
      else actStep = 0;
    }
  }

  // read frequently to unlock interrupts
  mcp.read(MCP23017::ADDRESS_1,MCP23017::INTCAPB);
  mcp.read(MCP23017::ADDRESS_2,MCP23017::INTCAPB);

  // MCP REGISTER A
  // step button as state machine [released, pressed, holding]
  if (stepButtonStateA == pressed){
    int buttonId = identifyStepButton(1);

    // handle function logic
    switch (shiftFunction)
    {
      case off:
        // no shift function selected
        // default step sequencing
        updatePattern(buttonId);
        break;
      case switchTrackActive:
        // change active track
        actTrack = buttonId;
        shiftFunction = off;
        digitalWrite(BUTTON_TRACK_LED, false);
        break;
      case switchPatternActive:
        // change active pattern
        // ...
        break;    
      default:
        break;
    }

    // logic handled, set state to holding
    // wait for release button
    stepButtonStateA = holding;
  }

  if (stepButtonStateA == holding)
  {
    // check if pin of mcp A is released
    if (digitalRead(MCP_PIN_INT_A) == 1 && millis() - mcpAStamp > 50) 
    {
      stepButtonStateA = released;
      mcpAStamp = millis();
    }
  }

  // MCP REGISTER B
  // step button as state machine [released, pressed, holding]
  if (stepButtonStateB == pressed)
  {
    int buttonId = identifyStepButton(2);

    // handle function logic
    switch (shiftFunction)
    {
      case off:
        // no shift function selected
        // default step sequencing
        updatePattern(buttonId);
        break;
      case switchTrackActive:
        // change active track  
        actTrack = buttonId;
        shiftFunction = off;
        digitalWrite(BUTTON_TRACK_LED, false);
        break;
      case switchPatternActive:
        // change active pattern  
        // ...
        break;    
      default:
        break;
    }

    // logic handled, set state to holding
    // wait for release button
    stepButtonStateB = holding;
  }

  if (stepButtonStateB == holding)
  {
    // check if pin of mcp B is released
    if (digitalRead(MCP_PIN_INT_B) == 1 && millis() - mcpBStamp > 50)
    {
      stepButtonStateB = released;
      mcpBStamp = millis();
    }
  }

  if (shiftTrackState == pressed) 
  {
    shiftTrackState = holding;
  }
}




/*
            TO DO: 
  create note off logic
  on next playback send note off
  send old active notes off when 
  stopping playback
*/
void sendMidiNotes(){
  for (int i=0; i < N_TRACKS; i++)
  {
    // in case previous note was played
    // send note off here
    if (prevNotesPlayed[i] == true)
    {
      usbMIDI.sendNoteOff(midiNotes[i][0], 127, 1);
      prevNotesPlayed[i] = false;
    }

    // if not of pattern is active in this step
    // send out midi note and store state
    if (pattern[i][actStep] == 1) 
    {
      usbMIDI.sendNoteOn(midiNotes[i][0], 127, 1);
      prevNotesPlayed[i] = true;
    }
  }
}

// Invertiert den aktuellen LED-Status, damit ein Lauflichteffekt entsteht
void seqLauflicht (byte schrittNr){
  if (pattern[actTrack][schrittNr] == 1)
  { 
    writeLed(schrittNr,0);
  }
  else 
  {
    writeLed(schrittNr,1);
  }
}

// Schaltet die LEDs der aktiven Spur so wie im SeqSpeicher an/aus
void seqTrackToLED(byte trackNr) 
{
  for (int i=0; i<=15; i++) 
  {
    writeLed(i,pattern[trackNr][i]);
  }
}

/// @brief toggle player state
void pauseButtonPressed(){
  if (millis() - pauseStamp < 10) return;

  pauseStamp = millis();

  if (playerState == stopped){
    actStep = 0;
    digitalWrite(BUTTON_PLAY_LED, false);
    playerState = playing;
    Serial.println("START");
  } 
  else{
    digitalWrite(BUTTON_PLAY_LED, true);
    playerState = stopped;
    Serial.println("STOP");
  }
}

/// @brief update sequencer tempo
/// @param bpm 
void updateTempo(float bpm){
  int noteLength = 4;
  tempoInMs = (1000.000/(bpm/60*noteLength));
}



void encoderSwitch(){
  if (millis() - encButtonLastStamp > 300)
  {
    encButtonIsActive = !encButtonIsActive;
    digitalWrite(40, encButtonIsActive);
    encButtonLastStamp = millis();
  }
}

void mcpAButtonPressedInterrupt()
{
  if (millis() - mcpAStamp < 50 || stepButtonStateA != released ) return;
  mcpAStamp = millis();
  stepButtonStateA = pressed;
}

void mcpBButtonPressedInterrupt()
{
  if (millis() - mcpBStamp < 50 || stepButtonStateB != released ) return;
  mcpBStamp = millis();
  stepButtonStateB = pressed;
}

uint8_t mcpRead (byte mcpAdress, byte registerAdress){
  Wire.beginTransmission(mcpAdress);
  Wire.write(registerAdress);
  Wire.endTransmission();
  Wire.requestFrom(mcpAdress, 1);
  return Wire.read();
}

int identifyStepButton(byte woGedrueckt) 
{
  byte statusICR = 0;
  byte mcpWahl = 0;
  int offset;

  switch (woGedrueckt)
  {
    case 1:
      mcpWahl = MCP23017::ADDRESS_1; 
      offset = 0;
      break;
    case 2:
      mcpWahl = MCP23017::ADDRESS_2; 
      offset = 8;
      break;

    default:
      return -1;
  }

  statusICR = mcpRead(mcpWahl,MCP23017::INTFB); 

  if (statusICR != 0) 
  { 
    byte value = 0;

    while ( bitRead(statusICR, value) == 0) 
    {
      value++;
    }

    return value + offset;
  }
  return -1;
}



// Schreibt Werte in den SeqSpeicher
void updatePattern(int buttonId)
{
  if (pattern[actTrack][buttonId] == 1) 
  { 
    pattern[actTrack][buttonId] = 0; 
  }
  else 
  {
    pattern[actTrack][buttonId] = 1; 
  }
}

void writeLed(uint8_t stepNummer, bool ledOn){
  uint8_t status = 0;
  uint8_t pin = 0;
  uint8_t address = 0;

  if ( stepNummer >= 8)
  { 
    address = MCP23017::ADDRESS_2; 
    pin = stepNummer - 8; 
  }
  else 
  { 
    address = MCP23017::ADDRESS_1; 
    pin = stepNummer;
  }
  status = mcp.read(address, MCP23017::GPIOA);

  if (ledOn == false) 
  { 
    status &= ~(B00000001 << (pin));
  }
  else 
  { 
    status |= (B00000001 << (pin));
  }
  mcp.write(address, MCP23017::GPIOA, status);
}


// Interrupt, welcher die aktuelle Spur ändert
void shiftTrackInterrupt(){
  if ((millis() - lastTimeTrack < 50) && shiftTrackState != released) return;

  shiftTrackState = pressed;

  if (shiftFunction == off)
  {
    digitalWrite(BUTTON_TRACK_LED, true);
    shiftFunction = switchTrackActive;
  }
  else 
  {
    digitalWrite(BUTTON_TRACK_LED, false);
    shiftFunction = off;
  }

  shiftFunction = switchTrackActive;
  lastTimeTrack = millis();
}
