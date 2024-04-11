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

volatile uint8_t buttonPressed = 0;
// int lastButtonPressed = 0;

// display -> options menu logic
bool displayValueChange = false;
bool invertMenu = true;
int menuActivePattern = 0;
int menuActiveSelection = 1;
unsigned long menuInvertedLastStamp = 0;

// encoder -> rotary encoder device
int encValue = HIGH;
int encValueOld = HIGH;
volatile bool encButtonIsActive = false;
unsigned long encButtonLastStamp = 0;
volatile int encoderPos = 0;  // a counter for the dial
int lastReportedPos = 1;   // change management

// playback -> player, midi, leds, sequencer logic
// NEW STUFF
IntervalTimer playbackTimer;
ShiftButtonState buttonState = off;
PlayerState playerState = playing;
StepButtonState stepButtonStateA = released;
StepButtonState stepButtonStateB = released;

float tempoInMs = 250000;

int actTrack = 5;
int actStep = 0;
int N_TRACKS = 8;
int N_STEPS = 16;

volatile bool mcpAPressed = false;
volatile bool mcpBPressed = false;



unsigned long mcpAStamp = 0;
unsigned long mcpBStamp = 0;
unsigned long pauseStamp = 0;
unsigned long shiftAStamp = 0;
unsigned long shiftBStamp = 0;

unsigned long ledStepUpdateStamp = 0;

unsigned long lastTimeTrack = 0;

unsigned long testStamp = 0;

unsigned long tempoStamp = 0;

uint8_t seqSpurAktiv = 0;
uint8_t seqStepAktuell = 0;
volatile uint8_t statusSeqTrackToLED = 0;
volatile uint8_t statusSeqLauflicht = 0;
int midiVelocityDisplay = 127;
int midiNoteDisplay = midiNotes[0][0];
int midiChannelDisplay = midiNotes[0][2];
bool sendOkay = true;
volatile bool changeTrack = false;
volatile bool startStopInterrupt = false;

// unknown
bool unknownFlag = true;
// interrupt service routine vars
bool A_set = false;            
bool B_set = false;
bool start = true;


// create singleton of devices
MCP23017& mcp = MCP23017::getInstance();
DisplayMenu& menu = DisplayMenu::getInstance();
U8GLIB_SSD1306_ADAFRUIT_128X64 u8g(10, 9, 12, 11, 13); // SW SPI Com: SCK = 10, MOSI = 9, CS = 12, DC = 11, RST = 13


void setup() {

  // serial
  Serial.begin(115200); 

  // menu
  //DisplayMenu::MenuSelection menuPosition = DisplayMenu::MenuSelection::SPUR;

  // i2c shift registers A & B
  mcp.begin();
  pinMode(MCP_PIN_INT_A, INPUT);
  //attachInterrupt(digitalPinToInterrupt(MCP_PIN_INT_A), buttonPressedMcpA, FALLING);
  attachInterrupt(MCP_PIN_INT_A, buttonPressedMcpA, FALLING);
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
  attachInterrupt(digitalPinToInterrupt(MCP_PIN_INT_B), buttonPressedMcpB, FALLING);
  attachInterrupt(MCP_PIN_INT_B, buttonPressedMcpB, FALLING);
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
  pinMode(BUTTON_PLAY_LED, OUTPUT); // LED BUTTON 1 -> StopSeq
  digitalWrite(BUTTON_PLAY_LED, LOW);

  // // wtf is this?
  // pinMode(40, OUTPUT); // LED BUTTON 1 -> StopSeq
  // digitalWrite(40, LOW);

  // // switch track button
  // pinMode(BUTTON_TRACK_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(BUTTON_TRACK_PIN), trackInterrupt, FALLING);  
  // pinMode(BUTTON_TRACK_LED, OUTPUT); // LED Button 2 -> Trackchange

  /************  Internal LED Setup  *************/
  // wtf is this?
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  /************  Reset PIN Setup  *************/
  pinMode(RESET_PIN, INPUT);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT);
  pinMode(ENC_PIN_A, INPUT);
  pinMode(ENC_PIN_B, INPUT); 

  // playbackTimer.priority(50);
  // playbackTimer.begin(playback, tempoInMs);

  //write LED output of selected track
  for (int i = 0; i < N_STEPS; i++)
  {
    writeStepLed(i,pattern[actTrack][i]);
    Serial.println("WRITE LED ON");
  }
} 



void loop() {
  // tempo stuff
  if (millis() - tempoStamp >= 250)
  {
    tempoStamp = millis();

    if (playerState == playing) 
    {
      // led stuff
      seqTrackToLED(actTrack);
      seqLauflicht(actStep);
      // send midi to host
      sendMidi();

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
    int id = getPressedButton(1);
    updateSequencer(id);
    stepButtonStateA = holding;
  }
  if (stepButtonStateA == holding)
  {
    if (digitalRead(MCP_PIN_INT_A) == 1 && millis() - mcpAStamp > 50) 
    {
      stepButtonStateA = released;
      mcpAStamp = millis();
      Serial.println("leave A");
      Serial.println(digitalRead(MCP_PIN_INT_A));
    }
  }

  // MCP REGISTER B
  if (stepButtonStateB == pressed){
    int id = getPressedButton(2);
    updateSequencer(id);
    stepButtonStateB = holding;
    }
  if (stepButtonStateB == holding){
    if (digitalRead(MCP_PIN_INT_B) == 1 && millis() - mcpBStamp > 50){
      stepButtonStateB = released;
      mcpBStamp = millis();
      Serial.println("leave B");
      Serial.println(digitalRead(MCP_PIN_INT_B));
      }
    }


}


void sendMidi(){
  for (int i=0; i < N_TRACKS; i++)
  {
    if (pattern[i][actStep] == 1) {
      usbMIDI.sendNoteOn(midiNotes[i][0], 127, 1);
      usbMIDI.sendNoteOff(midiNotes[i][0], 127, 1);
    }
  }
}

// Invertiert den aktuellen LED-Status, damit ein Lauflichteffekt entsteht
void seqLauflicht (byte schrittNr){
  if (pattern[actTrack][schrittNr] == 1){ writeStepLed(schrittNr,0);}
  else {writeStepLed(schrittNr,1);}
}

// Schaltet die LEDs der aktiven Spur so wie im SeqSpeicher an/aus
void seqTrackToLED(byte trackNr) {
  for (int i=0; i<=15; i++) {
    writeStepLed(i,pattern[trackNr][i]);
    }
  }


/*
            TO DO: 
  create note off logic
  on next playback send note off
  send old active notes off when 
  stopping playback
*/

void playback(){
  // increment current step
  if (actStep < N_STEPS - 1) actStep++;
  else actStep = 0;
}

/// @brief toggle player state
void pauseButtonPressed(){
  if (millis() - pauseStamp < 10) return;

  pauseStamp = millis();

  if (playerState == stopped){
    actStep = 0;
    // playbackTimer.begin(playback, tempoInMs);
    digitalWrite(BUTTON_PLAY_LED, false);
    playerState = playing;
    Serial.println("START");
  } 
  else{
    // playbackTimer.end();
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
  playbackTimer.update(tempoInMs);
}



void encoderSwitch(){
  if (millis() - encButtonLastStamp > 300)
  {
    encButtonIsActive = !encButtonIsActive;
    digitalWrite(40, encButtonIsActive);
    encButtonLastStamp = millis();
  }
}

void buttonPressedMcpA()
{
  if (millis() - mcpAStamp < 50 || stepButtonStateA != released ) return;

  Serial.println("int a");

  mcpAStamp = millis();
  stepButtonStateA = pressed;
}

void buttonPressedMcpB()
{
  if (millis() - mcpBStamp < 50 || stepButtonStateB != released ) return;

  Serial.println("int b");

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



int getPressedButton(byte woGedrueckt) 
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
}



// Schreibt Werte in den SeqSpeicher
void updateSequencer(int buttonId)
{
  if (pattern[actTrack][buttonId] == 1) 
  { 
    pattern[actTrack][buttonId] = 0; 
    sendOkay = false;
    Serial.println("Note 0");
  }
  else 
  {
    pattern[actTrack][buttonId] = 1; 
    sendOkay = false;
    Serial.println("Note 1");
  }
}

void writeStepLed(uint8_t stepNummer, bool ledOn){
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

  if (stepNummer == 0){
    Serial.println("STATUS");
    Serial.println(status);
  }

  if (ledOn == false) 
  { 
    status &= ~(B00000001 << (pin));

    if (stepNummer == 0){
      Serial.println("OFF");
      Serial.println(status);
    }
  }
  else 
  { 
    status |= (B00000001 << (pin));
    
    if (stepNummer == 0){
      Serial.println("ON");
      Serial.println(status);
    }
  }

  mcp.write(address, MCP23017::GPIOA, status);
}


// Interrupt, welcher die aktuelle Spur ändert
void trackInterrupt(){
  if (( millis()-lastTimeTrack >= 50) && changeTrack == false)
  {
    changeTrack = true;
    lastTimeTrack = millis();
  }
}
