#include <main.h>
#include <Arduino.h>
#include <Wire.h>
#include <Drumsi_MCP23017.h>
#include <Drumsi_Pattern.h>
#include <Drumsi_Menu.h>
#include <wiring.h>
#include <Adafruit_GFX.h> 
#include <Adafruit_ST7735.h> 
#include <SPI.h>

#if defined(__SAM3X8E__)
    #undef __FlashStringHelper::F(string_literal)
    #define F(string_literal) string_literal
#endif

Adafruit_ST7735 tft = Adafruit_ST7735(ST_CS, ST_DC, ST_RST);


// ******************************************
// *********** SEQUENCER BUTTONS ************
// ******************************************
// shift registers
MCP23017& mcp = MCP23017::getInstance();
// button states
ButtonState stepButtonStateA = BTN_OFF;
ButtonState stepButtonStateB = BTN_OFF;
ButtonState selectButtonState = BTN_OFF; 
ButtonState optionButtonState = BTN_OFF;
// timestamps for debouncing
unsigned long mcpAStamp = 0;
unsigned long mcpBStamp = 0;
unsigned long pauseStamp = 0;
unsigned long selectStamp = 0;

// *** PLAYBACK LOGIC ***
PlayerState playerState = PLAYER_PLAYING;
// store state when midi note on was send
int prevNotesPlayed[] = {false, false, false, false, false, false, false, false};
int actTrack = 0;       // active track of sequencer
int actStep = 0;        // active step of sequencer
int actPattern = 0;     // active pattern of sequencer
int actBpm = 120;
float tempoInMs = 250;  // time interval in ms
// timestamp for time interval
unsigned long tempoStamp = 0;

// *** ROTARY ENCODER ***
int encValue = HIGH;
int encValueOld = HIGH;
int lastReportedPos = 1; 
volatile bool encButtonIsActive = false;
unsigned long encButtonLastStamp = 0;
volatile int encoderPos = 0;  

// MENU
DisplayMenu& menu = DisplayMenu::getInstance();

void setup() {

  // serial
  Serial.begin(115200); 

  // ******************************************
  // ******** SEQUENCER BUTTONS SETUP *********
  // ******************************************
  // i2c shift registers A & B
  mcp.begin();
  pinMode(MCP_PIN_INT_A, INPUT);
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

  // ******************************************
  // ******** FUNCTION BUTTONS SETUP **********
  // ******************************************
  // play button (1. button top left)
  pinMode(BUTTON_PLAY_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PLAY_PIN), pauseButtonPressed, FALLING);  
  pinMode(BUTTON_PLAY_LED, OUTPUT);
  digitalWrite(BUTTON_PLAY_LED, LOW);

  // select button (2. button top left)
  pinMode(BUTTON_TRACK_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_TRACK_PIN), selectInterrupt, FALLING);  
  pinMode(BUTTON_TRACK_LED, OUTPUT);

  // option pattern button (3. button top left)
  pinMode(BUTTON_PATTERN_PIN, OUTPUT); 
  digitalWrite(BUTTON_PATTERN_LED, LOW);

  // internal teensy led
  // not used atm
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // reset pins
  pinMode(RESET_PIN, INPUT);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT);
  pinMode(ENC_PIN_A, INPUT);
  pinMode(ENC_PIN_B, INPUT); 


  // ******************************************
  // ************* DISPLAY SETUP **************
  // ******************************************
  pinMode(ST_SDCS, INPUT_PULLUP);  // don't touch the SD card
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(3);

  drawPlayerState();
  drawTrack();
  drawPattern();
  drawTempo();
  drawSequencer();
} 


void loop() {

  /* TODO
  
  REPLACE MILLIS BY MICROS

  */

  // time interval 
  if (millis() - tempoStamp >= tempoInMs)
  {
    tempoStamp = millis();

    if (playerState == PLAYER_PLAYING) 
    {
      // led sequence
      seqTrackToLED(actTrack);
      runLedEffect(actStep);

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
  if (stepButtonStateA == BTN_CLICKED){
    int buttonId = identifyStepButton(1);

    // handle function logic
    switch (selectButtonState)
    {
      case BTN_OFF:
        // no shift function selected
        // default step sequencing
        updatePattern(buttonId);
        break;
      case BTN_PRESSED:
        // change active track
        actTrack = buttonId;
        selectButtonState = BTN_OFF;
        digitalWrite(BUTTON_TRACK_LED, false);
        //drawPlayerState();
        drawTrack();
        break; 
      default:
        break;
    }

    // logic handled, set state to holding
    // wait for release button
    stepButtonStateA = BTN_PRESSED;
  }

  if (stepButtonStateA == BTN_PRESSED)
  {
    // check if pin of mcp A is released
    if (digitalRead(MCP_PIN_INT_A) == 1 && millis() - mcpAStamp > 50) 
    {
      stepButtonStateA = BTN_OFF;
      mcpAStamp = millis();
    }
  }

  // MCP REGISTER B
  // step button as state machine [released, pressed, holding]
  if (stepButtonStateB == BTN_CLICKED)
  {
    int buttonId = identifyStepButton(2);

    // handle function logic
    switch (selectButtonState)
    {
      case BTN_OFF:
        // no shift function selected
        // default step sequencing
        updatePattern(buttonId);
        break;
      case BTN_PRESSED:
        // change active pattern  
        // ...
        break;  
      default:
        break;
    }

    // logic handled, set state to holding
    // wait for release button
    stepButtonStateB = BTN_PRESSED;
  }

  if (stepButtonStateB == BTN_PRESSED)
  {
    // check if pin of mcp B is released
    if (digitalRead(MCP_PIN_INT_B) == 1 && millis() - mcpBStamp > 50)
    {
      stepButtonStateB = BTN_OFF;
      mcpBStamp = millis();
    }
  }

  if (selectButtonState == BTN_CLICKED) 
  {
    selectButtonState = BTN_PRESSED;
  }
}

/// @brief in case there is a midi host like a daw
/// available, send midi notes
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

/// @brief invert the led of the active step
/// to get a running light effect
/// @param step active step of sequencer
void runLedEffect (byte step){
  if (pattern[actTrack][step] == 1)
  { 
    writeLed(step,0);
  }
  else 
  {
    writeLed(step,1);
  }
}

/// @brief writes the led status of active track
/// @param track active track of sequencer 
void seqTrackToLED(byte track) 
{
  for (int i=0; i<=15; i++) 
  {
    writeLed(i,pattern[track][i]);
  }
}

/// @brief toggle player state
void pauseButtonPressed(){
  if (millis() - pauseStamp < 10) return;

  pauseStamp = millis();

  if (playerState == PLAYER_STOPPED){
    actStep = 0;
    digitalWrite(BUTTON_PLAY_LED, false);
    playerState = PLAYER_PLAYING;
    Serial.println("START");
    drawPlayerState();

  } 
  else{
    digitalWrite(BUTTON_PLAY_LED, true);
    playerState = PLAYER_STOPPED;
    Serial.println("STOP");
    drawPlayerState();
  }
}

/// @brief update sequencer tempo
/// @param bpm 
void updateTempo(float bpm){
  int noteLength = 4;
  tempoInMs = (1000.000/(bpm/60*noteLength));
}

// void encoderSwitch(){
//   if (millis() - encButtonLastStamp > 300)
//   {
//     encButtonIsActive = !encButtonIsActive;
//     digitalWrite(40, encButtonIsActive);
//     encButtonLastStamp = millis();
//   }
// }

// void doEncoderA()
// {
//   if ( rotating ) /*delay (1)*/;  // wait a little until the bouncing is done
//   if( digitalRead(encoderPinA) != A_set ) 
//   {  // debounce once more
//     A_set = !A_set;
//     // adjust counter + if A leads B
//     if ( A_set && !B_set ) 
//       encoderPos += 1;
//     rotating = false;  // no more debouncing until loop() hits again
//   }
// }

/// @brief interrupt when button of register A
/// was pressed. sets a flag which can be
/// analysed in main loop
void mcpAButtonPressedInterrupt()
{
  if (millis() - mcpAStamp < 50 || stepButtonStateA != BTN_OFF ) return;
  mcpAStamp = millis();
  stepButtonStateA = BTN_CLICKED;
}

/// @brief interrupt when button of register B
/// was pressed. sets a flag which can be
/// analysed in main loop
void mcpBButtonPressedInterrupt()
{
  if (millis() - mcpBStamp < 50 || stepButtonStateB != BTN_OFF ) return;
  mcpBStamp = millis();
  stepButtonStateB = BTN_CLICKED;
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


// switch track on sequencer
// press shift [control button 2] and step 1-8
void selectInterrupt()
{
  if ((millis() - selectStamp < 50)) return;

  if (selectButtonState == BTN_OFF)
  {
    selectButtonState = BTN_PRESSED;
    digitalWrite(BUTTON_TRACK_LED, true);
  }
  else 
  {
    selectButtonState = BTN_OFF;
    digitalWrite(BUTTON_TRACK_LED, false);
  }

  selectStamp = millis();
}

void drawPlayerState() {
  tft.setCursor(3, 4);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE);
  tft.print(ST_STR_PLAYER);
  tft.fillRect(64, 0, 64, 18, ST7735_BLUE);
  tft.setTextColor(ST7735_BLACK);
  tft.setTextWrap(true);
  tft.setCursor(68, 4);

  switch (playerState)
  {
    case PLAYER_PLAYING:
      tft.print("PLAYING");
      break;
    case PLAYER_STOPPED:
      tft.print("STOPPED");
      break; 
    default:
      break;
    }
}

void drawTrack() {
  char num[2];

  itoa(actTrack, num, 10);
  
  tft.setCursor(3, 24);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE);
  tft.print(ST_STR_TRACK);
  tft.fillRect(64, 20, 64, 18, ST7735_RED);
  tft.setTextColor(ST7735_BLACK);
  tft.setTextWrap(true);
  tft.setCursor(68, 24);
  tft.print(num);
}

void drawPattern() {
  char num[2];

  itoa(actPattern, num, 10);
  
  tft.setCursor(3, 44);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE);
  tft.print(ST_STR_PATTERN);
  tft.fillRect(64, 40, 64, 18, ST7735_GREEN);
  tft.setTextColor(ST7735_BLACK);
  tft.setTextWrap(true);
  tft.setCursor(68, 44);
  tft.print(num);
}

void drawTempo() {
  char num[4];

  itoa(actBpm, num, 10);
  
  tft.setCursor(3, 64);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE);
  tft.print(ST_STR_BPM);
  tft.fillRect(64, 60, 64, 18, ST7735_ORANGE);
  tft.setTextColor(ST7735_BLACK);
  tft.setTextWrap(true);
  tft.setCursor(68, 64);
  tft.print(num);
}

void drawShiftFunction() {
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextWrap(true);
}

void drawSequencer(){

  int x0 = 0;
  int y0 = 90;
  int w = 9;
  int h = 9;

  tft.drawRect( 0, y0, w, h, ST7735_CYAN);
  tft.drawRect(10, y0, w, h, ST7735_CYAN);
  tft.drawRect(20, y0, w, h, ST7735_CYAN);
  tft.drawRect(30, y0, w, h, ST7735_CYAN);
  tft.drawRect(40, y0, w, h, ST7735_CYAN);
  tft.drawRect(50, y0, w, h, ST7735_CYAN);
  tft.drawRect(60, y0, w, h, ST7735_CYAN);
  tft.drawRect(70, y0, w, h, ST7735_CYAN);
  tft.drawRect(80, y0, w, h, ST7735_CYAN);
  tft.drawRect(90, y0, w, h, ST7735_CYAN);
  tft.drawRect(100, y0, w, h, ST7735_CYAN);
  tft.drawRect(110, y0, w, h, ST7735_CYAN);
  tft.drawRect(120, y0, w, h, ST7735_CYAN);
  tft.drawRect(130, y0, w, h, ST7735_CYAN);
  tft.drawRect(140, y0, w, h, ST7735_CYAN);
  tft.drawRect(150, y0, w, h, ST7735_CYAN);
}