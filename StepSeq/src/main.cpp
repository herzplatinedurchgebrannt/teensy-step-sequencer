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
ButtonState mcp1ButtonState = BTN_OFF;
ButtonState mcp2ButtonState = BTN_OFF;
ButtonState selectButtonState = BTN_OFF; 
ButtonState functionButtonState = BTN_OFF;
// timestamps for debouncing
unsigned long mcpAStamp = 0;
unsigned long mcpBStamp = 0;
unsigned long pauseStamp = 0;
unsigned long selectStamp = 0;
unsigned long functionStamp = 0;

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
int encPinAState = HIGH;
int encPinAStateOld = HIGH;

volatile int encoderWert = 0;
int encoderWertAlt = 0;
volatile bool switchPressed = false;
unsigned long lastTimeSwitchPressed = 0;

volatile int encoderPos = 0;  // a counter for the dial
int lastReportedPos = 1;   // change management

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
  pinMode(BTN_PLAY_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_PLAY_PIN), pauseButtonPressed, FALLING);  
  pinMode(BTN_PLAY_LED, OUTPUT);
  digitalWrite(BTN_PLAY_LED, LOW);

  // select button (2. button top left)
  pinMode(BTN_SELECT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_SELECT_PIN), selectButtonPressed, FALLING);  
  pinMode(BTN_SELECT_LED, LOW);

  // function pattern button (3. button top left)
  pinMode(BTN_FUNCTION_PIN, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(BTN_FUNCTION_PIN), functionButtonPressed, FALLING);  
  digitalWrite(BTN_FUNCTION_LED, LOW);

  // internal teensy led
  // not used atm
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // reset pins
  pinMode(RST_PIN, INPUT);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT);
  pinMode(ENC_PIN_A, INPUT);
  pinMode(ENC_PIN_B, INPUT); 

  // encoder
  pinMode(ENC_PIN_A, INPUT); 
  pinMode(ENC_PIN_B, INPUT); 

 
  // ******************************************
  // ************* DISPLAY SETUP **************
  // ******************************************
  pinMode(ST_SDCS, INPUT_PULLUP);  // don't touch the SD card
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(1);

  drawPlayerState();
  drawTrackNum();
  drawPatternNum();
  drawTempo();
  drawSequencerGrid(10);
  drawActiveTrack();
  drawActiveSteps();
} 


void loop() {

  // **************************************
  //                PLAYBACK
  // **************************************
  if (millis() - tempoStamp >= tempoInMs)
  {
    tempoStamp = millis();

    if (playerState == PLAYER_PLAYING) 
    {
      // led sequence
      seqTrackToLED(actTrack);
      runLedEffect(actStep);
      drawActiveSteps();

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

  // **************************************
  //            MCP REGISTER A
  // **************************************
  // step button as state machine [released, pressed, holding]
  if (mcp1ButtonState == BTN_CLICKED)
  {
    int buttonId = identifyStepButton(1);

    // if select AND function buttons are not pressed
    // behave as default step sequencer
    if (selectButtonState == BTN_OFF && functionButtonState == BTN_OFF)
    {

        updatePatternStep(buttonId);
        drawActiveSteps();
    }
    // if select button is pressed change track or pattern
    else if (selectButtonState == BTN_PRESSED && functionButtonState == BTN_OFF)
    {
        
        actTrack = buttonId;
        selectButtonState = BTN_OFF;
        digitalWrite(BTN_SELECT_LED, false);
        drawSequencerGrid(0);
        drawTrackNum();
        drawActiveTrack();
    }
    // if function button is pressed, additional functions are available
    else if (selectButtonState == BTN_OFF && functionButtonState == BTN_PRESSED)
    {
      switch (buttonId)
      {
        case 0:
          clearActiveTrack();
          break;
        case 1:
          halfsVar1ActTrack();
          break;
        case 2:
          halfsVar2ActTrack();  
          break;
        case 3: 
          quarterVar1ActTrack();
          break;
        case 4:
          quarterVar2ActTrack();
          break;
        case 5:
          eigthsOnActiveTrack();
          break;
        case 6:

          break;
        case 7:

          break;
        default:
          break;
      }
      functionButtonState = BTN_OFF;
      digitalWrite(BTN_FUNCTION_LED, false);
    }
    // if both buttons are pressed
    else
    {
      // hmmmm no idea for now ?!
      // set back to default states
      selectButtonState = BTN_OFF;
      functionButtonState = BTN_OFF;
    }

    // logic handled, set state to holding
    // wait for release button
    mcp1ButtonState = BTN_PRESSED;
  }

  // release mcp1 step button
  // make it accessible again
  if (mcp1ButtonState == BTN_PRESSED)
  {
    // check if pin of mcp A is actually released
    if (digitalRead(MCP_PIN_INT_A) == 1 && millis() - mcpAStamp > 50) 
    {
      mcp1ButtonState = BTN_OFF;
      mcpAStamp = millis();
    }
  }

  // **************************************
  //            MCP REGISTER B
  // **************************************
  // step button as state machine [released, pressed, holding]

  if (mcp2ButtonState == BTN_CLICKED)
  {
    int buttonId = identifyStepButton(2);
    int prevPattern = actPattern;

    // if select AND function buttons are not pressed
    // behave as default step sequencer
    if (selectButtonState == BTN_OFF && functionButtonState == BTN_OFF)
    {
        updatePatternStep(buttonId);
        drawActiveSteps();
    }
    // if select button is pressed change track or pattern
    else if (selectButtonState == BTN_PRESSED && functionButtonState == BTN_OFF)
    {
      // change active pattern  
      // ...
      actPattern = buttonId - 8;
      selectButtonState = BTN_OFF;
      writePreset(prevPattern);
      loadPreset(actPattern);
      digitalWrite(BTN_SELECT_LED, false);
      drawPatternNum();
      drawSequencerGrid(0);
      drawTrackNum();
      drawActiveTrack();
    }
    // if function button is pressed, additional functions are available
    else if (selectButtonState == BTN_OFF && functionButtonState == BTN_PRESSED)
    {
      switch (buttonId - 8)
      {
        case 0:
          clearActivePattern();
          drawNoSteps();
          break;
        case 1:
          changeMidiOutActTrack();
          break;
        case 2:
          savePatternToSdCard();
          break;
        case 3: 
          getTimingByMidiHost();
          break;
        case 4:
          
          break;
        case 5:
          
          break;
        case 6:

          break;
        case 7:

          break;
        default:
          break;
      }
      functionButtonState = BTN_OFF;
      digitalWrite(BTN_FUNCTION_LED, false);
    }
    // if both buttons are pressed
    else
    {
      // hmmmm no idea for now ?!
      // set back to default states
      selectButtonState = BTN_OFF;
      functionButtonState = BTN_OFF;
    }

    // logic handled, set state to holding
    // wait for release button
    mcp2ButtonState = BTN_PRESSED;
  }

  // release mcp 2 step button
  // make it accessible again
  if (mcp2ButtonState == BTN_PRESSED)
  {
    // check if pin of mcp B is actually released
    if (digitalRead(MCP_PIN_INT_B) == 1 && millis() - mcpBStamp > 50)
    {
      mcp2ButtonState = BTN_OFF;
      mcpBStamp = millis();
    }
  }

  // **************************************
  //             SELECT BUTTON
  // **************************************
  if (selectButtonState == BTN_CLICKED) 
  {
    selectButtonState = BTN_PRESSED;
  }

  // **************************************
  //            ROTARY ENCODER
  // **************************************
  encPinAState = digitalRead(ENC_PIN_A);

  if ((encPinAState == LOW) && (encPinAStateOld == HIGH)) 
  {
    if (digitalRead(ENC_PIN_B) == HIGH) 
    {
      encoderPos++;
      actBpm++;
      updateTempo();
    }
    else 
    {
      actBpm--;
      updateTempo();
      encoderPos--;
    }
    Serial.println(actBpm);
    drawTempo();
  } 
  encPinAStateOld = encPinAState;
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

  if (playerState == PLAYER_STOPPED)
  {
    actStep = 0;
    digitalWrite(BTN_PLAY_LED, false);
    playerState = PLAYER_PLAYING;
    Serial.println("START");
    drawPlayerState();

  } 
  else
  {
    digitalWrite(BTN_PLAY_LED, true);
    playerState = PLAYER_STOPPED;
    Serial.println("STOP");
    drawPlayerState();
  }
}

/// @brief update sequencer tempo
/// @param bpm 
void updateTempo(){
  if (actBpm < 60)
  {
    actBpm = 60;
  }
  else if (actBpm > 220)
  {
    actBpm = 220;
  }
  else
  {
    // go on
  }

  int noteLength = 2;
  tempoInMs = (1000.000/(actBpm/60*noteLength));
}

/// @brief interrupt when button of register A
/// was pressed. sets a flag which can be
/// analysed in main loop
void mcpAButtonPressedInterrupt()
{
  if (millis() - mcpAStamp < 50 || mcp1ButtonState != BTN_OFF ) return;
  mcpAStamp = millis();
  mcp1ButtonState = BTN_CLICKED;
}

/// @brief interrupt when button of register B
/// was pressed. sets a flag which can be
/// analysed in main loop
void mcpBButtonPressedInterrupt()
{
  if (millis() - mcpBStamp < 50 || mcp2ButtonState != BTN_OFF ) return;
  mcpBStamp = millis();
  mcp2ButtonState = BTN_CLICKED;
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

void updatePatternStep(int buttonId)
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

void selectButtonPressed()
{
  if ((millis() - selectStamp < 50)) return;

  if (selectButtonState == BTN_OFF)
  {
    selectButtonState = BTN_PRESSED;
    digitalWrite(BTN_SELECT_LED, true);
  }
  else 
  {
    selectButtonState = BTN_OFF;
    digitalWrite(BTN_SELECT_LED, false);
  }

  selectStamp = millis();
}

void functionButtonPressed()
{
  if ((millis() - functionStamp < 50)) return;

  if (functionButtonState == BTN_OFF)
  {
    functionButtonState = BTN_PRESSED;
    digitalWrite(BTN_FUNCTION_LED, true);
  }
  else 
  {
    functionButtonState = BTN_OFF;
    digitalWrite(BTN_FUNCTION_LED, false);
  }

  functionStamp = millis();
}

void encoderTurned(){

}

void encoderPressed(){


}

void drawPlayerState() {
  tft.setTextSize(1);
  tft.fillRect(0, 0, 40, 20, ST7735_BLUE);
  tft.setTextColor(ST7735_BLACK);
  tft.setTextWrap(true);
  tft.setCursor(8, 8);

  switch (playerState)
  {
    case PLAYER_PLAYING:
      tft.print("PLAY");
      break;
    case PLAYER_STOPPED:
      tft.print("STOP");
      break; 
    default:
      break;
    }
}

void drawTrackNum() {
  char num[2];

  itoa(actTrack, num, 10);
  
  tft.setTextSize(1);
  tft.fillRect(40, 0, 40, 20, ST7735_RED);
  tft.setTextColor(ST7735_BLACK);
  tft.setTextWrap(true);
  tft.setCursor(56, 8);
  tft.print(num);
}

void drawPatternNum() {
  char num[2];

  itoa(actPattern, num, 10);
  
  tft.setTextSize(1);
  tft.fillRect(80, 0, 40, 20, ST7735_GREEN);
  tft.setTextColor(ST7735_BLACK);
  tft.setTextWrap(true);
  tft.setCursor(96, 8);
  tft.print(num);
}

void drawTempo() {
  char num[4];

  itoa(actBpm, num, 10);
  
  tft.setTextSize(1);
  tft.fillRect(120, 0, 40, 20, ST7735_ORANGE);
  tft.setTextColor(ST7735_BLACK);
  tft.setTextWrap(true);
  tft.setCursor(130, 8);
  tft.print(num);
}

void drawShiftFunction() {
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextWrap(true);
}

void drawSequencerGrid(int delayInMs = 0){
  int x = 0;  
  int y = 0;   

  for (int i = 0; i < N_TRACKS; i++)
  {
    for (int j = 0; j < N_STEPS; j++)
    {
      tft.drawRect(x + ST_X0_OFFSET, y + ST_Y0_OFFSET, ST_STEP_WIDTH, ST_STEP_HEIGHT, ST7735_CYAN);
      x = x + ST_STEP_WIDTH + ST_MARGIN_1PX;
      delay(delayInMs);
    }
    x = 0;
    y = y + ST_STEP_HEIGHT + ST_MARGIN_1PX;
  }
}

void drawActiveTrack(){
  int x = 0;  
  int y = actTrack * (ST_STEP_HEIGHT + ST_MARGIN_1PX);  

  for (int j = 0; j < N_STEPS; j++)
  {
    tft.drawRect(x + ST_X0_OFFSET, y + ST_Y0_OFFSET, ST_STEP_WIDTH, ST_STEP_HEIGHT, ST7735_YELLOW);
    x = x + ST_STEP_WIDTH + ST_MARGIN_1PX;
  }
  x = 0;
  y = y + ST_STEP_HEIGHT + ST_MARGIN_1PX;
}

void drawActiveSteps(){
  for (int i=0; i < N_TRACKS; i++)
  {
    for (int j = 0; j < N_STEPS; j++)
    {
      if (pattern[i][j] == true){
        drawSequencerStep(i, j, true);
      }
      else{
        drawSequencerStep(i, j, false);
      }
    }
  }
}

void drawNoSteps(){
  for (int i=0; i < N_TRACKS; i++)
  {
    for (int j = 0; j < N_STEPS; j++)
    {
      drawSequencerStep(i, j, false);
    }
  }
}

void drawSequencerStep(int track, int step, bool fill){

  int x = step * (ST_STEP_WIDTH + ST_MARGIN_1PX) + ST_MARGIN_1PX;
  int y = track * (ST_STEP_HEIGHT + ST_MARGIN_1PX) + ST_MARGIN_1PX;

  if (fill){
    tft.fillRect(ST_X0_OFFSET + x, ST_Y0_OFFSET + y, ST_STEP_WIDTH - 2, ST_STEP_HEIGHT - 2, ST7735_RED);
  }
  else{
  tft.fillRect(ST_X0_OFFSET + x, ST_Y0_OFFSET + y, ST_STEP_WIDTH - 2, ST_STEP_HEIGHT - 2, ST7735_BLACK);

  }
}


void writePreset (int patternNr){
  switch (patternNr)
    {
    case 0:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern0[i][j] = pattern[i][j];
        }   
      }
      break;
    case 1:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern1[i][j] = pattern[i][j];
        }   
      }
      break;
    case 2:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern2[i][j] = pattern[i][j];
        }   
      }
      break;
    case 3:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern3[i][j] = pattern[i][j];
        }   
      }
      break;
    case 4:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern4[i][j] = pattern[i][j];
        }   
      }
      break;
    case 5:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern5[i][j] = pattern[i][j];
        }   
      }
      break;
    case 6:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern6[i][j] = pattern[i][j];
        }   
      }
      break;
    case 7:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern7[i][j] = pattern[i][j];
        }   
      }
      break;
    default:
      break;
    }
}

void loadPreset (int patternNr){
  switch (patternNr)
    {
    case 0:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern[i][j] = pattern0[i][j];
        }   
      }
      break;
    case 1:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern[i][j] = pattern1[i][j];
        }   
      }
      break;
    case 2:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern[i][j] = pattern2[i][j];
        }   
      }
      break;
    case 3:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern[i][j] = pattern3[i][j];
        }   
      }
      break;
    case 4:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern[i][j] = pattern4[i][j];
        }   
      }
      break;
    case 5:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern[i][j] = pattern5[i][j];
        }   
      }
      break;
    case 6:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern[i][j] = pattern6[i][j];
        }   
      }
      break;
    case 7:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern[i][j] = pattern7[i][j];
        }   
      }
      break;
    default:
      break;
    }

    actPattern = patternNr;
}



// additional functions by function button
void clearActivePattern()
{
  for (int i=0; i < N_TRACKS; i++)
  {
    for (int j = 0; j < N_STEPS; j++)
    {
      pattern[i][j] = 0;
    }
  }
  Serial.println("CLEAR ACTIVE PATTERN");
};
void changeMidiOutActTrack()
{
  Serial.println("CHANGE MIDI OUT ACT TRACK");
}
void savePatternToSdCard()
{
  Serial.println("SAVE PATTERN TO SD CARD");
}
void getTimingByMidiHost()
{
  Serial.println("GET TIMING MIDI HOST");
}
void clearActiveTrack()
{
  for (int j = 0; j < N_STEPS; j++)
  {
    pattern[actTrack][j] = 0;
  }
  Serial.println("CLEAR ACTIVE TRACK");
}
void halfsVar1ActTrack()
{
  bool notes[16] = {1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0};

  for (int j = 0; j < N_STEPS; j++)
  {
    pattern[actTrack][j] = notes[j];
  }
  Serial.println("HALFS VAR 2");
}
void halfsVar2ActTrack()
{
  bool notes[16] = {0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0};

  for (int j = 0; j < N_STEPS; j++)
  {
    pattern[actTrack][j] = notes[j];
  }
  Serial.println("HALFS VAR 1");
}
void quarterVar1ActTrack()
{
  bool notes[16] = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};

  for (int j = 0; j < N_STEPS; j++)
  {
    pattern[actTrack][j] = notes[j];
  }
  Serial.println("QUARTER VAR 1");
};
void quarterVar2ActTrack()
{
  bool notes[16] = {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1};

  for (int j = 0; j < N_STEPS; j++)
  {
    pattern[actTrack][j] = notes[j];
  }
  Serial.println("QUARTER VAR 2");
}
void eigthsOnActiveTrack()
{
  for (int j = 0; j < N_STEPS; j++)
  {
    pattern[actTrack][j] = 1;
  }
  Serial.println("EIGHTHS");
};

