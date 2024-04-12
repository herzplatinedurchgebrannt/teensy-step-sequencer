#include <main.h>
#include <Arduino.h>
#include <Wire.h>
// #include <U8glib.h>
// #include <Adafruit_I2CDevice.h>
// #include <Adafruit_SSD1306.h>
#include <Drumsi_MCP23017.h>
#include <Drumsi_Pattern.h>
#include <Drumsi_Menu.h>
#include <wiring.h>

#include <Adafruit_GFX.h> 
#include <Adafruit_ST7735.h> 
#include <SPI.h>

// *** BUTTONS ***
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
int actTrack = 0;       // active track on sequencer
int actStep = 0;        // active step on sequencer
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
//U8GLIB_SSD1306_ADAFRUIT_128X64 u8g(10, 9, 12, 11, 13); // SW SPI Com: SCK = 10, MOSI = 9, CS = 12, DC = 11, RST = 13

void setup() {

  // serial
  Serial.begin(115200); 

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
void runLedEffect (byte schrittNr){
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

  if (playerState == PLAYER_STOPPED){
    actStep = 0;
    digitalWrite(BUTTON_PLAY_LED, false);
    playerState = PLAYER_PLAYING;
    Serial.println("START");
  } 
  else{
    digitalWrite(BUTTON_PLAY_LED, true);
    playerState = PLAYER_STOPPED;
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
  if (millis() - mcpAStamp < 50 || stepButtonStateA != BTN_OFF ) return;
  mcpAStamp = millis();
  stepButtonStateA = BTN_CLICKED;
}

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



/***************************************************
  This is an example sketch for the Adafruit 1.8" SPI display.
  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618
 
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

//This Teensy3 native optimized version requires specific pins





// #include <Adafruit_GFX.h>    // Core graphics library
// #include <Adafruit_ST7735.h> // Hardware-specific library
// #include <SPI.h>

// #if defined(__SAM3X8E__)
//     #undef __FlashStringHelper::F(string_literal)
//     #define F(string_literal) string_literal
// #endif


// int sclk = 13;  // SCLK can also use pin 14
// int mosi = 11;  // MOSI can also use pin 7
// int cs   = 10;  // CS & DC can use pins 2, 6, 9, 10, 15, 20, 21, 22, 23
// int dc   = 9;   //  but certain pairs must NOT be used: 2+10, 6+9, 20+23, 21+22
// int rst  = 8;   // RST can use any pin
// int sdcs = 4;   // CS for SD card, can use any pin

// // Option 1: use any pins but a little slower
// Adafruit_ST7735 tft = Adafruit_ST7735(cs, dc, mosi, sclk, rst);

// // Option 2: must use the hardware SPI pins
// // (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// // an output. This is much faster - also required if you want
// // to use the microSD card (see the image drawing example)
// //Adafruit_ST7735 tft = Adafruit_ST7735(cs, dc, rst);
// float p = 3.1415926;

 
// #define Neutral 0
// #define Press 1
// #define Up 2
// #define Down 3
// #define Right 4
// #define Left 5
 
// // Check the joystick position
// int CheckJoystick()
// {
//   int joystickState = analogRead(3);
  
//   if (joystickState < 50) return Left;
//   if (joystickState < 150) return Down;
//   if (joystickState < 250) return Press;
//   if (joystickState < 500) return Right;
//   if (joystickState < 650) return Up;
//   return Neutral;
// }


// void setup(void) {
//   pinMode(sdcs, INPUT_PULLUP);  // don't touch the SD card
//   Serial.begin(9600);
//   Serial.print("hello!");

//   // Our supplier changed the 1.8" display slightly after Jan 10, 2012
//   // so that the alignment of the TFT had to be shifted by a few pixels
//   // this just means the init code is slightly different. Check the
//   // color of the tab to see which init code to try. If the display is
//   // cut off or has extra 'random' pixels on the top & left, try the
//   // other option!
//   // If you are seeing red and green color inversion, use Black Tab

//   // If your TFT's plastic wrap has a Black Tab, use the following:
//   tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
//   // If your TFT's plastic wrap has a Red Tab, use the following:
//   //tft.initR(INITR_REDTAB);   // initialize a ST7735R chip, red tab
//   // If your TFT's plastic wrap has a Green Tab, use the following:
//   //tft.initR(INITR_GREENTAB); // initialize a ST7735R chip, green tab

//   Serial.println("init");

//   uint16_t time = millis();
//   tft.fillScreen(ST7735_BLACK);
//   time = millis() - time;

//   Serial.println(time, DEC);
//   delay(500);

//   // large block of text
//   tft.fillScreen(ST7735_BLACK);
//   testdrawtext("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", ST7735_WHITE);
//   delay(1000);

//   // tft print function!
//   tftPrintTest();
//   delay(4000);

//   // a single pixel
//   tft.drawPixel(tft.width()/2, tft.height()/2, ST7735_GREEN);
//   delay(500);

//   // line draw test
//   testlines(ST7735_YELLOW);
//   delay(500);

//   // optimized lines
//   testfastlines(ST7735_RED, ST7735_BLUE);
//   delay(500);

//   testdrawrects(ST7735_GREEN);
//   delay(500);

//   testfillrects(ST7735_YELLOW, ST7735_MAGENTA);
//   delay(500);

//   tft.fillScreen(ST7735_BLACK);
//   testfillcircles(10, ST7735_BLUE);
//   testdrawcircles(10, ST7735_WHITE);
//   delay(500);

//   testroundrects();
//   delay(500);

//   testtriangles();
//   delay(500);

//   mediabuttons();
//   delay(500);

//   Serial.println("done");
//   delay(1000);
// }

// void loop() {
//   tft.invertDisplay(true);
//   delay(500);
//   tft.invertDisplay(false);
//   delay(500);
//   int joy = CheckJoystick();
//   switch (joy)
//   {
//     case Left:
//       Serial.println("Left");
//       break;
//     case Right:
//       Serial.println("Right");
//       break;
//     case Up:
//       Serial.println("Up");
//       break;
//     case Down:
//       Serial.println("Down");
//       break;
//     case Press:
//       Serial.println("Press");
//       break;
//   }
// }

// void testlines(uint16_t color) {
//   tft.fillScreen(ST7735_BLACK);
//   for (int16_t x=0; x < tft.width(); x+=6) {
//     tft.drawLine(0, 0, x, tft.height()-1, color);
//   }
//   for (int16_t y=0; y < tft.height(); y+=6) {
//     tft.drawLine(0, 0, tft.width()-1, y, color);
//   }

//   tft.fillScreen(ST7735_BLACK);
//   for (int16_t x=0; x < tft.width(); x+=6) {
//     tft.drawLine(tft.width()-1, 0, x, tft.height()-1, color);
//   }
//   for (int16_t y=0; y < tft.height(); y+=6) {
//     tft.drawLine(tft.width()-1, 0, 0, y, color);
//   }

//   tft.fillScreen(ST7735_BLACK);
//   for (int16_t x=0; x < tft.width(); x+=6) {
//     tft.drawLine(0, tft.height()-1, x, 0, color);
//   }
//   for (int16_t y=0; y < tft.height(); y+=6) {
//     tft.drawLine(0, tft.height()-1, tft.width()-1, y, color);
//   }

//   tft.fillScreen(ST7735_BLACK);
//   for (int16_t x=0; x < tft.width(); x+=6) {
//     tft.drawLine(tft.width()-1, tft.height()-1, x, 0, color);
//   }
//   for (int16_t y=0; y < tft.height(); y+=6) {
//     tft.drawLine(tft.width()-1, tft.height()-1, 0, y, color);
//   }
// }

// void testdrawtext(char *text, uint16_t color) {
//   tft.setCursor(0, 0);
//   tft.setTextColor(color);
//   tft.setTextWrap(true);
//   tft.print(text);
// }

// void testfastlines(uint16_t color1, uint16_t color2) {
//   tft.fillScreen(ST7735_BLACK);
//   for (int16_t y=0; y < tft.height(); y+=5) {
//     tft.drawFastHLine(0, y, tft.width(), color1);
//   }
//   for (int16_t x=0; x < tft.width(); x+=5) {
//     tft.drawFastVLine(x, 0, tft.height(), color2);
//   }
// }

// void testdrawrects(uint16_t color) {
//   tft.fillScreen(ST7735_BLACK);
//   for (int16_t x=0; x < tft.width(); x+=6) {
//     tft.drawRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color);
//   }
// }

// void testfillrects(uint16_t color1, uint16_t color2) {
//   tft.fillScreen(ST7735_BLACK);
//   for (int16_t x=tft.width()-1; x > 6; x-=6) {
//     tft.fillRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color1);
//     tft.drawRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color2);
//   }
// }

// void testfillcircles(uint8_t radius, uint16_t color) {
//   for (int16_t x=radius; x < tft.width(); x+=radius*2) {
//     for (int16_t y=radius; y < tft.height(); y+=radius*2) {
//       tft.fillCircle(x, y, radius, color);
//     }
//   }
// }

// void testdrawcircles(uint8_t radius, uint16_t color) {
//   for (int16_t x=0; x < tft.width()+radius; x+=radius*2) {
//     for (int16_t y=0; y < tft.height()+radius; y+=radius*2) {
//       tft.drawCircle(x, y, radius, color);
//     }
//   }
// }

// void testtriangles() {
//   tft.fillScreen(ST7735_BLACK);
//   int color = 0xF800;
//   int t;
//   int w = 63;
//   int x = 159;
//   int y = 0;
//   int z = 127;
//   for(t = 0 ; t <= 15; t+=1) {
//     tft.drawTriangle(w, y, y, x, z, x, color);
//     x-=4;
//     y+=4;
//     z-=4;
//     color+=100;
//   }
// }

// void testroundrects() {
//   tft.fillScreen(ST7735_BLACK);
//   int color = 100;
//   int i;
//   int t;
//   for(t = 0 ; t <= 4; t+=1) {
//     int x = 0;
//     int y = 0;
//     int w = 127;
//     int h = 159;
//     for(i = 0 ; i <= 24; i+=1) {
//       tft.drawRoundRect(x, y, w, h, 5, color);
//       x+=2;
//       y+=3;
//       w-=4;
//       h-=6;
//       color+=1100;
//     }
//     color+=100;
//   }
// }

// void tftPrintTest() {
//   tft.setTextWrap(false);
//   tft.fillScreen(ST7735_BLACK);
//   tft.setCursor(0, 30);
//   tft.setTextColor(ST7735_RED);
//   tft.setTextSize(1);
//   tft.println("Hello World!");
//   tft.setTextColor(ST7735_YELLOW);
//   tft.setTextSize(2);
//   tft.println("Hello World!");
//   tft.setTextColor(ST7735_GREEN);
//   tft.setTextSize(3);
//   tft.println("Hello World!");
//   tft.setTextColor(ST7735_BLUE);
//   tft.setTextSize(4);
//   tft.print(1234.567);
//   delay(1500);
//   tft.setCursor(0, 0);
//   tft.fillScreen(ST7735_BLACK);
//   tft.setTextColor(ST7735_WHITE);
//   tft.setTextSize(0);
//   tft.println("Hello World!");
//   tft.setTextSize(1);
//   tft.setTextColor(ST7735_GREEN);
//   tft.print(p, 6);
//   tft.println(" Want pi?");
//   tft.println(" ");
//   tft.print(8675309, HEX); // print 8,675,309 out in HEX!
//   tft.println(" Print HEX!");
//   tft.println(" ");
//   tft.setTextColor(ST7735_WHITE);
//   tft.println("Sketch has been");
//   tft.println("running for: ");
//   tft.setTextColor(ST7735_MAGENTA);
//   tft.print(millis() / 1000);
//   tft.setTextColor(ST7735_WHITE);
//   tft.print(" seconds.");
// }

// void mediabuttons() {
//   // play
//   tft.fillScreen(ST7735_BLACK);
//   tft.fillRoundRect(25, 10, 78, 60, 8, ST7735_WHITE);
//   tft.fillTriangle(42, 20, 42, 60, 90, 40, ST7735_RED);
//   delay(500);
//   // pause
//   tft.fillRoundRect(25, 90, 78, 60, 8, ST7735_WHITE);
//   tft.fillRoundRect(39, 98, 20, 45, 5, ST7735_GREEN);
//   tft.fillRoundRect(69, 98, 20, 45, 5, ST7735_GREEN);
//   delay(500);
//   // play color
//   tft.fillTriangle(42, 20, 42, 60, 90, 40, ST7735_BLUE);
//   delay(50);
//   // pause color
//   tft.fillRoundRect(39, 98, 20, 45, 5, ST7735_RED);
//   tft.fillRoundRect(69, 98, 20, 45, 5, ST7735_RED);
//   // play color
//   tft.fillTriangle(42, 20, 42, 60, 90, 40, ST7735_GREEN);
// }



