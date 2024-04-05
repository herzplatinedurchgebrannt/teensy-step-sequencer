#include <Arduino.h>
#include <Wire.h>
#include <U8glib.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SSD1306.h>
#include <drumsi_mcp23017.h>
#include <drumsi_pattern.h>
#include <drumsi_menu.h>


// #define IODIRA 0x00   
// #define IODIRB 0x01   
// #define IOCONA 0x0A   
// #define IOCONB 0x0B  
// #define INTCAPA 0x10  
// #define INTCAPB 0x11  
// #define INTCONA 0x08  
// #define INTCONB 0x09  
// #define INTFA 0x0E    
// #define INTFB 0x0F
// #define GPINTENA 0x04 
// #define GPINTENB 0x05
// #define DEFVALA 0x06  
// #define DEFVALB 0x07
// #define IPOLA 0x02	 
// #define GPIOA 0x12    
// #define GPIOB 0x13
// #define INTPOL 1	
// #define INTODR 2
// #define MIRROR 6	
// #define GPPUA 0x0C	
// #define GPPUB 0x0D

#define MCP_ADDRESS_1 0x20 // (A2/A1/A0 = LOW) 
#define MCP_ADDRESS_2 0x21


// --- PIN CONFIGURATION ---
const int ENC_PIN_A = 6;
const int ENC_PIN_B = 7;

const int MCP_PIN_INT_A = 26;
const int MCP_PIN_INT_B = 28;

const byte MIDI_START = 250;
const byte MIDI_CONTINUE = 251;
const byte MIDI_STOP = 252;
const byte MIDI_CLOCK = 248;

const int SSD_PIN_RESET = 13;

const int BUTTON_PLAY_PIN = 2;
const int BUTTON_PLAY_LED = 38;
const int BUTTON_TRACK_PIN = 3;
const int BUTTON_TRACK_LED = 39;

const int RESET_PIN = 26;

volatile byte buttonPressed = 0;


bool displayValueChange = false;

bool invertMenu = true;
int menuActivePattern = 0;
int menuActiveSelection = 1;


unsigned long menuInvertedLastStamp = 0;

int encValue = HIGH;
int encValueOld = HIGH;
volatile bool encButtonIsActive = false;
unsigned long encButtonLastStamp = 0;

byte midiCounter = 0;
float midiLastStamp = 0;
float noteLength = 4.000;
float bpm = 120.000;
float offset = 0;
float tempo = (1000.000/(bpm/60*noteLength));
float bpmClock = 0;
int changeTempo = false;

unsigned long lastTime = 0;
unsigned long lastTimeTrack = 0;
unsigned long lastTimeStartStop = 0;

unsigned long lastInterrupt = 0;

bool unknownFlag = true;


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

volatile int encoderPos = 0;  // a counter for the dial
int lastReportedPos = 1;   // change management

// interrupt service routine vars
bool A_set = false;            
bool B_set = false;


// create singleton
MCP23017& mcp = MCP23017::getInstance();
DisplayMenu& menu = DisplayMenu::getInstance();



// MCP
uint8_t mcpRead (byte mcpAdress, byte registerAdress);
void mcpWrite (byte mcpAdress, byte registerAdress, byte registerValues);

// MENU
void markMenu(int test);

// SSD1306
void draw(void);

// Player
void beatClock(byte realtimebyte);
void sendMidiNotes(byte spur, byte schritt);

// buttons
void digitalWriteMCP(byte stepNummer, bool anOderAus);
void buttonInterruptA();
void buttonInterruptB();
void buttonsAbfragen(byte woGedrueckt);
void trackInterrupt();

// leds
void seqTrackToLED(byte trackNr);
void seqLauflicht (byte schrittNr);

// Pattern
void seqNoteSchreiben(byte noteInBits, int mcpNummer);
void togglePlaybackState();
void loadPreset (int whichPreset);

// encoder
void encoderSwitch();


IntervalTimer myTimer;

int actTrack = 0;
int actStep = 0;
int N_TRACKS = 8;
int N_STEPS = 16;



void timer() 
{
  // usbMIDI.sendNoteOn(midiNotes[0][0], 127, midiChannelDisplay);
  // usbMIDI.sendNoteOff(midiNotes[0][0], 127, midiChannelDisplay);

  // update LEDs


  digitalWrite(BUTTON_PLAY_LED, false);  





  for (int i = 0; i < N_STEPS; i++)
  {
    Serial.println("Value");
    Serial.println(DUMMY_PATTERN_A[2][i]);
    Serial.println(i);



    // read pattern
    if (DUMMY_PATTERN_A[2][i] == 1){
      digitalWriteMCP(i,1);
    }
    else{
      digitalWriteMCP(i,0);
    }
  }


  // }



  

  // if (DUMMY_PATTERN_A[0][actStep] == 1){ digitalWriteMCP(actStep,0);}
  // else {digitalWriteMCP(actStep,1);}

  // for (int i=0; i<=15; i++) {
  //   digitalWriteMCP(i,pattern[actTrack][i]);
  // }



  // send midi notes
  for (int i=0; i < N_TRACKS; i++)
  {
    if (DUMMY_PATTERN_A[i][actStep] == 1) {
      usbMIDI.sendNoteOn(midiNotes[i][0], 127, 1);
      usbMIDI.sendNoteOff(midiNotes[i][0], 127, 1);

      // Serial.println(actStep);
    }
  }

  if (actStep < N_STEPS - 1) actStep++;
  else actStep = 0;
}


void setup() {






  // --- TIMER INTERRUPT ---
  // myTimer.priority(0);
  //myTimer.begin(timer, 250000);

  // Wire.begin();
  //Serial.begin(31250); 
  Serial.begin(115200); 

  // --- DISPLAY ---
  U8GLIB_SSD1306_ADAFRUIT_128X64 u8g(10, 9, 12, 11, 13); // SW SPI Com: SCK = 10, MOSI = 9, CS = 12, DC = 11, RST = 13

  // --- MENU LOGIC ---
  DisplayMenu::MenuSelection menuPosition = DisplayMenu::MenuSelection::SPUR;


  //Wire.begin();
  Serial.begin(31250); 
  
  // /************  MCP23017 Setup  *************/
  // pinMode(MCP_PIN_INT_A, INPUT);
  // attachInterrupt(digitalPinToInterrupt(MCP_PIN_INT_A), buttonInterruptA, FALLING);

  // mcpWrite(MCP_ADDRESS_1, IODIRA,   B00000000);    // IO Direction Register, 1=Input, 0=Output, LEDs als Output
  // mcpWrite(MCP_ADDRESS_1, GPIOA,    B11111111);    // LEDs anschalten
  // delay(250); 
  // mcpWrite(MCP_ADDRESS_1, GPIOA,    B00000000);    // LEDs ausschalten
  // delay(250);
  // mcpWrite(MCP_ADDRESS_1, IOCONA,   B00000000);   // set InterruptPinPol Interrupt bei LOW-Signal
  // mcpWrite(MCP_ADDRESS_1, IOCONB,   B00000000);
  // delay(10);
  // mcpWrite(MCP_ADDRESS_1, IODIRB,   B11111111);   // IO Direction Register: 1=Input, 0=Output, Buttons als Input
  // mcpWrite(MCP_ADDRESS_1, GPINTENB, B11111111);   // Interrupt-on-change Control Register: 0=Disable, 1=Enable, alle B-Ports haben für die Buttons Interrupts
  // mcpWrite(MCP_ADDRESS_1, INTCONB,  B11111111);   // Interrupt Control Register: Bedingung mit welcher Interrupt ausgelöst wird, 0=InterruptOnChange, 1=InterruptOnDefValDeviation
  // mcpWrite(MCP_ADDRESS_1, DEFVALB,  B11111111);   // Default Value Register: Wenn der Wert im GPIO-Register von diesem Wert abweicht, wird ein Interrupt ausgelöst. In diesem Fall lösen die Interrupts bei einem LOW Signal aus -> =0
  // mcpWrite(MCP_ADDRESS_1, GPPUB,    B11111111);   // Pull-up Widerstände für Buttons aktivieren

  // pinMode(MCP_PIN_INT_B, INPUT);
  // attachInterrupt(digitalPinToInterrupt(MCP_PIN_INT_B), buttonInterruptB, FALLING);

  // mcpWrite(MCP_ADDRESS_2, IODIRA,   B00000000);    // IO Direction Register, 1=Input, 0=Output, LEDs als Output
  // mcpWrite(MCP_ADDRESS_2, GPIOA,    B11111111);    // LEDs anschalten
  // delay(250); 
  // mcpWrite(MCP_ADDRESS_2, GPIOA,    B00000000);    // LEDs ausschalten
  // delay(250);
  // mcpWrite(MCP_ADDRESS_2, IOCONA,   B00000000);   // set InterruptPinPol Interrupt bei LOW-Signal
  // mcpWrite(MCP_ADDRESS_2, IOCONB,   B00000000);
  // delay(10);
  // mcpWrite(MCP_ADDRESS_2, IODIRB,   B11111111);   // IO Direction Register: 1=Input, 0=Output, Buttons als Input
  // mcpWrite(MCP_ADDRESS_2, GPINTENB, B11111111);   // Interrupt-on-change Control Register: 0=Disable, 1=Enable, alle B-Ports haben für die Buttons Interrupts
  // mcpWrite(MCP_ADDRESS_2, INTCONB,  B11111111);   // Interrupt Control Register: Bedingung mit welcher Interrupt ausgelöst wird, 0=InterruptOnChange, 1=InterruptOnDefValDeviation
  // mcpWrite(MCP_ADDRESS_2, DEFVALB,  B11111111);   // Default Value Register: Wenn der Wert im GPIO-Register von diesem Wert abweicht, wird ein Interrupt ausgelöst. In diesem Fall lösen die Interrupts bei einem LOW Signal aus -> =0
  // mcpWrite(MCP_ADDRESS_2, GPPUB,    B11111111);   // Pull-up Widerstände für Buttons aktivieren


  // --- SHIFT REGISTER ---
  mcp.begin();
  // register A
  pinMode(MCP_PIN_INT_A, INPUT);
  attachInterrupt(digitalPinToInterrupt(MCP_PIN_INT_A), buttonInterruptA, FALLING);
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
  // register B
  pinMode(MCP_PIN_INT_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(MCP_PIN_INT_B), buttonInterruptB, FALLING);
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


  // play button    // usbMIDI.sendNoteOn(midiNotes[0][0], 127, midiChannelDisplay);
    // usbMIDI.sendNoteOff(midiNotes[0][0], 127, midiChannelDisplay);
  pinMode(BUTTON_PLAY_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PLAY_PIN), togglePlaybackState, FALLING);  
  pinMode(BUTTON_PLAY_LED, OUTPUT); // LED BUTTON 1 -> StopSeq
  digitalWrite(BUTTON_PLAY_LED, LOW);

  // wtf is this?
  pinMode(40, OUTPUT); // LED BUTTON 1 -> StopSeq
  digitalWrite(40, LOW);

  // switch track button
  pinMode(BUTTON_TRACK_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_TRACK_PIN), trackInterrupt, FALLING);  
  pinMode(BUTTON_TRACK_LED, OUTPUT); // LED Button 2 -> Trackchange

  /************  Internal LED Setup  *************/
  // wtf is this?
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  /************  Reset PIN Setup  *************/
  pinMode(RESET_PIN, INPUT);

  // usbMIDI.setHandleRealTimeSystem(beatClock);
  
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT);
  pinMode(ENC_PIN_A, INPUT);
  pinMode(ENC_PIN_B, INPUT); 


  // // assign default color value
  // if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
  //   u8g.setColorIndex(255); // white
  // } 
  // else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
  // u8g.setColorIndex(3); // max intensity
  // } 
  // else if ( u8g.getMode() == U8G_MODE_BW ) {
  //   u8g.setColorIndex(1); // pixel on
  // }
  // else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
  //   u8g.setHiColorByRGB(255,255,255);
  // }

  digitalWriteMCP(3, true);
} 



void loop() {
  
  #pragma region DISPLAY
  // // picture loop
  // u8g.firstPage(); 
  // do 
  // {
  //   draw(); 
  //   if (encButtonIsActive == true){
  //     markMenu(menuActiveSelection);
  //   }
  //   else
  //   {
  //     if (millis() - menuInvertedLastStamp > 500)
  //     {
  //       markMenu(menuActiveSelection);
  //     }
  //   }

    
  // }
  // while (u8g.nextPage());

  // encValue = digitalRead(6);

  // if ((encValue == LOW) && (encValueOld == HIGH)) {
  //   lastReportedPos = encoderPos;

  //   if (digitalRead(7) == HIGH) {
  //     encoderPos++;
  //   }
  //   else {
  //     encoderPos--;
  //   }Serial.println(encoderPos);
  // } 
  // encValueOld = encValue;

  // if (millis() - encButtonLastStamp > 300 && digitalRead(5) == LOW)
  // {
  //   encButtonIsActive = !encButtonIsActive;
  //   digitalWrite(13, encButtonIsActive);
  //   encButtonLastStamp = millis();
  // }


  // if (displayValueChange == true) { displayValueChange = false; } 

  // // Display aktualisieren
  // if (encButtonIsActive == true )
  // {  
  //   if (encoderPos != lastReportedPos)
  //   {
  //     if (encoderPos-lastReportedPos < 0)
  //     {
  //     menuActiveSelection = menuActiveSelection - 1;
  //     if (menuActiveSelection <= 0) { menuActiveSelection = 6; }
  //     }
  //     else 
  //     {
  //     menuActiveSelection = menuActiveSelection + 1;
  //     if (menuActiveSelection > 6) { menuActiveSelection = 1; }
  //     } 
  //   }
  //   lastReportedPos = encoderPos;
  // }  
  // // Display aktualisieren, Button nicht gedrückt, Parameter lassen sich ändern
  // else if ((encButtonIsActive == false  && displayValueChange == true) || (encButtonIsActive == false  && millis() - menuInvertedLastStamp >= 1000))
  // {
  //   if (invertMenu == true)
  //   {
  //     // unmark Menu BÖSE
  //     //markMenuInt(0);
  //   }
  //   else
  //   { // BÖSE
  //     //markMenuInt(menuAktuell);
  //       markMenu(menuActiveSelection);
  //   }
  //   // Display invertieren nach 500ms
  //   if (millis() - menuInvertedLastStamp >= 500)
  //   {
  //     invertMenu = !invertMenu;
  //     menuInvertedLastStamp = millis();
  //   }
  //   displayValueChange = false;
  // }

  // // Hier werden die Parameter der einzelnen Funktionen verändert
  // if (lastReportedPos != encoderPos && encButtonIsActive == false)
  // {
  //   switch (menuActiveSelection)
  //       {
  //         case 1:
  //           seqSpurAktiv = seqSpurAktiv + encoderPos - lastReportedPos;
  //           if (seqSpurAktiv > 200) { seqSpurAktiv = 0; }
  //           else if (seqSpurAktiv > 7 && seqSpurAktiv <= 200) { seqSpurAktiv = 7; }
  //           midiNoteDisplay = midiNotes[seqSpurAktiv][0];
  //           displayValueChange = true;
  //           break;
  //         case 2:
  //           menuActivePattern = menuActivePattern + encoderPos - lastReportedPos;
  //           if (menuActivePattern < 1){ menuActivePattern = 1; }
  //           else if (menuActivePattern > 8){ menuActivePattern = 8; }
  //           loadPreset(menuActivePattern+8);
  //           displayValueChange = true;
  //           break;
  //         case 3:
  //           bpm = bpm + encoderPos - lastReportedPos;
  //           if (bpm < 60) { bpm = 60; }
  //           else if (bpm > 240) { bpm = 240; }
  //           tempo = (1000.000 / (bpm / 60 * noteLength)) - offset;
  //           displayValueChange = true;          
  //           break;
  //         case 4:
  //           midiChannelDisplay = midiChannelDisplay + encoderPos - lastReportedPos;
  //           if (midiChannelDisplay <= 0) { midiChannelDisplay = 1; }
  //           else if (midiChannelDisplay > 16) { midiChannelDisplay = 16; }
  //           displayValueChange = true;
  //           break;
  //         case 5:
  //           midiNoteDisplay = midiNoteDisplay + encoderPos - lastReportedPos;
  //           if (midiNoteDisplay < 0) { midiNoteDisplay = 0; }
  //           else if (midiNoteDisplay > 127) { midiNoteDisplay = 127; }
  //           midiNotes[seqSpurAktiv][0] = midiNoteDisplay;
  //           displayValueChange = true;
  //           break;
  //         case 6:
  //           midiVelocityDisplay = midiVelocityDisplay + encoderPos - lastReportedPos;
  //           if (midiVelocityDisplay < 0) { midiVelocityDisplay = 127; }
  //           else if (midiVelocityDisplay > 127) { midiVelocityDisplay = 0; }
  //           displayValueChange = true;
  //           break;
  //         default:
  //           break;
  //       }   
  //       lastReportedPos = encoderPos;
  // }

  // rotating = true;  // reset the debouncer
  #pragma endregion

  // LED wird angeschaltet damit Shift-Funktion für Benutzer angedeutet wird
  if (changeTrack == true){
    digitalWrite(BUTTON_TRACK_LED, HIGH);
  }

  // ChangeTrack Button2 ist aktiv und ein Auswahlbutton wurde gedrückt. B1-8=> Spurwechselm B9-16=> Patternwechsel
  if (changeTrack == true && lastButtonPressed != 0)
  {
    if (lastButtonPressed <= 8)
    {
      seqSpurAktiv = lastButtonPressed-1;
      midiNoteDisplay = midiNotes[seqSpurAktiv][0];

      lastTimeTrack = millis();
      changeTrack = false;

      sendOkay = false;                // in diesem Durchlauf darf Controller keine Note mehr schicken aufgrund von ChangeTrack
      digitalWrite(BUTTON_TRACK_LED, LOW);           // LED von Button2 wird ausgeschaltet, dadurch wird Trackwechsel signalisiert
      lastButtonPressed = 0;
    }
    
    else if (lastButtonPressed > 8 && lastButtonPressed <= 16)
    {
      loadPreset (lastButtonPressed);  
      menuActivePattern = lastButtonPressed -8;

      lastTimeTrack = millis();
      changeTrack = false;

      sendOkay = false;                // in diesem Durchlauf darf Controller keine Note mehr schicken aufgrund von ChangeTrack
      digitalWrite(39, LOW);  
      lastButtonPressed = 0;
    }
  }

    // MCPs müssen ausgelesen werden um die Interrupts zurück zu setzen, ansonsten bleiben MCPs stehen
    mcp.read(MCP23017::ADDRESS_1,MCP23017::INTCAPB);
    mcp.read(MCP23017::ADDRESS_2,MCP23017::INTCAPB);

    // // MCPs müssen ausgelesen werden um die Interrupts zurück zu setzen, ansonsten bleiben MCPs stehen
    // mcpRead(MCP_ADDRESS_1,INTCAPB);
    // mcpRead(MCP_ADDRESS_2,INTCAPB);

    // usbMIDI.read();

    // Hier ist die Zeitschleife
    if (millis()-lastTime >= tempo  && startStopInterrupt == false && unknownFlag == true)
    {    
      // LEDs von der aktuell angewählten Spur werden angezeigt
      seqTrackToLED(seqSpurAktiv);

      // Lauflichteffekt
      seqLauflicht(seqStepAktuell);

      // Midi Noten raus schicken per USB
      //sendMidiNotes(seqSpurAktiv, seqStepAktuell);

      // Schritt hochzählen
      seqStepAktuell = seqStepAktuell + 1;
      if (seqStepAktuell == 16){ seqStepAktuell = 0;}

      // lastTime vllt mal an Anfang der Schleife ausprobieren für stabileres Timing?!?!?!
      lastTime = millis();
    }

    if (buttonPressed != 0 && sendOkay == true)
    {
      buttonsAbfragen(buttonPressed);
      buttonPressed = 0;    
    }

    if (digitalRead(MCP_PIN_INT_A) == 1 && digitalRead(MCP_PIN_INT_B) == 1)
    {
      sendOkay = true;
    }
}




void encoderSwitch(){
  if (millis() - encButtonLastStamp > 300)
  {
    encButtonIsActive = !encButtonIsActive;
    digitalWrite(40, encButtonIsActive);
    encButtonLastStamp = millis();
  }
}

void sendMidiNotes(byte track, byte step)
{
  for (int i=0; i<=7; i++)
  {
    if (pattern[i][step] == 1) 
    {
    // usbMIDI.sendNoteOn(midiNotes[i][0], velocitySpeicher[i][step], midiChannelDisplay);
    // usbMIDI.sendNoteOff(midiNotes[i][0], velocitySpeicher[i][step], midiChannelDisplay);

    }
  }
}

void loadPreset (int nr){
  switch (nr)
    {
    case 9:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern[i][j] = DUMMY_PATTERN_A[i][j];
          velocitySpeicher[i][j] = velocityP1[i][j];
        }   
      }
      break;
    case 10:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern[i][j] = DUMMY_PATTERN_B[i][j];
          velocitySpeicher[i][j] = velocityP2[i][j];
        }   
      }
      break;
    case 11:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern[i][j] = DUMMY_PATTERN_C[i][j];
          velocitySpeicher[i][j] = velocityP3[i][j];
        }   
      }
      break;
    case 12:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern[i][j] = DUMMY_PATTERN_D[i][j];
          velocitySpeicher[i][j] = velocityP4[i][j];
        }   
      }
      break;
    case 13:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern[i][j] = DUMMY_PATTERN_E[i][j];
          velocitySpeicher[i][j] = velocityP5[i][j];
        }   
      }
      break;
    case 14:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern[i][j] = DUMMY_PATTERN_F[i][j];
          velocitySpeicher[i][j] = velocityP6[i][j];
        }   
      }
      break;
    case 15:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern[i][j] = DUMMY_PATTERN_G[i][j];
          velocitySpeicher[i][j] = velocityP7[i][j];
        }   
      }
      break;
    case 16:
      for (int i=0; i<8; i++){
        for (int j=0; j<16; j++){
          pattern[i][j] = DUMMY_PATTERN_H[i][j];
          velocitySpeicher[i][j] = velocityP8[i][j];
        }   
      }
      break;
    default:
      break;
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
void togglePlaybackState(){
  if ((millis() - lastTimeStartStop >= 300))
  {
    //startStopInterrupt = true;
    startStopInterrupt = !startStopInterrupt;
    digitalWrite(BUTTON_PLAY_LED, startStopInterrupt);
    lastTimeStartStop = millis();
  }
}

// // Buttons werden abgefragt
void buttonsAbfragen(byte identifier) 
{
  byte statusICR = 0;
  byte mcpWahl = 0;

  if (identifier == 1 ) {  mcpWahl = MCP23017::ADDRESS_1; }
  if (identifier == 2 ) {  mcpWahl = MCP23017::ADDRESS_2; }

  statusICR = mcp.read(mcpWahl,MCP23017::INTFB); 
  lastButtonPressed = statusICR;

  if (statusICR != 0) { seqNoteSchreiben(statusICR, mcpWahl); }
}

// void buttonsAbfragen(byte woGedrueckt) 
// {
//   byte statusICR = 0;
//   byte mcpWahl = 0;

//   if (woGedrueckt == 1 ) {  mcpWahl = MCP_ADDRESS_1; }
//   if (woGedrueckt == 2 ) {  mcpWahl = MCP_ADDRESS_2; }

//   statusICR = mcpRead(mcpWahl,INTFB); 
//   lastButtonPressed = statusICR;

//   if (statusICR != 0) { seqNoteSchreiben(statusICR, mcpWahl); }
// }

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
    if (pattern[seqSpurAktiv][x + mcpNummer] ==  1) 
    { 
      pattern[seqSpurAktiv][x + mcpNummer] = 0; 
      sendOkay = false;
      Serial.println("Note 0");
      lastButtonPressed = 0;
    }
    else 
    {
      pattern[seqSpurAktiv][x + mcpNummer] = 1; 
      velocitySpeicher[seqSpurAktiv][x + mcpNummer] = midiVelocityDisplay;
      sendOkay = false;
      Serial.println("Note 1");
      lastButtonPressed = 0;
    }
  }
}

// Button von MPC23017-1 wird gedrückt
void buttonInterruptA(){
  if ( ( millis()-lastInterrupt >= 50) && (buttonPressed == 0))
  {
    buttonPressed = 1;
    lastInterrupt = millis();
  }
}

void buttonInterruptB(){
  if ( ( millis()-lastInterrupt >= 50) && (buttonPressed == 0))
  {
    buttonPressed = 2;
    lastInterrupt = millis();
  }
}

// Invertiert den aktuellen LED-Status, damit ein Lauflichteffekt entsteht
void seqLauflicht (byte schrittNr){
  if (pattern[seqSpurAktiv][schrittNr] == 1){ digitalWriteMCP(schrittNr,0);}
  else {digitalWriteMCP(schrittNr,1);}
}

// Schaltet die LEDs der aktiven Spur so wie im SeqSpeicher an/aus
void seqTrackToLED(byte trackNr) {
  for (int i=0; i<=15; i++) {
    digitalWriteMCP(i,pattern[trackNr][i]);
    }
  }

// // Schreibt in die Register des MCP23017 
void digitalWriteMCP(byte stepNummer, bool state){
  byte status = 0;
  byte pin = 0;
  byte address = 0;

  if ( stepNummer >= 8){ address = MCP23017::ADDRESS_2; }
  else { address = MCP23017::ADDRESS_1; }
  
  status = mcp.read(address, MCP23017::GPIOA);

  if (stepNummer >= 8) { pin = stepNummer-8; }
  else { pin = stepNummer;}

  if (state == 0) { status &= ~(B00000001 << (pin));
  }
  else if (state == 1) { status |= (B00000001 << (pin));
  }

  mcp.write(address, MCP23017::GPIOA, status);
}

// // Schreibt in die Register des MCP23017 
// void digitalWriteMCP(byte stepNummer, boolean anOderAus){
//   byte statusGP = 0;
//   byte pinNumber = 0;
//   byte mcpWahl = 0;

//   if ( stepNummer >= 8){ mcpWahl = MCP_ADDRESS_2; }
//   else { mcpWahl = MCP_ADDRESS_1; }
  
//   statusGP = mcpRead(mcpWahl, GPIOA);

//   if (stepNummer >= 8) { pinNumber = stepNummer-8; }
//   else { pinNumber = stepNummer;}

//   if (anOderAus == 0) { statusGP &= ~(B00000001 << (pinNumber));
//   }
//   else if (anOderAus == 1) { statusGP |= (B00000001 << (pinNumber));
//   }

//   mcpWrite(mcpWahl, GPIOA, statusGP);
// }


// void beatClock(byte realtimebyte) {

//   if(realtimebyte == MIDI_START) { midiCounter = 0; midiLastStamp = millis(); }
//   if(realtimebyte == MIDI_CONTINUE) { midiLastStamp = millis(); }
//   if(realtimebyte == MIDI_STOP) { digitalWrite(13, LOW); }
  
//   if(realtimebyte == MIDI_CLOCK) {
    
//     midiCounter++;
//     if (midiCounter == 97) {midiCounter = 1;}
//     if(midiCounter == 1 || midiCounter == 24 || midiCounter == 48 || midiCounter == 72) { 
//       digitalWrite(13, HIGH);
//     } 
//     if(midiCounter == 5 || midiCounter == 25 || midiCounter == 49 || midiCounter == 73) { 
//       digitalWrite(13, LOW);
//     }
//   }

//   if (midiCounter == 24 || midiCounter == 48 || midiCounter == 72 || midiCounter == 96 ) {
//     bpm = round((60000 / (millis() - midiLastStamp)));

//     changeTempo = true;

//     Serial.print(bpm);
//     Serial.println(" BPM");
//     midiLastStamp = millis();
//   }
// }


#pragma region MOVED TO MCP LIB
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
#pragma endregion


// void draw(void) {
//   //Presetname
//   u8g.setColorIndex(1);
//   u8g.drawBox(0,0,128,16);
//   u8g.setColorIndex(0);
//   u8g.setFont(u8g_font_helvB10);
//   u8g.drawStr( 2, 13, "DRUMSI");
//   //u8g.drawBitmapP( 116, 4, 1, 8, MidiIn);
//   u8g.setColorIndex(1);


//   //Parameternamen
//   u8g.setFont(u8g_font_profont12);
//   u8g.drawStr( 0, 28, "Trck");
//   u8g.setPrintPos(38, 28); 
//   u8g.print(spurNamen[seqSpurAktiv]);

//   u8g.drawStr( 0, 45, "Patt"); //Presetname
//   u8g.setColorIndex(1);
//   u8g.drawBox(0,0,128,16);
//   u8g.setColorIndex(0);
//   u8g.setFont(u8g_font_helvB10);
//   u8g.drawStr( 2, 13, "DRUMSI");
//   //u8g.drawBitmapP( 116, 4, 1, 8, MidiIn);
//   u8g.setColorIndex(1);

//   //Parameternamen
//   u8g.setFont(u8g_font_profont12);
//   u8g.drawStr( 0, 28, "Trck");
//   u8g.setPrintPos(38, 28); 
//   u8g.print(spurNamen[seqSpurAktiv]);

//   u8g.drawStr( 0, 45, "Patt");
//   u8g.setPrintPos(38, 45); 
//   u8g.print(menuActivePattern);

//   //int bpmCut = bpm;

//   u8g.drawStr( 0, 62, "BPM");
//   u8g.setPrintPos(38, 62); 
//   u8g.print(bpm);

//   u8g.drawStr( 72, 28, "Chan");
//   u8g.setPrintPos(110, 28); 
//   u8g.print(midiChannelDisplay);

//   u8g.drawStr( 72, 45, "Note");
//   u8g.setPrintPos(110, 45); 
//   u8g.print(midiNoteDisplay);

//   u8g.drawStr( 72, 62, "Velo");
//   u8g.setPrintPos(110, 62); 
//   u8g.print(midiVelocityDisplay);

//   //Menü
//   u8g.setColorIndex(1);
//   u8g.setColorIndex(0);
//   u8g.setColorIndex(1);

//   u8g.drawStr( 0, 62, "BPM");
//   u8g.setPrintPos(38, 62); 
//   u8g.print(bpm);

//   u8g.drawStr( 72, 28, "Chan");
//   u8g.setPrintPos(110, 28); 
//   u8g.print(midiChannelDisplay);

//   u8g.drawStr( 72, 45, "Note");
//   u8g.setPrintPos(110, 45); 
//   u8g.print(midiNoteDisplay);

//   u8g.drawStr( 72, 62, "Velo");
//   u8g.setPrintPos(110, 62); 
//   u8g.print(midiVelocityDisplay);

//   //Menü
//   u8g.setColorIndex(1);
//   u8g.setColorIndex(0);
//   u8g.setColorIndex(1);
// }


// void markMenu(int test){
//   u8g.setFont(u8g_font_profont12);
//   u8g.setColorIndex(1);

//   switch (test) {
//     case 1:
//       u8g.drawBox(36,17,32,14);
//       u8g.setColorIndex(0);
//       u8g.setPrintPos(38, 28); 
//       u8g.print(spurNamen[seqSpurAktiv]);
//       break; 
//     case 2:
//       u8g.drawBox(36,34,32,14);
//       u8g.setColorIndex(0);
//       u8g.setPrintPos(38, 45); 
//       u8g.print(menuActivePattern);
//       break;
//     case 3:
//       u8g.drawBox(36,51,32,14);
//       u8g.setColorIndex(0);
//       u8g.setPrintPos(38, 62); 
//       u8g.print(bpm);
//       break;
//     case 4:
//       u8g.drawBox(98,17,32,14);
//       u8g.setColorIndex(0);
//       u8g.setPrintPos(110, 28); 
//       u8g.print(midiChannelDisplay);
//       break;
//     case 5:
//       u8g.drawBox(98,34,32,14);
//       u8g.setColorIndex(0);
//       u8g.setPrintPos(110, 45); 
//       u8g.print(midiNoteDisplay);
//       break;
//     case 6:
//       u8g.drawBox(98,51,32,14);
//       u8g.setColorIndex(0);
//       u8g.setPrintPos(110, 62); 
//       u8g.print(midiVelocityDisplay);
//       break;
//     default:
//       break;
//   }
// }