#include <Arduino.h>
#include <Wire.h>
#include <U8glib.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SSD1306.h>
#include "mcp23017.h"
#include <drumsi_pattern.h>

const int ENC_PIN_A = 6;
const int ENC_PIN_B = 7;

const int MCP_PIN_INT_A = 26;
const int MCP_PIN_INT_B = 28;

const byte MIDI_START = 250;
const byte MIDI_CONTINUE = 251;
const byte MIDI_STOP = 252;
const byte MIDI_CLOCK = 248;

const int SSD_PIN_RESET = 13;

volatile byte buttonPressed = 0;


U8GLIB_SSD1306_ADAFRUIT_128X64 u8g(10, 9, 12, 11, 13); // SW SPI Com: SCK = 10, MOSI = 9, CS = 12, DC = 11, RST = 13
bool displayValueChange = false;

bool invertMenu = true;
int menuActivePattern = 0;
int menuActiveSelection = 1;
enum menuSelection {SPUR, TEMPO, SAVE, PATTERN, MIDI_NOTE, MIDI_VELOCITY};
menuSelection menuPosition = SPUR;
unsigned long menuInvertedLastStamp = 0;

int encValue = HIGH;
int encValueOld = HIGH;
volatile bool encButtonIsActive = false;
unsigned long encButtonLastStamp = 0;

byte midiCounter = 0;
float midiLastStamp = 0;
float noteLength = 4.000;
float bpm = 120.000;
float offset = 15;
float tempo = (1000.000/(bpm/60*noteLength))-offset;
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
static bool rotating=false;      // debounce management

// interrupt service routine vars
bool A_set = false;            
bool B_set = false;



uint8_t mcpRead (byte mcpAdress, byte registerAdress);
void mcpWrite (byte mcpAdress, byte registerAdress, byte registerValues);
void markMenu(int test);
void draw(void);
void beatClock(byte realtimebyte);
void digitalWriteMCP(byte stepNummer, bool anOderAus);
void seqTrackToLED(byte trackNr);
void seqLauflicht (byte schrittNr);
void buttonInterrupt1();
void buttonInterrupt0();
void seqNoteSchreiben(byte noteInBits, int mcpNummer);
void buttonsAbfragen(byte woGedrueckt);
void togglePlaybackState();
void trackInterrupt();
void loadPreset (int whichPreset);
void sendMidiNotes(byte spur, byte schritt);
void encoderSwitch();




void setup() {

  Wire.begin();
  Serial.begin(31250); 
  
  /************  MCP23017 Setup  *************/
  pinMode(MCP_PIN_INT_A, INPUT);
  attachInterrupt(digitalPinToInterrupt(MCP_PIN_INT_A), buttonInterrupt0, FALLING);

  mcpWrite(MCP_ADDRESS_1, MCP_IODIRA,   B00000000);    // IO Direction Register, 1=Input, 0=Output, LEDs als Output
  mcpWrite(MCP_ADDRESS_1, MCP_GPIOA,    B11111111);    // LEDs anschalten
  delay(250); 
  mcpWrite(MCP_ADDRESS_1, MCP_GPIOA,    B00000000);    // LEDs ausschalten
  delay(250);
  mcpWrite(MCP_ADDRESS_1, MCP_IOCONA,   B00000000);   // set InterruptPinPol Interrupt bei LOW-Signal
  mcpWrite(MCP_ADDRESS_1, MCP_IOCONB,   B00000000);
  delay(10);
  mcpWrite(MCP_ADDRESS_1, MCP_IODIRB,   B11111111);   // IO Direction Register: 1=Input, 0=Output, Buttons als Input
  mcpWrite(MCP_ADDRESS_1, MCP_GPINTENB, B11111111);   // Interrupt-on-change Control Register: 0=Disable, 1=Enable, alle B-Ports haben für die Buttons Interrupts
  mcpWrite(MCP_ADDRESS_1, MCP_INTCONB,  B11111111);   // Interrupt Control Register: Bedingung mit welcher Interrupt ausgelöst wird, 0=InterruptOnChange, 1=InterruptOnDefValDeviation
  mcpWrite(MCP_ADDRESS_1, MCP_DEFVALB,  B11111111);   // Default Value Register: Wenn der Wert im GPIO-Register von diesem Wert abweicht, wird ein Interrupt ausgelöst. In diesem Fall lösen die Interrupts bei einem LOW Signal aus -> =0
  mcpWrite(MCP_ADDRESS_1, MCP_GPPUB,    B11111111);   // Pull-up Widerstände für Buttons aktivieren

  pinMode(MCP_PIN_INT_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(MCP_PIN_INT_B), buttonInterrupt1, FALLING);

  mcpWrite(MCP_ADDRESS_2, MCP_IODIRA,   B00000000);    // IO Direction Register, 1=Input, 0=Output, LEDs als Output
  mcpWrite(MCP_ADDRESS_2, MCP_GPIOA,    B11111111);    // LEDs anschalten
  delay(250); 
  mcpWrite(MCP_ADDRESS_2, MCP_GPIOA,    B00000000);    // LEDs ausschalten
  delay(250);
  mcpWrite(MCP_ADDRESS_2, MCP_IOCONA,   B00000000);   // set InterruptPinPol Interrupt bei LOW-Signal
  mcpWrite(MCP_ADDRESS_2, MCP_IOCONB,   B00000000);
  delay(10);
  mcpWrite(MCP_ADDRESS_2, MCP_IODIRB,   B11111111);   // IO Direction Register: 1=Input, 0=Output, Buttons als Input
  mcpWrite(MCP_ADDRESS_2, MCP_GPINTENB, B11111111);   // Interrupt-on-change Control Register: 0=Disable, 1=Enable, alle B-Ports haben für die Buttons Interrupts
  mcpWrite(MCP_ADDRESS_2, MCP_INTCONB,  B11111111);   // Interrupt Control Register: Bedingung mit welcher Interrupt ausgelöst wird, 0=InterruptOnChange, 1=InterruptOnDefValDeviation
  mcpWrite(MCP_ADDRESS_2, MCP_DEFVALB,  B11111111);   // Default Value Register: Wenn der Wert im GPIO-Register von diesem Wert abweicht, wird ein Interrupt ausgelöst. In diesem Fall lösen die Interrupts bei einem LOW Signal aus -> =0
  mcpWrite(MCP_ADDRESS_2, MCP_GPPUB,    B11111111);   // Pull-up Widerstände für Buttons aktivieren

  /************  Button 1 -STOP- Interrupt  *************/
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), togglePlaybackState, FALLING);  
  pinMode(38, OUTPUT); // LED BUTTON 1 -> StopSeq
  digitalWrite(38, LOW);

  pinMode(40, OUTPUT); // LED BUTTON 1 -> StopSeq
  digitalWrite(40, LOW);

  /************  Button 2 -CHANGETRACK- Interrupt  *************/
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), trackInterrupt, FALLING);  
  pinMode(39, OUTPUT); // LED Button 2 -> Trackchange

  /************  Internal LED Setup  *************/
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  /************  Reset PIN Setup  *************/
  pinMode(26, INPUT);

  usbMIDI.setHandleRealTimeSystem(beatClock);
  
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT);
  pinMode(ENC_PIN_A, INPUT);
  pinMode(ENC_PIN_B, INPUT); 


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



void loop() {

  // picture loop
  u8g.firstPage(); 
  do 
  {
    draw(); 
    if (encButtonIsActive == true){
      markMenu(menuActiveSelection);
    }
    else
    {
      if (millis() - menuInvertedLastStamp > 500)
      {
        markMenu(menuActiveSelection);
      }
    }

    
  }
  while (u8g.nextPage());

  


  encValue = digitalRead(6);


  if ((encValue == LOW) && (encValueOld == HIGH)) {
    lastReportedPos = encoderPos;
    

    if (digitalRead(7) == HIGH) {
      encoderPos++;
    }
    else {
      encoderPos--;
    }Serial.println(encoderPos);
  } 
  encValueOld = encValue;

  if (millis() - encButtonLastStamp > 300 && digitalRead(5) == LOW)
  {
    encButtonIsActive = !encButtonIsActive;
    digitalWrite(13, encButtonIsActive);
    encButtonLastStamp = millis();
  }


  if (displayValueChange == true) { displayValueChange = false; } 

  // Display aktualisieren
  if (encButtonIsActive == true )
  {  
    if (encoderPos != lastReportedPos)
    {
      if (encoderPos-lastReportedPos < 0)
      {
      menuActiveSelection = menuActiveSelection - 1;
      if (menuActiveSelection <= 0) { menuActiveSelection = 6; }
      }
      else 
      {
      menuActiveSelection = menuActiveSelection + 1;
      if (menuActiveSelection > 6) { menuActiveSelection = 1; }
      } 
    }
    lastReportedPos = encoderPos;
  }  
  // Display aktualisieren, Button nicht gedrückt, Parameter lassen sich ändern
  else if ((encButtonIsActive == false  && displayValueChange == true) || (encButtonIsActive == false  && millis() - menuInvertedLastStamp >= 1000))
  {
    if (invertMenu == true)
    {
      // unmark Menu BÖSE
      //markMenuInt(0);
    }
    else
    { // BÖSE
      //markMenuInt(menuAktuell);
        markMenu(menuActiveSelection);
    }
    // Display invertieren nach 500ms
    if (millis() - menuInvertedLastStamp >= 500)
    {
      invertMenu = !invertMenu;
      menuInvertedLastStamp = millis();
    }
    displayValueChange = false;
  }

  // Hier werden die Parameter der einzelnen Funktionen verändert
  if (lastReportedPos != encoderPos && encButtonIsActive == false)
  {
    switch (menuActiveSelection)
        {
          case 1:
            seqSpurAktiv = seqSpurAktiv + encoderPos - lastReportedPos;
            if (seqSpurAktiv > 200) { seqSpurAktiv = 0; }
            else if (seqSpurAktiv > 7 && seqSpurAktiv <= 200) { seqSpurAktiv = 7; }
            midiNoteDisplay = midiNotes[seqSpurAktiv][0];
            displayValueChange = true;
            break;
          case 2:
            menuActivePattern = menuActivePattern + encoderPos - lastReportedPos;
            if (menuActivePattern < 1){ menuActivePattern = 1; }
            else if (menuActivePattern > 8){ menuActivePattern = 8; }
            loadPreset(menuActivePattern+8);
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

      lastTimeTrack = millis();
      changeTrack = false;

      sendOkay = false;                // in diesem Durchlauf darf Controller keine Note mehr schicken aufgrund von ChangeTrack
      digitalWrite(39, LOW);           // LED von Button2 wird ausgeschaltet, dadurch wird Trackwechsel signalisiert
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
    mcpRead(MCP_ADDRESS_1,MCP_INTCAPB);
    mcpRead(MCP_ADDRESS_2,MCP_INTCAPB);

    usbMIDI.read();

    // Hier ist die Zeitschleife
    if (millis()-lastTime >= tempo  && startStopInterrupt == false && unknownFlag == true)
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

    if (buttonPressed != 0 && sendOkay == true)
    {
      buttonsAbfragen(buttonPressed);
      buttonPressed = 0;    
    }

    if (digitalRead(26) == 1 && digitalRead(28) == 1)
    {
      sendOkay = true;
    }
  }




void encoderSwitch(){
  if (millis() - encButtonLastStamp > 300)
    encButtonIsActive = !encButtonIsActive;
    digitalWrite(40, encButtonIsActive);
    encButtonLastStamp = millis();
}

// sendet MIDI Noten aus dem aktuellen S
void sendMidiNotes(byte spur, byte schritt)
{
  for (int i=0; i<=7; i++)
  {
    if (pattern[i][schritt] == 1) 
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

  statusICR = mcpRead(mcpWahl,MCP_INTFB); 
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
void buttonInterrupt0(){
  if ( ( millis()-lastInterrupt >= 50) && (buttonPressed == 0))
  {
    buttonPressed = 1;
    lastInterrupt = millis();
  }
}

void buttonInterrupt1(){
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

// Schreibt in die Register des MCP23017 
void digitalWriteMCP(byte stepNummer, bool anOderAus){
  byte statusGP = 0;
  byte pinNumber = 0;
  byte mcpWahl = 0;

  if ( stepNummer >= 8){ mcpWahl = MCP_ADDRESS_2; }
  else { mcpWahl = MCP_ADDRESS_1; }
  
  statusGP = mcpRead(mcpWahl, MCP_GPIOA);

  if (stepNummer >= 8) { pinNumber = stepNummer-8; }
  else { pinNumber = stepNummer;}

  if (anOderAus == 0) { statusGP &= ~(B00000001 << (pinNumber));
  }
  else if (anOderAus == 1) { statusGP |= (B00000001 << (pinNumber));
  }

  mcpWrite(mcpWahl, MCP_GPIOA, statusGP);
}


void beatClock(byte realtimebyte) {

  if(realtimebyte == MIDI_START) { midiCounter = 0; midiLastStamp = millis(); }
  if(realtimebyte == MIDI_CONTINUE) { midiLastStamp = millis(); }
  if(realtimebyte == MIDI_STOP) { digitalWrite(13, LOW); }
  
  if(realtimebyte == MIDI_CLOCK) {
    
    midiCounter++;
    if (midiCounter == 97) {midiCounter = 1;}
    if(midiCounter == 1 || midiCounter == 24 || midiCounter == 48 || midiCounter == 72) { 
      digitalWrite(13, HIGH);
    } 
    if(midiCounter == 5 || midiCounter == 25 || midiCounter == 49 || midiCounter == 73) { 
      digitalWrite(13, LOW);
    }
  }

  if (midiCounter == 24 || midiCounter == 48 || midiCounter == 72 || midiCounter == 96 ) {
    bpm = round((60000 / (millis() - midiLastStamp)));

    changeTempo = true;

    Serial.print(bpm);
    Serial.println(" BPM");
    midiLastStamp = millis();
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

  u8g.drawStr( 0, 45, "Patt"); //Presetname
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
  u8g.print(menuActivePattern);

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
      u8g.print(menuActivePattern);
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