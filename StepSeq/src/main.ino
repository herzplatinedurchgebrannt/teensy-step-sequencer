#include <Arduino.h>
#include <Wire.h>

//  ***********  MCP23017 STUFF **************
#include <MCP23017.h> 

#define MCP_ADDRESS 0x20 // (A2/A1/A0 = LOW) 

int interruptPin = 26;
volatile bool event = false;
byte intCapReg; 
MCP23017 myMCP(MCP_ADDRESS,27); // 27 = ResetPin


//  ***********  Display STUFF **************
#include <Adafruit_SSD1306.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_GFX.h>

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);





int messungPin1 = LOW;
int messungPin1Alt = LOW;

int encoderWert = 0;
int encoderWertAlt = 0;




/*



const byte START = 250;
const byte CONTINUE = 251;
const byte STOP = 252;
const byte CLOCK = 248;


byte zaehler = 0;
float zeitAlt = 0;

bool buttonPressed1 = false;
bool buttonPressed2 = false;
bool buttonPressed3 = false;
bool buttonPressed4 = false;
*/











float bpm = 120;
float tempo = 1/(bpm/60);
//int tempo = 1000/(bpm/60)*noteLength;

/*
int position = 1;
int nextPosition = 0;
*/

unsigned long lastTime = 0;
unsigned long lastTimeTrack = 0;
unsigned long lastTimeStartStop = 0;




volatile byte buttonGedrueckt = 0;
unsigned long lastInterrupt = 0;


// ************  MIDI STUFF  **************

boolean seqSpeicher[8][8] =   { {1,0,0,0,0,0,0,0},
                                {1,1,0,0,0,0,0,0},
                                {0,1,1,0,0,0,0,0},
                                {0,0,1,1,0,0,0,0},
                                {0,0,0,1,1,0,0,0},
                                {0,0,0,0,1,1,0,0},
                                {0,0,0,0,0,1,1,0},
                                {0,0,0,0,0,0,1,1} 
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





String spurNamen [8] = { "KICK", 
                         "SNARE",
                         "HIHAT",
                         "CRASH",
                         "SHAKER",
                         "TOM1",
                         "TOM2",
                         "TAMBURIN"};



byte seqSpurAktiv = 0;
byte seqStepAktuell = 0;

volatile byte statusSeqTrackToLED = 0;
volatile byte statusSeqLauflicht = 0;

bool sendOkay = true;

volatile bool changeTrack = false;
volatile bool startStopInterrupt = false;

bool start = true;

int lastButtonPressed = 0;


void setup() {

  /*
  mcp1.begin();               // Start MCP 1 on Hardware address 0x20

  mcp1.pinMode(0, OUTPUT);
  mcp1.pinMode(1, OUTPUT);
  mcp1.pinMode(2, OUTPUT); 
  mcp1.pinMode(3, OUTPUT); 
  mcp1.pinMode(4, OUTPUT); 
  mcp1.pinMode(5, OUTPUT); 
  mcp1.pinMode(6, OUTPUT); 
  mcp1.pinMode(7, OUTPUT);  
  mcp1.pinMode(8, OUTPUT); 
  */
  
  Wire.begin();
  
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), buttonInterrupt0, FALLING);

  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), stopInterrupt, FALLING);  

  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), trackInterrupt, FALLING);  

  Serial.begin(31250); 


  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // I2C address = 0x3C
  delay(1000);

  /*
  display.clearDisplay();
  display.setTextSize(3);     
  display.setTextColor(WHITE);  
  display.setCursor(0, 0);  
  display.print("STEP SEQUENCER");
  display.display(); 
  delay(500);
  display.clearDisplay();
*/
  display.clearDisplay();

/*
  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("STEP SEQUENCER"));
  display.display();      // Show initial text
  delay(100);

  // Scroll in various directions, pausing in-between:
  display.startscrollright(0x00, 0x0F);
  delay(200);
  display.stopscroll();
  delay(100);
  display.startscrollleft(0x00, 0x0F);
  delay(200);
  display.stopscroll();
  delay(100);
  display.startscrolldiagright(0x00, 0x07);
  delay(200);
  display.startscrolldiagleft(0x00, 0x07);
  delay(200);
  display.stopscroll();
  delay(100);
  */



    
  display.clearDisplay();

  display.fillRect(0,0,128,18,WHITE);
  display.setTextSize(2);     
  display.setTextColor(BLACK);  
  display.setCursor(10, 1);  
  display.print("SEQUENCER");

  display.drawLine(0,17,128,17,WHITE);

  display.setCursor(1, 25); 
  display.setTextSize(1); 
  display.setTextColor(WHITE);  
  display.print("TEMPO:"); 

  display.setCursor(63, 25); 
  display.setTextColor(WHITE);  
  display.print(bpm + (encoderWert - encoderWertAlt)); 

  display.setCursor(1, 40); 
  display.setTextColor(WHITE);  
  display.print("SPUR:"); 

  display.setCursor(63, 40); 
  display.setTextColor(WHITE);  
  display.print(spurNamen[seqSpurAktiv]); 

  display.display(); 

  delay(500);
 // display.clearDisplay();



  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);





//115200
//31250

  Wire.begin();
  myMCP.Init();
  myMCP.setPortMode(B11111111, A);
  myMCP.setPort(B11111111, A); // kurzer LED Test
  delay(1000); 
  myMCP.setAllPins(A, OFF);
  delay(1000);
  myMCP.setInterruptPinPol(LOW);
  delay(10);
  myMCP.setInterruptOnDefValDevPort(B11111111, B, B11111111); // IntPins, Port, DEFVAL
  myMCP.setPortPullUp(B11111111, B);
  event=false;

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  pinMode(26, INPUT);

  pinMode(39, OUTPUT);






  /*
  // Im Register A befinden sich die LEDs, Register A muss auf OUTPUT gestellt werden
  // MCP23017 befindet sich auf Adresse 0x20 -> Dezimal 32, Binär B00100000
  
  Wire.beginTransmission(0x20);
  Wire.write(0x00); // IODIRA register
  Wire.write(0x00); // set all of bank A to outputs
  Wire.endTransmission();



  // http://robert-fromm.info/?post=elec_i2c_mcp23017
  // interne Pullup Widerstände aktivieren. Adresse ist laut Datenblatt S.22 0x06, laut Beispiel 0x0D ?!?!?
  Wire.beginTransmission(0x20);
  Wire.write(0x0D);
  Wire.write(B11111111);
  Wire.endTransmission();

// Interrupt Stuff 

  Wire.beginTransmission(0x20);
  Wire.write(0x05);
  Wire.write(B00000000);
  Wire.endTransmission();

  Wire.beginTransmission(0x20);
  Wire.write(0x07);
  Wire.write(B11111111);
  Wire.endTransmission();

  
  Wire.beginTransmission(0x20);
  Wire.write(0x09);
  Wire.write(B11111111);
  Wire.endTransmission();
  


/*
  // Im Register B befinden sich die Schalter, Standardnmäßig sind die Register auf INPUT gestellt
  Wire.beginTransmission(B00100000);
  Wire.write(B00000111); // IODIRB register
  Wire.endTransmission();
  */
  
  /*usbMIDI.setHandleRealTimeSystem(beatClock);
  */
} 

void loop() {

messungPin1 = digitalRead(6);

if ((messungPin1 == HIGH) && (messungPin1Alt == LOW)){
  if (digitalRead(7) == HIGH){
    encoderWert++;
    
    display.fillRect(63,20,127,20, BLACK);
    display.setCursor(63, 25); 
    display.setTextColor(WHITE);  
    display.print(bpm + (encoderWert - encoderWertAlt)); 
    bpm = bpm+(encoderWert - encoderWertAlt);

    display.display(); 
    encoderWertAlt = encoderWert;
  }
  else {
    encoderWert--;
    
    display.fillRect(63,20,127,20, BLACK);
    display.setCursor(63, 25); 
    display.setTextColor(WHITE);  
    display.print(bpm + (encoderWert - encoderWertAlt)); 
    bpm = bpm+(encoderWert - encoderWertAlt);
    display.display(); 

    encoderWertAlt = encoderWert;

  
  }
  Serial.println (encoderWert);
}
messungPin1Alt = messungPin1;




if (startStopInterrupt == true && start == true){
  start = false;
  startStopInterrupt = false;
  seqStepAktuell = 0;
  
}
else if (startStopInterrupt == true && start == false){
  start = true;
  startStopInterrupt = false;
}


if (changeTrack == true){
  digitalWrite(39, HIGH);
}


if (changeTrack == true && lastButtonPressed != 0){

  debugMessage(1,seqSpurAktiv,lastButtonPressed);
  
  /*
  seqSpurAktiv = seqSpurAktiv +1;
  
  if (seqSpurAktiv >= 4){
    seqSpurAktiv = 0;
  }
  */
  if (lastButtonPressed != 0){
    seqSpurAktiv = lastButtonPressed-1;
  }

  debugMessage(2,seqSpurAktiv,lastButtonPressed);

    display.fillRect(63,40,127,40, BLACK);
    display.setCursor(63, 40); 
    display.setTextColor(WHITE);  
    display.print(spurNamen[seqSpurAktiv]); 
    display.display(); 





  lastTimeTrack = millis();
  changeTrack = false;

  // in diesem Durchlauf darf Controller keine Note mehr sagen aufgrund von ChangeTrack
  sendOkay = false;

  digitalWrite(39, LOW);
}

  intCapReg = myMCP.getIntCap(B);
  if(event){
    delay(200);
    
    byte intFlagReg, eventPin; 
    byte test, test2;

    intFlagReg = myMCP.getIntFlag(B);
    eventPin = log(intFlagReg)/log(2);
    
    /*
    test = log(intFlagReg);
    test2 = log(2);
    Serial.println("hallo!");
    Serial.println(test);
    Serial.println(test2);
    */
    
    
    intCapReg = myMCP.getIntCap(B);
    Serial.println("Interrupt!");
    Serial.print("Interrupt Flag Register: ");
    Serial.println(intFlagReg,BIN); 
    Serial.print("Interrupt Capture Register: ");
    Serial.println(intCapReg,BIN); 
    Serial.print("Pin No.");
    Serial.print(eventPin);
    Serial.print(" went ");
    if((intFlagReg&intCapReg) == 0){
      Serial.println("LOW");
    }
    else{
      Serial.println("HIGH");
    }
    myMCP.setPort(intFlagReg, A);
    delay(1000);
    
    event = false;
  }
  
  /*
   // read the inputs of bank B
  Wire.beginTransmission(0x20);
  Wire.write(0x13);
  Wire.endTransmission();
  Wire.requestFrom(0x20, 1);
  inputs=Wire.read();

  //Serial.println(inputs);

/*
  if (inputs < 255){
    digitalWrite(13, LOW);
  }
  */
/*
  // now send the input data to bank A
  Wire.beginTransmission(0x20);
  Wire.write(0x12); // GPIOA
  Wire.write(inputs);    // bank A
  Wire.endTransmission();
  delay(200); // for debounce
  */
  /*
  Wire.requestFrom(0x20, 1); // request one byte of data from MCP20317
    inputs = Wire.read(); // store the incoming byte into "inputs"
    if (inputs > 0) {
        // if a button was pressed
        Serial.println(inputs, BIN); // display the contents of the GPIOB register in binary
        delay(200); // for debounce
    }
    */
/*  
mcp1.begin(0x20); // Init MCP23017 at address 0x20
    for (byte i=0; i<8; i++) {
        mcp1.pinMode(i, OUTPUT);
    }

*/
/*


Wire.beginTransmission(0x20);
Wire.write(0x12); // address port A
Wire.write(B11111111);  // value to send
Wire.endTransmission();

Wire.beginTransmission(0x20);
Wire.write(0x13); // address port B
Wire.write(B11111111 );  // value to send
Wire.endTransmission();


delay(1000);

Wire.beginTransmission(0x20);
Wire.write(0x12); // address port A
Wire.write(B00000000);  // value to send
Wire.endTransmission();

Wire.beginTransmission(0x20);
Wire.write(0x13); // address port B
Wire.write(B00000000 );  // value to send
Wire.endTransmission();


delay(1000);

*/

  // Hier ist die Zeitschleife
  if (millis()-lastTime >= bpm  && start == true)
  {
    
    // LEDs von der aktuell angewählten Spur werden angezeigt
    seqTrackToLED(seqSpurAktiv);

    // Lauflichteffekt
    seqLauflicht(seqStepAktuell);

    // Midi Noten raus schicken per USB
    sendMidiNotes(seqSpurAktiv, seqStepAktuell);

    // Schritt hochzählen
    seqStepAktuell = seqStepAktuell + 1;
    if (seqStepAktuell == 8){ seqStepAktuell = 0;}

    // lastTime vllt mal an Anfang der Schleife ausprobieren für stabileres Timing?!?!?!
    lastTime = millis();

    //usbMIDI.read();
  }

  if (buttonGedrueckt != 0 && sendOkay == true)
  {
    buttonsAbfragen(buttonGedrueckt);
    buttonGedrueckt = 0;    
  }

  if (digitalRead(26) == 1)
  {
    sendOkay = true;
  }
}

void sendMidiNotes(byte spur, byte schritt){
  
  for (int i=0; i<=7; i++){
    if (seqSpeicher[i][schritt] == 1) 
    {
    //usbMIDI.sendNoteOn(midiNotes[i][0], 127, 1);
    //usbMIDI.sendNoteOff(midiNotes[i][0], 127, 1);
    }
  }
}

void trackInterrupt(){
  if (( millis()-lastTimeTrack >= 50) && changeTrack == false){
    changeTrack = true;
    lastTimeTrack = millis();
  }
}

void stopInterrupt(){
  if (( millis()-lastTimeStartStop >= 100) && startStopInterrupt == false){
    startStopInterrupt = true;
    lastTimeStartStop = millis();
  }
}

void debugMessage (int stelle, int variable1, int variable2){
  Serial.println("...");
  Serial.println("Debugging startet hier. Stelle:");
  Serial.println(stelle);
  Serial.println("Integer Variable 1:");
  Serial.println(variable1);
  Serial.println("Integer Variable 2:");
  Serial.println(variable1);
  Serial.println("Debugging ENDE");
  Serial.println("...");
  Serial.println("...");
}

void buttonsAbfragen(byte woGedrueckt) {
   byte statusICR = 0;
   byte mcpWahl = 0;

   if (woGedrueckt == 1 ) {  mcpWahl = 0; }
   if (woGedrueckt == 2 ) {  mcpWahl = 1; }

   Wire.beginTransmission(0x20);
   Wire.write(0x0F);
   Wire.endTransmission();
   Wire.requestFrom(0x20,1);
   statusICR = Wire.read();
   Wire.endTransmission();

  // Serial.println(statusICR);
   lastButtonPressed = statusICR;

   if (statusICR != 0) { seqNoteSchreiben(statusICR); }
}

void seqNoteSchreiben(byte noteInBits){
  byte x = 0;

  while ( bitRead(noteInBits, x) == 0) 
  {
    x++;
  }

  lastButtonPressed = x+1;
  // HIER SCHREIBEN
  if(changeTrack == false)
  
  {
    

    if (seqSpeicher[seqSpurAktiv][x] ==  1) 
    { 
      seqSpeicher[seqSpurAktiv][x]=0; 
      sendOkay = false;
      Serial.println("HAHJAHAHA");
      lastButtonPressed = 0;
    }
    else 
    {
      seqSpeicher[seqSpurAktiv][x] = 1; 
      sendOkay = false;
      Serial.println("98676566");
      lastButtonPressed = 0;
    }
  }
  else
  {
    
  }




  //Serial.println("aus schleife raus");
}

void buttonInterrupt0(){
  if ( ( millis()-lastInterrupt >= 50) && (buttonGedrueckt == 0))
  {
    buttonGedrueckt = 1;
    lastInterrupt = millis();
  }
}

void seqLauflicht (byte schrittNr){
  if (seqSpeicher[seqSpurAktiv][schrittNr] == 1){ digitalWriteMCP(schrittNr,0);}
  else {digitalWriteMCP(schrittNr,1);}
}

void seqTrackToLED(byte trackNr) {
  for (int i=0; i<=7; i++) {
    digitalWriteMCP(i,seqSpeicher[trackNr][i]);
    }
  }

void digitalWriteMCP(byte stepNummer, boolean anOderAus){
  byte statusGP = 0;
  byte pinNumber = 0;
  byte mcpWahl = 0;

  /*

  if ( stepNummer >= 16){ mcpWahl = 1; }
  else {mcpWahl = 0;}
  */


  Wire.beginTransmission(0x20);
  Wire.write(0x12);
  Wire.endTransmission();
  Wire.requestFrom(0x20, 1);
  statusGP = Wire.read();
  Wire.endTransmission();

  if (stepNummer >= 8) {pinNumber = stepNummer-8;}
  else { pinNumber = stepNummer;}


  if (anOderAus == 0) { statusGP &= ~(B00000001 << (pinNumber));
  }
  else if (anOderAus == 1) { statusGP |= (B00000001 << (pinNumber));
  }

  Wire.beginTransmission(0x20);
  Wire.write(0x12);
  Wire.write(statusGP);
  Wire.endTransmission();


}








/*
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
    Serial.print(bpm);
    Serial.println(" BPM");
    zeitAlt = millis();
  }
}*/



////////////////////////////////////// SCHROTT /////////////////////////////////////////////////////////////////////////////


void eventHappened(){
  event = true;
}
