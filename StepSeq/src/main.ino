#include <Arduino.h>
#include <Wire.h>

/************  MCP23017   ***************/
#include <MCP23017.h> 

#define MCP_ADDRESS 0x20 // (A2/A1/A0 = LOW) 

int interruptPin = 26;
volatile bool event = false;
byte intCapReg; 
MCP23017 myMCP(MCP_ADDRESS,27); // 27 = ResetPin


/************  Display   ***************/
#include <Adafruit_SSD1306.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_GFX.h>

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

enum menuAnwahl {TEMPO, SPUR, MIDI_NOTE, MIDI_VELOCITY};


/************  Encoder   **************/
int messungPin1 = LOW;
int messungPin1Alt = LOW;

volatile int encoderWert = 0;
int encoderWertAlt = 0;



/************  MIDI IN   **************/
const byte START = 250;
const byte CONTINUE = 251;
const byte STOP = 252;
const byte CLOCK = 248;


byte zaehler = 0;
float zeitAlt = 0;



/************  Timer and Tempo   ************/
float bpm = 120;
float tempo = 1/(bpm/60);
//int tempo = 1000/(bpm/60)*noteLength;

unsigned long lastTime = 0;
unsigned long lastTimeTrack = 0;
unsigned long lastTimeStartStop = 0;




volatile byte buttonGedrueckt = 0;
unsigned long lastInterrupt = 0;


/************  MIDI OUT   **************/
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

  


  Wire.begin();
  Serial.begin(31250); 
  
  /************  MCP23017 Setup  *************/
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), buttonInterrupt0, FALLING);

  myMCP.Init();
  myMCP.setPortMode(B11111111, A);
  myMCP.setPort(B11111111, A); // kurzer LED Test
  delay(500); 
  myMCP.setAllPins(A, OFF);
  delay(500);
  myMCP.setInterruptPinPol(LOW);
  delay(10);
  myMCP.setInterruptOnDefValDevPort(B11111111, B, B11111111); // IntPins, Port, DEFVAL
  myMCP.setPortPullUp(B11111111, B);
  event=false;


  /************  Button 1 -STOP- Interrupt  *************/
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), stopInterrupt, FALLING);  


  /************  Button 2 -CHANGETRACK- Interrupt  *************/
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), trackInterrupt, FALLING);  
  pinMode(39, OUTPUT); // LED Button 2 -> Trackchange


  /************  SSD1306 Setup  *************/
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // I2C address = 0x3C
  delay(1000);
    
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


  /************  Encoder Setup  *************/
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  //attachInterrupt(6, encoderInterrupt, FALLING);


  /************  Internal LED Setup  *************/
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  /************  Reset PIN Setup  *************/
  pinMode(26, INPUT);

  
  

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

  if (lastButtonPressed != 0){
    seqSpurAktiv = lastButtonPressed-1;
  }
    display.fillRect(63,40,127,40, BLACK);
    display.setCursor(63, 40); 
    display.setTextColor(WHITE);  
    display.print(spurNamen[seqSpurAktiv]); 
    display.display(); 

  lastTimeTrack = millis();
  changeTrack = false;

  sendOkay = false;                // in diesem Durchlauf darf Controller keine Note mehr schicken aufgrund von ChangeTrack
  digitalWrite(39, LOW);           // LED von Button2 wird ausgeschaltet, dadurch wird Trackwechsel signalisiert
}

  intCapReg = myMCP.getIntCap(B);  // Register muss hier ausgelesen werden, dadurch wird der Interrupt abgelöscht
  /*
  if(event){

    Serial.println("lalalalalalala");
    delay(200);
    
    byte intFlagReg, eventPin; 
    byte test, test2;

    intFlagReg = myMCP.getIntFlag(B);
    eventPin = log(intFlagReg)/log(2);
    
    
    test = log(intFlagReg);
    test2 = log(2);
    Serial.println("hallo!");
    Serial.println(test);
    Serial.println(test2);
    
    
    
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
  */
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

// sendet MIDI Noten aus dem aktuellen S
void sendMidiNotes(byte spur, byte schritt){
  
  for (int i=0; i<=7; i++){
    if (seqSpeicher[i][schritt] == 1) 
    {
    //usbMIDI.sendNoteOn(midiNotes[i][0], 127, 1);
    //usbMIDI.sendNoteOff(midiNotes[i][0], 127, 1);
    }
  }
}


/*  ZUERST KONDENSATOR EINBAUEN!!!
void encoderInterrupt(){
  if (digitalRead(7) == HIGH){
    encoderWert++;
  }
  else {
    encoderWert--;
  }
}
*/

// Interrupt, welcher die aktuelle Spur ändert
void trackInterrupt(){
  if (( millis()-lastTimeTrack >= 50) && changeTrack == false){
    changeTrack = true;
    lastTimeTrack = millis();
  }
}

// Interrupt, welcher den Sequencer startet+stoppt
void stopInterrupt(){
  if (( millis()-lastTimeStartStop >= 100) && startStopInterrupt == false){
    startStopInterrupt = true;
    lastTimeStartStop = millis();
  }
}

// Buttons werden abgefragt
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

// Schreibt Werte in den SeqSpeicher
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
      Serial.println("Note 0");
      lastButtonPressed = 0;
    }
    else 
    {
      seqSpeicher[seqSpurAktiv][x] = 1; 
      sendOkay = false;
      Serial.println("Note 1");
      lastButtonPressed = 0;
    }
  }
  else
  {
    
  }




  //Serial.println("aus schleife raus");
}

// Button von MPC23017-1 wird gedrückt
void buttonInterrupt0(){
  if ( ( millis()-lastInterrupt >= 50) && (buttonGedrueckt == 0))
  {
    buttonGedrueckt = 1;
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
  for (int i=0; i<=7; i++) {
    digitalWriteMCP(i,seqSpeicher[trackNr][i]);
    }
  }

// Schreibt in die Register des MCP23017 
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
}


