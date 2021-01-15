#include <Arduino.h>

// MCP23017 and I2C stuff
#include <Wire.h>
#include <MCP23017.h> 

#define MCP_ADDRESS 0x20 // (A2/A1/A0 = LOW) 

int interruptPin = 26;
volatile bool event = false;
byte intCapReg; 
MCP23017 myMCP(MCP_ADDRESS,27); // 5 = ResetPin

/*
#include <Adafruit_MCP23017.h>

Adafruit_MCP23017 mcp1;           // Create MCP 1


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
//int tempo = 1000/(bpm/60)*noteLength;

/*
int position = 1;
int nextPosition = 0;
*/

unsigned long lastTime = 0;

bool fuck = true;

volatile byte buttonGedrueckt = 0;
unsigned long lastInterrupt = 0;


boolean seqSpeicher[4][8] =   { {1,0,1,0,1,1,0,1},
                                {0,0,1,0,1,1,1,0},
                                {0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0} };

byte seqSpurAktiv = 0;
byte seqStepAktuell = 0;

volatile byte statusSeqTrackToLED = 0;
volatile byte statusSeqLauflicht = 0;



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


  Serial.begin(115200); 
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
  //attachInterrupt(26, buttonInterrupt0, LOW );



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


  intCapReg = myMCP.getIntCap(B);
  if(event){
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
if (millis()-lastTime >= bpm  && fuck == true){
  seqTrackToLED(seqSpurAktiv);
  seqLauflicht(seqStepAktuell);


  seqStepAktuell = seqStepAktuell + 1;
  if (seqStepAktuell == 8){ seqStepAktuell = 0;}

  lastTime = millis();

  //usbMIDI.read();
}


  if (buttonGedrueckt != 0){

    buttonsAbfragen(buttonGedrueckt);

    buttonGedrueckt = 0;
    
  }
  


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

   Serial.println(statusICR);

   if (statusICR != 0) { seqNoteSchreiben(statusICR); }

}

void seqNoteSchreiben(byte noteInBits){
  byte x = 0;

  while ( bitRead(noteInBits, x) == 0) {
    x++;

    Serial.println("in while schleife");
    Serial.println("x:");
    Serial.println(x);
    Serial.println("noteInBits: ");
    Serial.println(noteInBits);

  }
  Serial.println("aus schleife raus");
}



void buttonInterrupt0(){
  if ( ( millis()-lastInterrupt >= 50) && (buttonGedrueckt == 0)){
    buttonGedrueckt = 1;
    lastInterrupt = millis();
  }
  //Serial.println("interrupt!");
  
}

void seqLauflicht (byte schrittNr){
  if (seqSpeicher[seqSpurAktiv][schrittNr] == 1){ digitalWriteMCP(schrittNr,0);}
  else {digitalWriteMCP(schrittNr,1);}
}

void seqTrackToLED(byte trackNr) {
  for (int i=0; i<=7; i++) {
    digitalWriteMCP(i,seqSpeicher[trackNr][i]);
   // Serial.println(seqSpeicher[trackNr][i]);
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



// Funktion nur für Arduino, funktioniert mit Teensy über USB-> Midi nicht
void sendNote(byte statusByte, byte dataByte1, byte dataByte2){
     Serial1.write(statusByte);
     Serial1.write(dataByte1); 
     Serial1.write(dataByte2);
}

void ledAnschalten(int _Position){
     switch (_Position) {
    case 1:
    /*
      usbMIDI.sendNoteOn(144, 60, 127);
      usbMIDI.sendNoteOff(144, 60, 0);
      
      mcp1.digitalWrite(0, HIGH);
      
      Wire.beginTransmission(0x20);
      Wire.write(0x12); // address port A
      Wire.write(B00000001);  // value to send
      Wire.endTransmission();
*/
      myMCP.setPort(B00000001, A);
      
      break;
    case 2:
    /*
      usbMIDI.sendNoteOn(144, 60, 127);
      usbMIDI.sendNoteOff(144, 60, 0);
      
      mcp1.digitalWrite(1, HIGH);
      
      Wire.beginTransmission(0x20);
      Wire.write(0x12); // address port A
      Wire.write(B00000010);  // value to send
      Wire.endTransmission();
      */
      myMCP.setPort(B00000010, A);

      break;
    case 3:
    /*
      usbMIDI.sendNoteOn(144, 60, 127);
      usbMIDI.sendNoteOff(144, 60, 0);
      
      mcp1.digitalWrite(2, HIGH);
      
      Wire.beginTransmission(0x20);
      Wire.write(0x12); // address port A
      Wire.write(B00000100);  // value to send
      Wire.endTransmission();
*/
      myMCP.setPort(B00000100, A);
      
      break;
    case 4:
    /*
      usbMIDI.sendNoteOn(144, 60, 127);
      usbMIDI.sendNoteOff(144, 60, 0);
      
      mcp1.digitalWrite(3, HIGH);
      
      Wire.beginTransmission(0x20);
      Wire.write(0x12); // address port A
      Wire.write(B00001000);  // value to send
      Wire.endTransmission();
*/
      myMCP.setPort(B00001000, A);
      
      break;
    case 5:
    /*
      usbMIDI.sendNoteOn(144, 60, 127);
      usbMIDI.sendNoteOff(144, 60, 0);
      
      mcp1.digitalWrite(4, HIGH);
      
      Wire.beginTransmission(0x20);
      Wire.write(0x12); // address port A
      Wire.write(B00010000);  // value to send
      Wire.endTransmission();
*/
      myMCP.setPort(B00010000, A);
      
      break;
    case 6:
    /*
      usbMIDI.sendNoteOn(144, 60, 127);
      usbMIDI.sendNoteOff(144, 60, 0);
      
      mcp1.digitalWrite(5, HIGH);
      
      Wire.beginTransmission(0x20);
      Wire.write(0x12); // address port A
      Wire.write(B00100000);  // value to send
      Wire.endTransmission();
*/
      myMCP.setPort(B00100000, A);
      
      break;
    case 7:
    /*
      usbMIDI.sendNoteOn(144, 60, 127);
      usbMIDI.sendNoteOff(144, 60, 0);
      
      mcp1.digitalWrite(6, HIGH);
      
      Wire.beginTransmission(0x20);
      Wire.write(0x12); // address port A
      Wire.write(B01000000);  // value to send
      Wire.endTransmission();
*/
      myMCP.setPort(B01000000, A);
      
      break;
    case 8:
    /*
      usbMIDI.sendNoteOn(144, 60, 127);
      usbMIDI.sendNoteOff(144, 60, 0);
      
      mcp1.digitalWrite(7, HIGH);
      
      Wire.beginTransmission(0x20);
      Wire.write(0x12); // address port A
      Wire.write(B10000000);  // value to send
      Wire.endTransmission();
*/
      myMCP.setPort(B10000000, A);
      
      break;
    default:
      // Statement(s)
      break; // Wird nicht benötigt, wenn Statement(s) vorhanden sind
}
}

void eventHappened(){
  event = true;
}
