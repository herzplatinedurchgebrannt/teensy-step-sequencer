#include <Arduino.h>

////////////////////////
/*
#define MCP_ADDRESS 0x20 // (A2/A1/A0 = LOW) 
#include <Wire.h>
#include <MCP23017.h>
MCP23017 myMCP(MCP_ADDRESS,3); // 3= ResetPin, nicht weiter wichtig hier 
*/
///////////////////////

const byte START = 250;
const byte CONTINUE = 251;
const byte STOP = 252;
const byte CLOCK = 248;

byte zaehler = 0;
float bpm = 0;
float zeitAlt = 0;


bool buttonPressed1 = false;
bool buttonPressed2 = false;
bool buttonPressed3 = false;
bool buttonPressed4 = false;



// als tempo sollte die Notenlänge von 1/4 Note herauskommen
int noteLength = 1;

static unsigned long tempo = 1000;
//int tempo = 1000/(bpm/60)*noteLength;

int position = 1;
int nextPosition = 0;

unsigned long lastTime = 0;

unsigned long test = 0;


void setup() {
  
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  pinMode(13, OUTPUT);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);

  ////////////////
  /*
  Wire.begin(18,19);
  Wire.begin(19); //SDA = GPIO0 / SCL = GPIO2
  myMCP.Init();
  myMCP.setPortMode(B11111111,A);
  myMCP.setPortMode(B11111111,B);
  myMCP.setPort(B00000000, A); // alles auf LOW 
  myMCP.setPort(B00000000, B); delay(1000); 
  */
  //////////////////////



  Serial.begin(115200); 
  
  /*usbMIDI.setHandleRealTimeSystem(beatClock);
  */
}

void loop() {

  /////////////////////////////
  /*
  myMCP.setPin(1,A,ON); 
  delay(1000); 
  myMCP.setPin(1,B,ON); 
  delay(1000); 
  myMCP.setPin(1,A,OFF); 
  delay(1000); 
  myMCP.setPin(1,B,OFF); 
  delay(1000); 
  myMCP.setPort(B10101010,B00011000); 
  delay(1000); 
  for(int i=0; i<=7; i++){ 
    myMCP.togglePin(i,A); 
  } 
  delay(1000); 
  myMCP.setPort(B00000000, A); 
  myMCP.setPort(B00000000, B); 
  delay(1000); 
  */
////////////////////////////////



if (millis()-lastTime >= tempo){
  digitalWrite(13, HIGH);
  
  ledAusschalten();
  
  ledAnschalten(position);
  nextPosition = ledNext(position);
 

  lastTime = millis();
 
  
  position = nextPosition;
 


}


  //usbMIDI.read();
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
      */
      digitalWrite(9, HIGH);
      break;
    case 2:
    /*
      usbMIDI.sendNoteOn(144, 60, 127);
      usbMIDI.sendNoteOff(144, 60, 0);
      */
      digitalWrite(10, HIGH);
      break;
    case 3:
    /*
      usbMIDI.sendNoteOn(144, 60, 127);
      usbMIDI.sendNoteOff(144, 60, 0);
      */
      digitalWrite(11, HIGH);
      break;
    case 4:
    /*
      usbMIDI.sendNoteOn(144, 60, 127);
      usbMIDI.sendNoteOff(144, 60, 0);
      */
      digitalWrite(12, HIGH);
      break;
    default:
      // Statement(s)
      break; // Wird nicht benötigt, wenn Statement(s) vorhanden sind
}
}

int ledNext(int _NextPosition){
     switch (_NextPosition) {
    case 1:
      Serial.print(_NextPosition);
      return 2;
    case 2:
      Serial.print(_NextPosition);
      return 3;
    case 3:
      Serial.print(_NextPosition);
      return 4;
    case 4:
      Serial.print(_NextPosition);
      return 1;
    default:
      Serial.print(_NextPosition);
      break; // Wird nicht benötigt, wenn Statement(s) vorhanden sind
}
}

void ledAusschalten(){
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
}