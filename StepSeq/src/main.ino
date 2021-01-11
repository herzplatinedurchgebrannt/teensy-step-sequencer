#include <Arduino.h>


#include <Wire.h>

/*
#include <Adafruit_MCP23017.h>

Adafruit_MCP23017 mcp1;           // Create MCP 1
*/

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

byte inputs = 0;
byte blub = 0;



// als tempo sollte die Notenlänge von 1/4 Note herauskommen
int noteLength = 1;

static unsigned long tempo = 1000;
//int tempo = 1000/(bpm/60)*noteLength;

int position = 1;
int nextPosition = 0;

unsigned long lastTime = 0;

unsigned long test = 0;

bool fuck = true;




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


/*
  // Im Register B befinden sich die Schalter, Standardnmäßig sind die Register auf INPUT gestellt
  Wire.beginTransmission(B00100000);
  Wire.write(B00000111); // IODIRB register
  Wire.endTransmission();
  */
  

 Serial.begin(115200); 

  
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

  digitalWrite(13, HIGH);




  
  
  /*usbMIDI.setHandleRealTimeSystem(beatClock);
  */
}

void loop() {
  
  
   // read the inputs of bank B
  Wire.beginTransmission(0x20);
  Wire.write(0x13);
  Wire.endTransmission();
  Wire.requestFrom(0x20, 1);
  inputs=Wire.read();

  Serial.println(inputs);

  if (inputs < 255){
    digitalWrite(13, LOW);
  }

  blub = !inputs;

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


/*

mcp1.digitalWrite(0, HIGH);
mcp1.digitalWrite(1, HIGH);
mcp1.digitalWrite(2, HIGH);
mcp1.digitalWrite(3, HIGH);
mcp1.digitalWrite(4, HIGH);
mcp1.digitalWrite(5, HIGH);
mcp1.digitalWrite(6, HIGH);
mcp1.digitalWrite(7, HIGH);

*/

// Hier ist die Zeitschleife
if (millis()-lastTime >= tempo  && fuck == true){
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
      */
      Wire.beginTransmission(0x20);
      Wire.write(0x12); // address port A
      Wire.write(B00000001);  // value to send
      Wire.endTransmission();
      
      break;
    case 2:
    /*
      usbMIDI.sendNoteOn(144, 60, 127);
      usbMIDI.sendNoteOff(144, 60, 0);
      
      mcp1.digitalWrite(1, HIGH);
      */
      Wire.beginTransmission(0x20);
      Wire.write(0x12); // address port A
      Wire.write(B00000010);  // value to send
      Wire.endTransmission();
      
      break;
    case 3:
    /*
      usbMIDI.sendNoteOn(144, 60, 127);
      usbMIDI.sendNoteOff(144, 60, 0);
      
      mcp1.digitalWrite(2, HIGH);
      */
      Wire.beginTransmission(0x20);
      Wire.write(0x12); // address port A
      Wire.write(B00000100);  // value to send
      Wire.endTransmission();
      
      break;
    case 4:
    /*
      usbMIDI.sendNoteOn(144, 60, 127);
      usbMIDI.sendNoteOff(144, 60, 0);
      
      mcp1.digitalWrite(3, HIGH);
      */
      Wire.beginTransmission(0x20);
      Wire.write(0x12); // address port A
      Wire.write(B00001000);  // value to send
      Wire.endTransmission();
      
      break;
    case 5:
    /*
      usbMIDI.sendNoteOn(144, 60, 127);
      usbMIDI.sendNoteOff(144, 60, 0);
      
      mcp1.digitalWrite(4, HIGH);
      */
      Wire.beginTransmission(0x20);
      Wire.write(0x12); // address port A
      Wire.write(B00010000);  // value to send
      Wire.endTransmission();
      
      break;
    case 6:
    /*
      usbMIDI.sendNoteOn(144, 60, 127);
      usbMIDI.sendNoteOff(144, 60, 0);
      
      mcp1.digitalWrite(5, HIGH);
      */
      Wire.beginTransmission(0x20);
      Wire.write(0x12); // address port A
      Wire.write(B00100000);  // value to send
      Wire.endTransmission();
      
      break;
    case 7:
    /*
      usbMIDI.sendNoteOn(144, 60, 127);
      usbMIDI.sendNoteOff(144, 60, 0);
      
      mcp1.digitalWrite(6, HIGH);
      */
      Wire.beginTransmission(0x20);
      Wire.write(0x12); // address port A
      Wire.write(B01000000);  // value to send
      Wire.endTransmission();
      
      break;
    case 8:
    /*
      usbMIDI.sendNoteOn(144, 60, 127);
      usbMIDI.sendNoteOff(144, 60, 0);
      
      mcp1.digitalWrite(7, HIGH);
      */
      Wire.beginTransmission(0x20);
      Wire.write(0x12); // address port A
      Wire.write(B10000000);  // value to send
      Wire.endTransmission();
      
      break;
    default:
      // Statement(s)
      break; // Wird nicht benötigt, wenn Statement(s) vorhanden sind
}
}

int ledNext(int _NextPosition){
     switch (_NextPosition) {
    case 1:
      //Serial.print(_NextPosition);
      return 2;
    case 2:
      //Serial.print(_NextPosition);
      return 3;
    case 3:
      //Serial.print(_NextPosition);
      return 4;
    case 4:
      //Serial.print(_NextPosition);
      return 5;
    case 5:
      //Serial.print(_NextPosition);
      return 6;
    case 6:
      //Serial.print(_NextPosition);
      return 7;
    case 7:
      //Serial.print(_NextPosition);
      return 8;
    case 8:
      //Serial.print(_NextPosition);
      return 1;
    default:
      // Serial.print(_NextPosition);
      break; // Wird nicht benötigt, wenn Statement(s) vorhanden sind
}
}

void ledAusschalten(){

    Wire.beginTransmission(0x20);
    Wire.write(0x12); // address port A
    Wire.write(B00000000);  // value to send
    Wire.endTransmission();
  
}