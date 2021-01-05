byte statusByte;
byte dataByte1;
byte dataByte2;

void setup(){
 pinMode(13, OUTPUT); // Eingebaute LED
 digitalWrite(13, LOW);
 Serial1.begin(31250);

 digitalWrite(8,HIGH);
}


void loop(){
 empfangeMIDI();
 delay(10);
 // Ohne Delay leuchtet die LED nicht, dadurch verpasst man aber einige Signale
 digitalWrite(13, LOW);
}


void empfangeMIDI(){
 do {
 if (Serial1.available()){
 
 statusByte = Serial1.read();
 dataByte1 = Serial1.read();
 dataByte2 = Serial1.read();

 if (statusByte >= 144 && statusByte <= 159) { // Entspricht NoteOn MIDI Kanal 1-16
 digitalWrite(13,HIGH);
 }
 }
 }
 while (Serial1.available() > 2); //Ausführen, wenn mindestens 3 Bytes da sind
}
