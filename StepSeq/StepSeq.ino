void setup() {
 pinMode(4, INPUT);
 pinMode(13, OUTPUT); // Eingebaute LED
}

void loop() {
 int variable = digitalRead(4);
 if (variable == HIGH) {
 digitalWrite(13, LOW);
 }
 else {
 digitalWrite(13, HIGH);
 delay(100);
 }
}