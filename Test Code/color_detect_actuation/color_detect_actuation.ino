int x = 3;
const int ledPin = 27;

void setup() {
 Serial.begin(115200);
 pinMode(ledPin, OUTPUT);
 Serial.setTimeout(1);
}

void loop() {
 while (!Serial.available());
  //Serial.print(Serial.readString().toInt());
  if (Serial.readString().toInt() == 1) {
    digitalWrite(ledPin, HIGH);
  }
  else {
    digitalWrite(ledPin, LOW);
  }
}

















