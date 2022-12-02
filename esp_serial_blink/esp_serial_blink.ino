const int ledPin = 12; // pin the LED is attached to
int incomingByte;      // variable stores  serial data
int x;
void setup() {
  // initialize serial communication:
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  while (!Serial.available());
  x = Serial.readString().toInt();
  delay(100);
}
