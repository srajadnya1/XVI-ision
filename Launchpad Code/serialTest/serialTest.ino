byte msgArray[8];
#define LED RED_LED



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
//  digitalWrite(LED, LOW);
  Serial.println("Arduino is ready");
}

void loop() {
  // put your main code here, to run repeatedly: 
  if (Serial.available()>2) {
    char receivedMsg = Serial.read();
//    long int coord = Serial.parseInt();
//    Serial.println(coord);
      Serial.println(receivedMsg);
    if (receivedMsg == 'c') {
      digitalWrite(LED, HIGH);
    }
    else {
      digitalWrite(LED, LOW);
    }
    delay(10);
  }

//  delay(1000);
//while (Serial.available() > 0) {
//    receivedMsg = Serial.read();
//    Serial.println(receivedMsg + 8);
//  }
//  if (receivedMsg == 8) {
//    digitalWrite(LED, HIGH);
//    delay(100);
//    digitalWrite(LED, LOW);
//    delay(100);
//    digitalWrite(LED, HIGH);
//    delay(100);
//    digitalWrite(LED, LOW);
//    delay(100);
//  }
//  delay(1000);
//  readFromSerial();
//  writeToSerial();
//
}


//void readFromSerial() {
//  while (Serial.available() > 0) {
//    receivedMsg = Serial.read();
//  }
//}
//
//void writeToSerial() {
//  Serial.println("Received new data");
//  Serial.print(receivedMsg);
//}
