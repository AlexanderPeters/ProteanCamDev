#include "SerialTransfer.h"


SerialTransfer myTransfer;
HardwareSerial Comm(1);

String alive = "ALIVE";
const String takeImage = "TAKEIMAGE";

void setup() {
  Comm.begin(115200, SERIAL_8N1, 15, 14);   //, Comm_Txd_pin, Comm_Rxd_pin); // Define and start Comm serial port
  myTransfer.begin(Comm);

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
}

String receiveMessage() {
  int messageLength = sizeof(myTransfer.packet.rxBuff);
  char received[messageLength];
  
  for (int i = 0; i <= messageLength; i++)
    received[i] = myTransfer.packet.rxBuff[i];

  return String(received);
}

void sendMessage(String message) {
  for (int i = 0; i < sizeof(message); i++)
    myTransfer.packet.txBuff[i] = message[i];

  myTransfer.sendData(sizeof(message));
}

void loop() {
  if (myTransfer.available()) {
    String received = receiveMessage();

    if (received == (alive + "?")) {
      sendMessage(alive);
      digitalWrite(2, HIGH);
      delay(200);
      digitalWrite(2, LOW);
      delay(200);
    }
    else if (received == takeImage) {
      sendMessage("Ok I will take a picture.865476547654");
    }
  }
  else if (myTransfer.status < 0) {
    Serial.print("ERROR: ");
    Serial.println(myTransfer.status);
  }
  else
    delay(50);
}
