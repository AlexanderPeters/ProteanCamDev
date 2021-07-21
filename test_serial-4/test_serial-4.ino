#include "SerialTransfer.h"


SerialTransfer myTransfer;
HardwareSerial Comm(1);

const String question = "ALIVE?";
char alive[] = "ALIVE";
char takeImage[] = "TAKEIMAGE";

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

void sendMessage(char arr[]) {
  uint16_t sendSize = 0;
  sendSize = myTransfer.txObj(arr, sendSize);
  myTransfer.sendData(sendSize);
}

void loop() {
  if (myTransfer.available()) {
    String received = receiveMessage();

    if (received == question) {
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
