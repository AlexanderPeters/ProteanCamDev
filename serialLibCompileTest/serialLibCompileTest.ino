#include "SerialTransfer.h"


SerialTransfer myTransfer;


void setup()
{
  Serial.begin(115200);
  myTransfer.begin(Serial);
}


void loop()
{
  if(myTransfer.available())
  {
    // send all received data back
    for(uint16_t i=0; i < myTransfer.bytesRead; i++)
      myTransfer.txBuff[i] = myTransfer.rxBuff[i];

    myTransfer.sendData(myTransfer.bytesRead);
  }
}
