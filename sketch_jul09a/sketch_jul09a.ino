// Receives and image over the hardware comm port and fowards it to the USB Serial port

#define DEBUG_ESP              // Comment out to deactivate debug console

#ifdef DEBUG_ESP
  #define pDBGln(x) Serial.println(x)
  #define pDBG(x)   Serial.print(x)
#else 
  #define pDBG(...)
  #define pDBGln(...)
#endif

#include "Arduino.h"
#include "SerialTransfer.h"

SerialTransfer myTransfer;
HardwareSerial Comm(1);

struct img_meta_data{
  uint16_t counter;
  uint16_t imSize;
  uint16_t numLoops;
  uint16_t sizeLastLoop;
} ImgMetaData;

uint16_t packetCounter=1;
uint16_t bufferPointer=0;
char tempImageBuffer[32000];

// picture name
String picPrefix ="";
String pic_name;  


void setup(){
  //Serial.begin(Baud Rate, Data Protocol, Txd pin, Rxd pin);
  Serial.begin(115200);                                                     // Define and start serial monitor
  // 115200,256000,512000,962100
  Comm.begin(962100,SERIAL_8N1,1,0);     //, Comm_Txd_pin, Comm_Rxd_pin); // Define and start Comm serial port
  myTransfer.begin(Comm);
}

void loop(){
  if(myTransfer.available())  {
    myTransfer.rxObj(ImgMetaData, sizeof(ImgMetaData));
    pDBG("Struct Data: ");
    pDBG(ImgMetaData.counter);
    pDBG(", ");
    pDBG(ImgMetaData.imSize);
    pDBG(", ");  
    pDBG(ImgMetaData.numLoops);
    pDBG(", ");
    pDBG(ImgMetaData.sizeLastLoop);
    pDBG(", PacketCounter: ");
    pDBGln(packetCounter);  

    if(ImgMetaData.counter==1){  
      copyToImageBuff(MAX_PACKET_SIZE-sizeof(ImgMetaData));
    }else{
      if(ImgMetaData.counter==packetCounter){  
        if(packetCounter<ImgMetaData.numLoops){        
          copyToImageBuff(MAX_PACKET_SIZE-sizeof(ImgMetaData));
        }else if(ImgMetaData.counter==packetCounter){
          copyToImageBuff(ImgMetaData.sizeLastLoop);     
        }
      }
    }
 
    if(packetCounter>ImgMetaData.numLoops){  
      pic_name  = picPrefix + ".jpg"; 
      
      Serial_upload();
         
      packetCounter=1;
      bufferPointer=0;
      delay(2000);
    }  

  }else if(myTransfer.status < 0) { 
    pDBG("ERROR: ");
    if(myTransfer.status == -1)
      pDBGln(F("CRC_ERROR"));
    else if(myTransfer.status == -2)
      pDBGln(F("PAYLOAD_ERROR"));
    else if(myTransfer.status == -3)
      pDBGln(F("STOP_BYTE_ERROR"));
  }
}
  

void copyToImageBuff(uint16_t dataLenght){
  for(int y=0;y<dataLenght;y++){
    tempImageBuffer[bufferPointer+y] = myTransfer.packet.rxBuff[y+sizeof(ImgMetaData)];
  } 
  bufferPointer+=dataLenght;
  packetCounter++;  

  pDBG("dataLenght: ");
  pDBG(dataLenght);  
  pDBG(", bufferPointer: ");
  pDBGln(bufferPointer);
}

void printBuf(char localBuff){
  pDBG(F("Pixel Values: { "));
  for (uint16_t k=0; k<sizeof(myTransfer.packet.rxBuff); k++){
    pDBG(myTransfer.packet.rxBuff[k]);
    if (k < (sizeof(myTransfer.packet.rxBuff) - 1))
      pDBG(F(", "));
    else
      pDBGln(F(" }"));
  }
}

void Serial_upload(){
  Serial.println(ImgMetaData.imSize);
  Serial.println(tempImageBuffer);

  pDBGln("Done... Waiting next transfer...");
  delay(100);
}
