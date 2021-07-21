/*Receives picture via serial and saves to ftp*/

#define DEBUG_ESP              //comment out to deactivate debug console

#ifdef DEBUG_ESP
  #define pDBGln(x) Serial.println(x)
  #define pDBG(x)   Serial.print(x)
#else 
  #define pDBG(...)
  #define pDBGln(...)
#endif

#include "SerialTransfer.h"
SerialTransfer myTransfer;
struct img_meta_data{
  uint16_t counter;
  uint16_t imSize;
  uint16_t numLoops;
  uint16_t sizeLastLoop;
} ImgMetaData;
uint16_t packetCounter=1;
uint16_t bufferPointer=0;
char tempImageBuffer[32000];

#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;

// FTP Client Lib
#include "ESP32_FTPClient.h"

// Your WiFi credentials.
char ssid[] = "myssid";
char pass[] = "mypass";

// FTP Server credentials
char ftp_server[] = "192.168.0.1";
char ftp_user[]   = "pcborges"; 
char ftp_pass[]   = "mypass";

// picture name
String picPrefix ="";
String pic_name;  

// Connection timeout;
#define CON_TIMEOUT   10*1000                     // milliseconds

ESP32_FTPClient ftp(ftp_server, ftp_user, ftp_pass);

void setup(){
  //Serial.begin(Baud Rate, Data Protocol, Txd pin, Rxd pin);
  Serial.begin(115200);                                                     // Define and start serial monitor
  // 115200,256000,512000,962100
  Serial2.begin(962100, SERIAL_8N1); //Receiver_Txd_pin, Receiver_Rxd_pin); // Define and start Receiver serial port
  myTransfer.begin(Serial2);

  WiFi.begin(ssid, pass);
  pDBGln("Connecting Wifi...");
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      pDBG(".");
  }
  pDBGln("");
  pDBG("IP address: ");
  pDBGln(WiFi.localIP());
  pDBGln();

  // Initialize a NTPClient to get time
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600, GMT +8 = 28800, GMT -1 = -3600, GMT 0 = 0
  timeClient.setTimeOffset(3600*-3);  
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
      pic_name  = picPrefix;
      pic_name += getDateTime() + ".jpg";  
      FTP_upload();     
      packetCounter=1;
      bufferPointer=0;
      delay(2000);
      //while(1){}
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

void FTP_upload(){
  pDBG("Uploading via FTP: "); 
  ftp.OpenConnection();
  //ftp.ChangeWorkDir("/public_html/zyro/");
  //Create a file and write the image data to it;
  ftp.InitFile("Type I");
  const char *f_name = pic_name.c_str();
  ftp.NewFile( f_name ); 
  pDBGln( f_name );    
  ftp.WriteData((unsigned char*)tempImageBuffer, ImgMetaData.imSize);
  ftp.CloseFile();
  // Breath, withouth delay URL failed to update.
  pDBGln("Done... Waiting next transfer...");
  delay(100);
}

String getDateTime(){
  String curDateTime="";

  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
  // The formattedDate comes with the following format:
  // 2018-05-28T16:00:13Z
  // We need to extract date and time
  formattedDate = timeClient.getFormattedDate();

  #ifdef DEGUB_ESP
    // Extract date
    int splitT = formattedDate.indexOf("T");
    dayStamp = formattedDate.substring(0, splitT);
    // Extract time
    timeStamp = formattedDate.substring(splitT+1, formattedDate.length()-1);
    pDBGln(formattedDate);  
    pDBG("DATE: ");
    pDBGln(dayStamp);
    pDBG("HOUR: ");
    pDBGln(timeStamp);    
  #endif
  
  curDateTime  = formattedDate.substring(8,10);  //day
  curDateTime += formattedDate.substring(5,7);   //month
  curDateTime += formattedDate.substring(2,4);   //year
  curDateTime += formattedDate.substring(11,13); //hour
  curDateTime += formattedDate.substring(14,16); //minute
  curDateTime += formattedDate.substring(17,19); //seconds
  return curDateTime;
}
