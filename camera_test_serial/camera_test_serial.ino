//#define DEBUG_ESP              //comment out to deactivate debug console

#ifdef DEBUG_ESP
  #define pDBGln(x) Serial.println(x)
  #define pDBG(x)   Serial.print(x)
#else 
  #define pDBG(...)
  #define pDBGln(...)
#endif

#include "esp_camera.h"
#include "Arduino.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include "SerialTransfer.h"

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

SerialTransfer myTransfer;
HardwareSerial Comm(1);
struct img_meta_data{
  uint16_t counter;
  uint16_t imSize;
  uint16_t numLoops;
  uint16_t sizeLastLoop;
} ImgMetaData;
const uint16_t PIXELS_PER_PACKET = MAX_PACKET_SIZE - sizeof(ImgMetaData);



// Blink LED so we know module is powered and working
// the number of the LED pin
const int ledPin = 4;  // 4 corresponds to GPIO4 Onboard Cam Flash LED

// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int ledResolution = 8;

// Loop management
int loopTimers[] = {0, 0}; // Loop 1 Clock, Loop 2 Clock
int startTime = 0;
int endTime = 0;

void setup(){
  // Blink LED
    // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, ledResolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);

  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  Serial.begin(115200);                     // Define and start serial monitor
  // 115200,256000,512000,962100
  Comm.begin(962100, SERIAL_8N1,15,14);     //, Comm_Txd_pin, Comm_Rxd_pin); // Define and start Comm serial port
  myTransfer.begin(Comm);
 
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
 
//  if(psramFound()){
//    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
//    config.jpeg_quality = 10;
//    config.fb_count = 2;
//  } else {
//    config.frame_size = FRAMESIZE_SVGA;
//    config.jpeg_quality = 12;
//    config.fb_count = 1;
//  }

    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
 
  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK){
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  startTime = micros() / 1000;
  endTime = micros() / 1000;
}

void loop(){ // Allows for the parralelization of task through independent timers
  int previousLoopDuration = endTime - startTime;
  Serial.print("Loop Time: ");
  Serial.println(previousLoopDuration);
  startTime = micros() / 1000;
  for(int i = 0; i < sizeof(loopTimers); i++) {// Increment through list of paralleled tasks (loops)
    loopTimers[i] = loopTimers[i] - previousLoopDuration; // Decrement clock for each loop
    if(loopTimers[i] <= 0)
      runLoop(i); // Clock has expired, loop can run
  }
  endTime = micros() / 1000;
}


// Blink loop
boolean ledOnOff = true; // true = turn on, false = turn off
void runLoop(int loopIndex) {
  if(loopIndex == 0) { // LED Blink Loop
    if(ledOnOff)
        ledcWrite(ledChannel, 1); // Turn On 
    else
        ledcWrite(ledChannel, 0); // Turn Off
        
    ledOnOff = !ledOnOff;
    loopTimers[loopIndex] += 1000; // Wait 1000 milliseconds until logic runs again    
  }
  else if(loopIndex == 1) { // Camera image capture loop
    Serial.println("Starting in camera");
    uint16_t startIndex = 0;
    camera_fb_t * fb = NULL;
    //Take Picture with Camera
    fb = esp_camera_fb_get(); 
     
    ImgMetaData.imSize       = fb->len;                             //sizeof(myFb);
    ImgMetaData.numLoops     = (fb->len / PIXELS_PER_PACKET) + 1;   //(sizeof(myFb)/PIXELS_PER_PACKET) + 1; 
    ImgMetaData.sizeLastLoop = fb->len % PIXELS_PER_PACKET;         //(sizeof(myFb)%PIXELS_PER_PACKET);

    Serial.print("NumLoops: ");
    Serial.println(ImgMetaData.numLoops);
    
  
    for(ImgMetaData.counter=1; ImgMetaData.counter<=ImgMetaData.numLoops; ImgMetaData.counter++){
      myTransfer.txObj(ImgMetaData, sizeof(ImgMetaData));
      //printStructBuf();
      stuffPixels(fb->buf, startIndex, sizeof(ImgMetaData), PIXELS_PER_PACKET);
      //stuffPixels(myFb, startIndex, sizeof(ImgMetaData), PIXELS_PER_PACKET);
      //printStructBuf();   
      myTransfer.sendData(MAX_PACKET_SIZE);
       
      pDBGln(F("Sent:"));
      pDBG(F("img.counter: "));      pDBGln((uint16_t)((myTransfer.packet.txBuff[1] << 8) | myTransfer.packet.txBuff[0]));
      pDBG(F("img.imSize: "));       pDBGln((uint16_t)((myTransfer.packet.txBuff[3] << 8) | myTransfer.packet.txBuff[2]));
      pDBG(F("img.numLoops: "));     pDBGln((uint16_t)((myTransfer.packet.txBuff[5] << 8) | myTransfer.packet.txBuff[4]));
      pDBG(F("img.sizeLastLoop: ")); pDBGln((uint16_t)((myTransfer.packet.txBuff[7] << 8) | myTransfer.packet.txBuff[6]));
  
      startIndex += PIXELS_PER_PACKET;    
      delay(100);
      //printBuf();  
    }
    esp_camera_fb_return(fb);   //clear camera memory

    Serial.println("Done in camera");
    loopTimers[loopIndex] += 15000;
  }
}

void printStructBuf(){
  pDBG(F("Internal Struct: { "));
  for (uint16_t k=0; k<sizeof(ImgMetaData); k++){
    pDBG(myTransfer.packet.txBuff[k]);
    if (k<(sizeof(ImgMetaData)-1))
      pDBG(F(", "));
    else
      pDBGln(F(" }"));
  }
}

void printBuf(){
  pDBG(F("Pixel Values: { "));
  for (uint16_t k=8; k<MAX_PACKET_SIZE; k++){
    pDBG(myTransfer.packet.txBuff[k]);
    if (k < (MAX_PACKET_SIZE - 1))
      pDBG(F(", "));
    else
      pDBGln(F(" }"));
  }
  pDBGln();
}

void stuffPixels(const uint8_t * pixelBuff, const uint16_t &bufStartIndex, const uint16_t &txStartIndex, const uint16_t &len){
  uint16_t txi = txStartIndex;
  for (uint16_t i=bufStartIndex; i<(bufStartIndex + len); i++)  {
    myTransfer.packet.txBuff[txi] = pixelBuff[i];
    txi++;
  }
}
