#include <Arduino.h>
#include <esp_camera.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "PubSubClient.h"
#include "camera_pins.h"


WiFiClient Wclient;
PubSubClient client(Wclient);
const char *hostA = "esp32";
const char *ssid = "belkin,b54";
const char *password = "yad7c8cw";
const char *mqtt_server = "default";
boolean shotflag = false;
boolean WiFiDisconnect = false;
String msg;
int timeCount = 0;
void callback(char *topic, byte *message, unsigned int length);
void getimg();
void reconnect();
void setupCamera();
void WiFiEvent(WiFiEvent_t event);
void setup()
{
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
    }
    Serial.println("WiFi connected");
    Serial.println(WiFi.localIP());
    WiFi.onEvent(WiFiEvent);
    client.setServer(mqtt_server, 1890);
    client.setCallback(callback);
    setupCamera();
    Serial.println("Ready");
}

void loop()
{
    reconnect();
    client.loop();
    if (shotflag == true)
    {
        getimg();
        shotflag = false;
    }
}

void getimg()
{

    camera_fb_t *fb = esp_camera_fb_get();
    // int tranCount = (fb->len * 2 + (1000 - 1)) / 1000;
    if (fb)
    {
        Serial.printf("width: %d, height: %d, buf: 0x%x, len: %d\n", fb->width, fb->height, fb->buf, fb->len);
        char data[4104];
        for (int i = 0; i < fb->len; i++)
        {
            sprintf(data, "%02X", *((fb->buf + i)));
            msg += data;
            if (msg.length() == 4096)
            {
                timeCount += 1;
                client.beginPublish("img", msg.length(), 0);
                client.print(msg);
                client.endPublish();
                msg = "";
                // Serial.println(timeCount);
            }
        }
        if (msg.length() > 0)
        {
            client.beginPublish("img", msg.length(), 0);
            client.print(msg);
            client.endPublish();
            msg = "";
        }
        client.publish("img", "1");
        timeCount = 0;
        esp_camera_fb_return(fb);
    }
}

void callback(char *topic, byte *payload, unsigned int length)
{
    // Serial.print("Message arrived [");
         // Serial.print (topic); // print information topics
    // Serial.print("] ");
    // for (int i = 0; i < length; i++)
    // {
         // Serial.print ((char) payload [i]); // Print the subject matter
    // }
    // Serial.println();
    if ((char)payload[0] == '1')
    {
        shotflag = true;
    }
    if ((char)payload[0] == '0')
    {
        shotflag = false;
    }
}
void reconnect()
{
    if (WiFiDisconnect)
    {
        WiFi.reconnect();
    }
    while (!client.connected())
    {
        client.connect("EspClient");
        client.subscribe("CAMcontrol");
    }
}

void setupCamera()
{
    camera_config_t config;
        config.pin_pwdn = PWDN_GPIO_NUM;
        config.pin_reset = RESET_GPIO_NUM;
        config.pin_xclk = XCLK_GPIO_NUM;
        config.pin_sscb_sda = SIOD_GPIO_NUM;
        config.pin_sscb_scl = SIOC_GPIO_NUM;
        config.pin_d7 = Y9_GPIO_NUM;
        config.pin_d6 = Y8_GPIO_NUM;
        config.pin_d5 = Y7_GPIO_NUM;
        config.pin_d4 = Y6_GPIO_NUM;
        config.pin_d3 = Y5_GPIO_NUM;
        config.pin_d2 = Y4_GPIO_NUM;
        config.pin_d1 = Y3_GPIO_NUM;
        config.pin_d0 = Y2_GPIO_NUM;
        config.pin_vsync = VSYNC_GPIO_NUM;
        config.pin_href = HREF_GPIO_NUM;
        config.pin_pclk = PCLK_GPIO_NUM;
        config.xclk_freq_hz = 20000000;
        config.ledc_timer = LEDC_TIMER_0;
        config.ledc_channel = LEDC_CHANNEL_0;
        config.pixel_format = PIXFORMAT_JPEG;
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 10;
        config.fb_count = 2;

    esp_err_t err = esp_camera_init(&config);
    Serial.printf("esp_camera_init: 0x%x\n", err);

    sensor_t *s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_XGA);
}

void WiFiEvent(WiFiEvent_t event)
{
    switch (event)
    {
    case SYSTEM_EVENT_STA_DISCONNECTED:
        WiFiDisconnect = true;
    case SYSTEM_EVENT_STA_CONNECTED:
        WiFiDisconnect = false;
    }
}
