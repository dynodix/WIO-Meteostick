/*
  Helium-E5-DHT22 
  https://tutorial.cytron.io/2022/02/28/sending-data-to-helium-console-using-grove-lora-e5/
  https://gist.github.com/NorHairil/808ec64b1d4eac3f4b6f286a9392abce 
  https://github.com/limengdu/Seeed-Studio-LoRaWAN-Dev-Kit

*/

#include <Arduino.h>
//#include "disk91_LoRaE5.h"
#include "DHT.h"
#include"TFT_eSPI.h"
#include "RTC_SAMD51.h"
#include "DateTime.h"
// WiFi 
//#include "rpcWiFi.h"  // https://wiki.seeedstudio.com/Wio-Terminal-Wi-Fi/
//#include <WiFiMulti.h>
#include <millisDelay.h>
//needed for library https://github.com/Seeed-Studio/Seeed_Arduino_rpcWiFiManager
#include <rpcWiFi.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>  
#include "Seeed_HM330X.h"
#include <FlashStorage_SAMD.h>
#include <HTTPClient.h>

#define SERIAL Serial 
#define SERIAL_OUTPUT Serial 

// STORAGE ---------------------
#define FLASH_DEBUG 0
const int WRITTEN_SIGNATURE = 0xBEEFDEED;

typedef struct
{
  char mqttserver[80];
  char mqttuser[30];
  char mqttpass[30];
  char wifissid[80];
  char wifipass[80];
} hassconf;

hassconf hassio;
int shouldsave = 0;
int prevmin = 0;

// AIR QUALITY -----------------
HM330X sensor;

uint8_t buf[30]; 
int ppm25;


const char* str[] = {"sensor num: ", 
                     "PM1.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM2.5 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM10 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM10 concentration(Atmospheric environment,unit:ug/m3): ",
                    };

HM330XErrorCode print_result(const char* str, uint16_t value) {
    if (NULL == str) {
        return ERROR_PARAM;
    }
    SERIAL_OUTPUT.print(str);
    SERIAL_OUTPUT.println(value);
    return NO_ERROR;
}

/*parse buf with 29 uint8_t-data*/
HM330XErrorCode parse_result(uint8_t* data) {
    uint16_t value = 0;
    if (NULL == data) {
        return ERROR_PARAM;
    }
    for (int i = 1; i < 8; i++) {
        value = (uint16_t) data[i * 2] << 8 | data[i * 2 + 1];
        //print_result(str[i - 1], value);
        if (i==4) {
          ppm25=(int) value;
          print_result("PPM25: ", value);
        } 
    }

    return NO_ERROR;
}

HM330XErrorCode parse_result_value(uint8_t* data) {
    if (NULL == data) {
        return ERROR_PARAM;
    }
    for (int i = 0; i < 28; i++) {
        SERIAL_OUTPUT.print(data[i], HEX);
        SERIAL_OUTPUT.print("  ");
        if ((0 == (i) % 5) || (0 == i)) {
            SERIAL_OUTPUT.println("");
        }
    }
    uint8_t sum = 0;
    for (int i = 0; i < 28; i++) {
        sum += data[i];
    }
    if (sum != data[28]) {
        SERIAL_OUTPUT.println("wrong checkSum!!");
    }
    SERIAL_OUTPUT.println("");
    return NO_ERROR;
}

// -------------------------------

#define DHTPIN 1 // what pin we're connected to
// Uncomment whatever type you're using!
// #define DHTTYPE DHT11 // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
 
DHT dht(DHTPIN, DHTTYPE);
//Disk91_LoRaE5 lorae5(&Serial); // Where the AT command and debut traces are printed
//#define Frequency DSKLORAE5_ZONE_EU868
//char deveui[] = "2C F7 F1 C0 42 80 01 A2";
//char appeui[] = "8000000000000009";
//char appkey[] = "D4 61 9A 08 21 D7 B2 A0 D0 5C 16 05 02 14 2C 87"; 

//uint8_t deveui[] = { 0x2C, 0xF7, 0xF1, 0xC0, 0x42, 0x80, 0x01, 0xA2 };
//uint8_t appeui[] = { 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09 };
//uint8_t appkey[] = { 0xD4, 0x61, 0x9A, 0x08, 0x21, 0xD7, 0xB2, 0xA0, 0xD0, 0x5C, 0x16, 0x05, 0x02, 0x14, 0x2C, 0x87 };

//const char* ssid = "okolje-informatika";
//const char* password =  "041.dino.687056";
//const char* ssid = "dynomobile";
//const char* password =  "tatitati19";

//WiFiMulti wifiMulti;
WiFiManager wifiManager;
unsigned int localPort = 2390;
char timeServer[] = "ntp.siol.net";
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];
millisDelay updateDelay;

//TFT Display Setup
TFT_eSPI tft;
TFT_eSprite spr = TFT_eSprite(&tft);  //sprite

// Domoticz
//char dz_server[40];
//char dz_aqdevice[4];
//char dz_thdevice[4];
//char dz_temps[10];
//char dz_humis[10];
//char dz_ppm2s[10];
char hass_server[40];
char hass_user[40];
char hass_pass[40];
char wifi_ssid[80];
char wifi_pass[80];

char temps[10];
char humis[10];
char clocks[20];
char ppm2s[20];

RTC_SAMD51 rtc;
// define WiFI client
WiFiClient client;
//The udp library class
WiFiUDP udp;
// localtime
unsigned long devicetime;
char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

void data_record(int t, int h, int f,int s, uint8_t data[5])
{ 
  data[0] = t;
  data[1] = h;
  data[2] = f;
  data[3] = s / 100;
  data[4] = s % 100;
}

WiFiManagerParameter custom_text("<p>Home Assitant setup:</p>");
WiFiManagerParameter custom_mqttserver("server", "mqtt server ip", hass_server, 40);
WiFiManagerParameter custom_user("username", "mqtt Username", hass_user, 40);
WiFiManagerParameter custom_pass("password", "mqtt Password", hass_pass, 40);

void configModeCallback (WiFiManager *wifiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(wifiManager->getConfigPortalSSID());
  //strcopy()
  //wifiManager->getConfigPortalSSID();
  //wifiManager->getPassword();
  //dz_server = domoticz_server.getValue();

  shouldsave=1;
}

unsigned long getNTPtime() {
 
    // module returns a unsigned long time valus as secs since Jan 1, 1970 
    // unix time or 0 if a problem encounted
 
    //only send data when connected
    if (WiFi.status() == WL_CONNECTED) {
        //initializes the UDP state
        //This initializes the transfer buffer
        udp.begin(WiFi.localIP(), localPort);
 
        sendNTPpacket(timeServer); // send an NTP packet to a time server
        // wait to see if a reply is available
        delay(1000);
        if (udp.parsePacket()) {
            Serial.println("ntp udp packet received");
            Serial.println("");
            // We've received a packet, read the data from it
            udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
 
            //the timestamp starts at byte 40 of the received packet and is four bytes,
            // or two words, long. First, extract the two words:
 
            unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
            unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
            // combine the four bytes (two words) into a long integer
            // this is NTP time (seconds since Jan 1 1900):
            unsigned long secsSince1900 = highWord << 16 | lowWord;
            // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
            const unsigned long seventyYears = 2208988800UL;
            // subtract seventy years:
            unsigned long epoch = secsSince1900 - seventyYears;
 
            // adjust time for timezone offset in secs +/- from UTC
            // WA time offset from UTC is +8 hours (28,800 secs)
            // + East of GMT
            // - West of GMT
            //long tzOffset = 28800UL;
            long tzOffset = 3600UL;
 
            // WA local time 
            unsigned long adjustedTime;
            return adjustedTime = epoch + tzOffset;
        }
        else {
            // were not able to parse the udp packet successfully
            // clear down the udp connection
            udp.stop();
            return 0; // zero indicates a failure
        }
        // not calling ntp time frequently, stop releases resources
        udp.stop();
    }
    else {
        // network not connected
        return 0;
    }
 
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(const char* address) {
    // set all bytes in the buffer to 0
    for (int i = 0; i < NTP_PACKET_SIZE; ++i) {
        packetBuffer[i] = 0;
    }
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;
 
    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    udp.beginPacket(address, 123); //NTP requests are to port 123
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    udp.endPacket();
}

void setup(void)
{
    Serial.begin(9600);
    //pinMode(LED_BUILTIN, OUTPUT);
    //digitalWrite(LED_BUILTIN, HIGH);
    uint32_t start = millis();
    while ( !Serial && (millis() - start) < 10000 );  // Open the Serial Monitor to get started or wait for 1.5"
    Serial.print("Wio Terminal WiFi Init\r\n");    

    pinMode(WIO_KEY_A, INPUT_PULLUP);
    pinMode(WIO_KEY_B, INPUT_PULLUP);
    pinMode(WIO_KEY_C, INPUT_PULLUP);    

    // Check signature at address 0
    int signature;
    uint16_t storedAddress = 0;
  
    EEPROM.get(storedAddress, signature);
    if (signature == WRITTEN_SIGNATURE)
    {
       EEPROM.get(storedAddress + sizeof(signature), hassio);
       //char server[40];
       //char thid[4];
       //char aqid[4];
       // Say hello to the returning user!
       Serial.print("Homeassistant settings loaded: "); 
       Serial.print(hassio.mqttserver); Serial.print(" "); Serial.print(hassio.mqttuser);Serial.print(" "); Serial.print(hassio.mqttpass);
       //strcpy(dz_server, domoticz.server);
       //dz_server=domoticz.server;
       memcpy(hass_server, hassio.mqttserver, sizeof(hassio.mqttserver));
       strcpy(hass_user, hassio.mqttuser);
       strcpy(hass_pass, hassio.mqttpass);       
       Serial.println("");
    } else {
       Serial.println("No EEPROM data!");
    }   
 
    tft.begin();
    tft.setRotation(3);

    //Display the Headings
    tft.fillScreen(TFT_BLACK);
    tft.setFreeFont(&FreeSansBold18pt7b);
    tft.setTextColor(TFT_WHITE);
    tft.drawString("Wifi T/H+AQ proto", 5, 10 , 1);
    
    tft.setFreeFont(&FreeSans12pt7b);
    tft.setTextColor(TFT_GREEN);
    //tft.drawString("Flies:", 7 , 65 , 1);
    //tft.drawString("Moisture:", 180 , 65 , 1);
    tft.drawString("Temperature:", 7 , 65 , 1);
    tft.drawString("Humidity:", 180 , 65 , 1);
    tft.drawString("2.5 pPM AQI:", 7 , 140 , 1);

    //tft.setFreeFont(&FreeSans12pt7b);
    //tft.setTextColor(TFT_WHITE);
    //tft.drawString("wifi connecting...", 80 , 180 , 1);
    spr.createSprite(140, 80); // w,h
    //spr.setFreeFont(&FreeSans12pt7b);
    spr.drawString("wifi connecting...", 0, 1);
    spr.pushSprite(90, 180); // x,y
    spr.deleteSprite();
       
   // Set WiFi to station mode and disconnect from an AP if it was previously connected
   //wifiMulti.addAP("DOMOTICZ", "041.dino.687056");
   //wifiMulti.addAP("dynomobile", "tatitati19");
   
//   WiFi.begin("dynomobile", "tatitati19");
   //WiFi.begin("DOMOTICZ", "041.dino.687056");
//   if (strlen(domoticz.wifi)>2){
//      Serial.print("starting CLIENT: ");Serial.println(domoticz.wifi);
//      WiFi.begin(domoticz.wifi, domoticz.pass);
//   }
   //WiFi.begin("okolje-informatika", "041.dino.687056");
   
   //wifiMulti.addAP("ssid_from_AP_3", "your_password_for_AP_3");
   //WiFi.mode(WIFI_STA);
   //WiFi.disconnect();
   //WiFiManagerParameter custom_text("<p>Domoticz setup:</p>");

   wifiManager.addParameter(&custom_text);
   Serial.print(hass_server);Serial.print("  ");
   wifiManager.addParameter(&custom_mqttserver);
   wifiManager.addParameter(&custom_user);
   wifiManager.addParameter(&custom_pass);
   wifiManager.setAPCallback(configModeCallback);

   //wifiManager.connectWifi("DOMOTICZ","041.dino.687056");
   //wifiManager.autoConnect();
 
   //Serial.println("Connecting to WiFi..");
   //WiFi.begin(ssid, password);
   /*
   if (wifiMulti.run() == WL_CONNECTED) {
        Serial.println("");
        Serial.println("WiFi connected");
        //Serial.println("IP address: ");
        //Serial.println(WiFi.localIP());
    }
    while (WiFi.status() != WL_CONNECTED) {
      Serial.println("Connecting to WiFi..");
      delay(1000);
    }  
    */
    Serial.println("Connected to the WiFi network");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP()); // prints out the device's IP address    
        
    //if (WiFi.status() == WL_CONNECTED) {
    //    delay(500);
    //    Serial.println("Connecting to WiFi..");
    //WiFi.begin(ssid, password);
    //    devicetime = getNTPtime();
    //    // check if rtc present
    //    if (devicetime == 0) {
    //        Serial.println("Failed to get time from network time server.");
    //    }
    //}    

  if (shouldsave == 1) 
  {
    strcpy(hass_server, custom_mqttserver.getValue());
    Serial.print("HomeAssistant mqtt server: ");Serial.println(hass_server);
    strcpy(hass_user, custom_user.getValue());
    strcpy(hass_pass, custom_pass.getValue());
    //Serial.print("Domozicz device: ");Serial.println(dz_thdevice);  
    // https://randomnerdtutorials.com/wifimanager-with-esp8266-autoconnect-custom-parameter-and-manage-your-ssid-and-password/
    //uint16_t storedAddress = 0; 
    strcpy(hassio.mqttserver, hass_server);
    strcpy(hassio.mqttuser, hass_user);
    strcpy(hassio.mqttpass, hass_pass);
    //strcpy(hassio.wifissid, WiFi.SSID());
    strcpy(hassio.wifissid, &wifiManager.getSSID()[0]);
    //char hassio.wifissid[wifiManager.getConfigPortalSSID().length() + 1]; 
    //strcpy(hassio.wifissid, wifiManager.getConfigPortalSSID().c_str());
//    strcpy(hassio.wifipass, &wifiManager.getPassword()[0]);
    
    EEPROM.put(storedAddress, WRITTEN_SIGNATURE);
    EEPROM.put(storedAddress + sizeof(WRITTEN_SIGNATURE), hassio);
  
    if (!EEPROM.getCommitASAP())
    {
       Serial.println("CommitASAP not set. Need commit()");
       EEPROM.commit();
    }    
  }
   /*while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi..");
        WiFi.begin(ssid, password);
    }
    Serial.println("Connected to the WiFi network");
    Serial.print("IP Address: ");
    Serial.println (WiFi.localIP()); // prints out the device's IP address
    */
    //Serial.println(Datetime(devicetime));
    devicetime = getNTPtime();
    // check if rtc present
    if (devicetime == 0) {
       Serial.println("Failed to get time from network time server.");
    }    

    DateTime now = DateTime(F(__DATE__), F(__TIME__));
    Serial.println("adjust time!");
    rtc.begin();
    //rtc.adjust(now);
    //rtc.adjust(DateTime(devicetime));
    rtc.adjust(devicetime);
    
    now = rtc.now();
    Serial.print(now.year(), DEC);Serial.print('/');Serial.print(now.month(), DEC);Serial.print('/');Serial.print(now.day(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);Serial.print(':');Serial.print(now.minute(), DEC);Serial.print(':');Serial.print(now.second(), DEC);
    Serial.println();  
    prevmin=now.minute();
  

  // Setup the LoRaWan Credentials
  
  // Initialize dht reader 
  dht.begin();

  // Initializy AQI sensor
  if(sensor.init())
    {
        SERIAL.println("HM330X init failed!!!");
        //while(1);
    }
  
  updateDelay.start(12 * 60 * 60 * 1000); // update time via ntp every 12 hrs
        
}
 
void loop(void)
{
    float temp = 0;
    float humi = 0;
    int SoilMoisture = 0;
    int Flies = 0;

    if ((digitalRead(WIO_KEY_C) == LOW) && (digitalRead(WIO_KEY_B) == LOW))
     {
       if  (digitalRead(WIO_KEY_A) == LOW) {
          wifiManager.resetSettings();
       }
       Serial.println("RESET proceure pressed");
       delay(1000);
       NVIC_SystemReset();
     }
    

    
    DateTime now = DateTime(F(__DATE__), F(__TIME__));
    static uint8_t data[5] = { 0x00 };  //Use the data[] to store the values of the sensors    

    if (updateDelay.justFinished()) { // 12 hour loop
        // repeat timer
        updateDelay.repeat(); // repeat
 
        // update rtc time
        devicetime = getNTPtime();
        if (devicetime == 0) {
            Serial.println("Failed to get time from network time server.");
        }
        else {
            rtc.adjust(DateTime(devicetime));
            Serial.println("");
            Serial.println("rtc time updated.");
            // get and print the adjusted rtc time
            now = rtc.now();
            Serial.print("Adjusted RTC time is: ");
            Serial.println(now.timestamp(DateTime::TIMESTAMP_FULL));
        }
    }
    
    temp = dht.readTemperature();
    humi = dht.readHumidity();    
 
    now = rtc.now();
    sprintf(temps, "%g *C", temp);
    sprintf(humis, "%g %%", humi);
    //sprintf(dz_temps, "%g", temp);
    //sprintf(dz_humis, "%g", humi);    
    // sprintf( str, "%s/%s %s:%s", Day, Month, Hour, Min );
    //now.hour()
    sprintf(clocks, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    //temps = strcat(temps, " *C")
   
    data_record(temp,humi,Flies,SoilMoisture,data);    
      
    //Display the Values on Screen
    spr.createSprite(100, 30);
    spr.setFreeFont(&FreeSansBold12pt7b);
    spr.setTextColor(TFT_WHITE);
    /*spr.drawNumber(Flies, 0, 0, 1);
    spr.pushSprite(15, 100);
    spr.deleteSprite();
    spr.createSprite(150, 30);
    spr.drawNumber(SoilMoisture, 0, 0, 1);
    spr.pushSprite(180, 100);
    spr.deleteSprite();*/

    spr.createSprite(100, 30);
    spr.drawString(temps, 0, 1);
    spr.pushSprite(15, 100);
    spr.deleteSprite();

    spr.createSprite(150, 30);
    spr.drawString(humis, 0, 1);
    spr.pushSprite(180, 100);
    spr.deleteSprite();
    
  // clock
    spr.createSprite(140, 80); // w,h
    spr.setFreeFont(&FreeSansBold18pt7b);
    spr.drawString(clocks, 0, 1);
    spr.pushSprite(90, 180); // x,y
    spr.deleteSprite();

    //if (now.minute() == 7 or now.minute() == 37)
    if ((now.second() % 10) == 0)
    {
      if (sensor.read_sensor_value(buf, 29)) {
           SERIAL_OUTPUT.println("HM330X read result failed!!");
      }
      //parse_result_value(buf);
      parse_result(buf);
      SERIAL_OUTPUT.println("");
      
      sprintf(ppm2s, "%d ppm", ppm25);
      //sprintf(dz_ppm2s, "%d", ppm25);
      spr.createSprite(150, 30);
      spr.setFreeFont(&FreeSansBold12pt7b);
      spr.drawString(ppm2s, 0, 1);
      spr.pushSprite(180, 140);
      spr.deleteSprite();


      Serial.print("Temperature: ");
      //Serial.print(temp);
      //Serial.print("   ");    
      Serial.print(temps);    
      //Serial.println(" *C");
      Serial.print(" \t");
      Serial.print("Humidity: ");
      Serial.print(humis);
      Serial.print(" \t");
      Serial.println(clocks);
      //delay(60000);
    } 
    
    if (now.minute() != prevmin )
    {
        prevmin=now.minute();
        
      //delay(300);         
    } 
    delay(800);
 // delay(1800000);
}
