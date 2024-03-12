/*

*/
#include <cdcftdi.h>
#include <usbhub.h>
#include <Arduino.h>
//#include "disk91_LoRaE5.h"
//#include "DHT.h"
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
#include <ArduinoJson.h>
#include <ArduinoHA.h>
//#include <SoftwareSerial.h>
//#include <Wire.h>
//#include <wiring_private.h>
#include "SAMCrashMonitor.h"
// https://github.com/cyrusbuilt/SAMCrashMonitor?tab=readme-ov-file

//SoftwareSerial mySerial(D0, D1); // RX, TX

//static Uart Serial3(&sercom3, PIN_WIRE_SCL, PIN_WIRE_SDA, SERCOM_RX_PAD_1, UART_TX_PAD_0);

#define SERIAL Serial1
#define SERIAL_OUTPUT Serial1
#define SERIAL_PORT_MONITOR Serial1

// STORAGE ---------------------
#define FLASH_DEBUG 0
const int WRITTEN_SIGNATURE = 0xBEEFDEED;

// -------------------------------------------------------------- USB 

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
//#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
//#define SerialDebug SERIAL_PORT_MONITOR
//#else
//#define SerialDebug Serial1
//#endif

class FTDIAsync : public FTDIAsyncOper
{
  public:
    uint8_t OnInit(FTDI *pftdi);
};

uint8_t FTDIAsync::OnInit(FTDI *pftdi)
{
  uint8_t rcode = 0;

  rcode = pftdi->SetBaudRate(115200);

  if (rcode)
  {
    ErrorMessage<uint8_t>(PSTR("SetBaudRate"), rcode);
    return rcode;
  }
  rcode = pftdi->SetFlowControl(FTDI_SIO_DISABLE_FLOW_CTRL);

  if (rcode)
    ErrorMessage<uint8_t>(PSTR("SetFlowControl"), rcode);

  return rcode;
}

USBHost          UsbH;
USBHub           Hub(&UsbH);
FTDIAsync        FtdiAsync;
FTDI             Ftdi(&UsbH, &FtdiAsync);
int startingproc;

// --------------------------------------------------------------

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
float temp = 0;
float humi = 0;
int SoilMoisture = 0;
int Flies = 0;  
float Barometer = 0;
float WindSpeed = 0;
float tempout = 0;
float humiout = 0;
float WindDirection = 0;
float RainClicks = 0; // remember to reset rainclicks at midnight
float SolarIndex = 0;
float UVIndex = 0;
float Battery = 50.0;

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
    //TelnetStream.println(str);
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

char hass_server[40];
char hass_user[40];
char hass_pass[40];
char wifi_ssid[80];
char wifi_pass[80];

char temps[10];
char humis[10];
char baros[10];
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
  SERIAL_OUTPUT.println("Entered config mode");
  SERIAL_OUTPUT.println(WiFi.softAPIP());
  SERIAL_OUTPUT.println(wifiManager->getConfigPortalSSID());
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
            SERIAL_OUTPUT.println("ntp udp packet received");
            SERIAL_OUTPUT.println("");
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

byte mac[] = {0x00, 0x10, 0xFA, 0x6E, 0x38, 0x4A};
HADevice device(mac, sizeof(mac));
HAMqtt mqtt(client, device);

//HASensorNumber analogSensorTI("TemperatureInside");
HASensorNumber analogSensorTI("TemperatureInside", HASensorNumber::PrecisionP1);
HASensorNumber analogSensorHI("HumidityInside", HASensorNumber::PrecisionP1);
HASensorNumber analogSensorTO("TemperatureOutside", HASensorNumber::PrecisionP1);
HASensorNumber analogSensorHO("HumidityOutside", HASensorNumber::PrecisionP1);
HASensorNumber analogSensorAQ25("DavisAQI25", HASensorNumber::PrecisionP1);
HASensorNumber analogSensorBPS("BarometricPressure", HASensorNumber::PrecisionP1);
HASensorNumber analogSensorWindSpeed("WindSpeed", HASensorNumber::PrecisionP1);
HASensorNumber analogSensorWindDir("WindDirection", HASensorNumber::PrecisionP1);
HASensorNumber analogSensorRainClicks("RainClicks", HASensorNumber::PrecisionP1);
HASensorNumber analogSensorUVIndex("UVIndex", HASensorNumber::PrecisionP1);
HASensorNumber analogSensorSolarIndex("SolarIndex", HASensorNumber::PrecisionP1);

HASensorNumber analogSensorSolarPanel("SolarPanel", HASensorNumber::PrecisionP1);

void setup(void)
{
  //SERIAL_OUTPUT.begin(9600);

  //pinPeripheral(D0, PIO_SERCOM_ALT);
  //pinPeripheral(D1, PIO_SERCOM_ALT);
  SERIAL_OUTPUT.begin(115200);
  //pinPeripheral(PIN_WIRE_SCL, PIO_SERCOM_ALT);
  //pinPeripheral(PIN_WIRE_SDA, PIO_SERCOM_ALT);


    uint32_t start = millis();
    while ( !Serial && (millis() - start) < 10000 );  // Open the Serial Monitor to get started or wait for 1.5"
    SERIAL_OUTPUT.print("Wio Terminal WiFi Init\r\n");    

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
       // Say hello to the returning user!
       SERIAL_OUTPUT.print("Homeassistant settings loaded: "); 
       SERIAL_OUTPUT.print(hassio.mqttserver); SERIAL_OUTPUT.print(" User="); SERIAL_OUTPUT.print(hassio.mqttuser);SERIAL_OUTPUT.print(" ,Pass="); SERIAL_OUTPUT.print(hassio.mqttpass);
       memcpy(hass_server, hassio.mqttserver, sizeof(hassio.mqttserver));
       strcpy(hass_user, hassio.mqttuser);
       strcpy(hass_pass, hassio.mqttpass);       
       SERIAL_OUTPUT.println("");
    } else {
       SERIAL_OUTPUT.println("No EEPROM data!");
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
    tft.drawString("Barometer:", 7 , 140 , 1);
    //tft.drawString("2.5 pPM AQI:", 7 , 140 , 1);

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

   wifiManager.addParameter(&custom_text);
   SERIAL_OUTPUT.print(hass_server);SERIAL_OUTPUT.print("  ");
   wifiManager.addParameter(&custom_mqttserver);
   wifiManager.addParameter(&custom_user);
   wifiManager.addParameter(&custom_pass);
   wifiManager.setAPCallback(configModeCallback);

   //wifiManager.connectWifi("DOMOTICZ","041.dino.687056");
   wifiManager.autoConnect();

   SERIAL_OUTPUT.println("Connected to the WiFi network");
   SERIAL_OUTPUT.print("IP Address: ");
   SERIAL_OUTPUT.println(WiFi.localIP()); // prints out the device's IP address    
      
  

  if (shouldsave == 1) 
  {
    strcpy(hass_server, custom_mqttserver.getValue());
    SERIAL_OUTPUT.print("HomeAssistant mqtt server: ");SERIAL_OUTPUT.println(hass_server);
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
       SERIAL_OUTPUT.println("CommitASAP not set. Need commit()");
       EEPROM.commit();
    }    
  }
   
    //Serial.println(Datetime(devicetime));
    devicetime = getNTPtime();
    // check if rtc present
    if (devicetime == 0) {
       SERIAL_OUTPUT.println("Failed to get time from network time server.");
    }    

    DateTime now = DateTime(F(__DATE__), F(__TIME__));
    SERIAL_OUTPUT.println("adjust time!");
    rtc.begin();
    //rtc.adjust(now);
    //rtc.adjust(DateTime(devicetime));
    rtc.adjust(devicetime);
    
    now = rtc.now();
    SERIAL_OUTPUT.print(now.year(), DEC);SERIAL_OUTPUT.print('/');SERIAL_OUTPUT.print(now.month(), DEC);SERIAL_OUTPUT.print('/');SERIAL_OUTPUT.print(now.day(), DEC);
    SERIAL_OUTPUT.print(" ");
    SERIAL_OUTPUT.print(now.hour(), DEC);SERIAL_OUTPUT.print(':');SERIAL_OUTPUT.print(now.minute(), DEC);SERIAL_OUTPUT.print(':');SERIAL_OUTPUT.print(now.second(), DEC);
    SERIAL_OUTPUT.println();  
    prevmin=now.minute();
  

  // Setup the LoRaWan Credentials
  
  // Initialize dht reader 
  //dht.begin();

  // Initializy AQI sensor
  if(sensor.init())
    {
        SERIAL_OUTPUT.println("HM330X init failed!!!");
        //while(1);
    }

  // Remove for non development
  //strcpy(hassio.mqttserver,"192.168.66.47");
  //strcpy(hassio.mqttuser,"Okolje"); 
  //strcpy(hassio.mqttpass,"Okolje");
  // ---------------------------------------------
 
  String mqttname = "WiOTH";
  
  updateDelay.start(12 * 60 * 60 * 1000); // update time via ntp every 12 hrs

  //Ethernet.begin(mac);
  device.setName("WiO Davis Weather");
  device.setSoftwareVersion("1.0.0");

    // configure sensor (optional)
  analogSensorTI.setIcon("mdi:thermometer");
  analogSensorTI.setName("Temperature Inside");
  analogSensorTI.setUnitOfMeasurement("C");
  analogSensorHI.setIcon("mdi:water-percent");
  analogSensorHI.setName("Humidity Inside");
  analogSensorHI.setUnitOfMeasurement("%");  

  //analogSensorHI.setIcon("mdi:water-percent");
  analogSensorAQ25.setName("AQI 25");
  analogSensorAQ25.setUnitOfMeasurement("AQI");  
  
  analogSensorBPS.setIcon("mdi:gauge");
  analogSensorBPS.setName("Barometric Pressure");
  analogSensorBPS.setUnitOfMeasurement("hPa");  
  
  analogSensorTO.setIcon("mdi:thermometer");  
  analogSensorTO.setName("Temperature Outside");
  analogSensorTO.setUnitOfMeasurement("C");
  analogSensorHO.setIcon("mdi:water-percent");
  analogSensorHO.setName("HumidityOutside");
  analogSensorHO.setUnitOfMeasurement("%");  

  analogSensorWindSpeed.setIcon("mdi:windsock");
  analogSensorWindSpeed.setName("WindSpeed");
  analogSensorWindSpeed.setUnitOfMeasurement("m/s");  
  
  analogSensorWindDir.setIcon("mdi:compass-outline");
  analogSensorWindDir.setName("WindDirection");
  analogSensorWindDir.setUnitOfMeasurement("deg");  

  analogSensorRainClicks.setIcon("mdi:water-check");
  analogSensorRainClicks.setName("RainClicks");
  
  // mdiSunWirelessOutline 
  analogSensorUVIndex.setIcon("mdi:sun-wireless-outline");
  analogSensorUVIndex.setName("UVIndex");
  
  analogSensorSolarIndex.setIcon("mdi:sun-wireless");
  analogSensorSolarIndex.setName("SolarIndex");  

  analogSensorSolarPanel.setIcon("mdi:battery");
  analogSensorSolarPanel.setName("SolarPanel");    
  analogSensorSolarPanel.setUnitOfMeasurement("%");  

    //mqtt.begin(BROKER_ADDR);
    //mqtt.begin(hassio.mqttserver);
  //Serial.println("Connecting to mqtt: ");    
  mqtt.begin(hassio.mqttserver, hassio.mqttuser, hassio.mqttpass);
  //Serial.println("end Connecting to mqtt");    

  //TelnetStream.begin();

  if (UsbH.Init())
    SERIAL_OUTPUT.println("USB host did not start.");  

  startingproc = 1;      

  SAMCrashMonitor::begin();
  SAMCrashMonitor::disableWatchdog(); // Make sure it is turned off during init.
  SAMCrashMonitor::dump();            // Dump watchdog reset data to the console.
  
  int timeout = SAMCrashMonitor::enableWatchdog(60000);
  SERIAL_OUTPUT.print(F("Watchdog enabled for: "));
  SERIAL_OUTPUT.print(timeout);
  SERIAL_OUTPUT.println(F(" ms"));

  /*
  float temp = 0;
  float humi = 0;
  int SoilMoisture = 0;
  int Flies = 0;  
  float Barometer = 0;
  float WindSpeed = 0;
  float tempout = 0;
  float humiout = 0;
  int WindDirection = 0;
  int RainClicks = 0; // remember to reset rainclicks at midnight
  */
}
 
void loop(void)
{

    SAMCrashMonitor::iAmAlive();

    UsbH.Task();

    if ((digitalRead(WIO_KEY_C) == LOW) && (digitalRead(WIO_KEY_B) == LOW))
     {
       if  (digitalRead(WIO_KEY_A) == LOW) 
       {
          wifiManager.resetSettings();
       }
       SERIAL_OUTPUT.println("RESET proceure pressed");
       delay(1000);
       NVIC_SystemReset();
     }
    
//--------------------------------------------------------------------------------------------------------------------------------------------

  if (( UsbH.getUsbTaskState() == USB_STATE_RUNNING ) && Ftdi.isReady() ) 
  {
    uint8_t rcode;
    int bytesIn;
    char buf[64]; // was 64
    char rcvc[64];  // was 64

    if (startingproc>0) 
    {
      bytesIn=4;
      strcpy(buf,"t1\n");
      rcode = Ftdi.SndData(bytesIn, (uint8_t*)buf);
      if (rcode)
          ErrorMessage<uint8_t>(PSTR("SndData"), rcode);
      SERIAL_OUTPUT.write("Sent t1");
      delay(200);
      strcpy(buf,"f1\n");
      rcode = Ftdi.SndData(bytesIn, (uint8_t*)buf);
      if (rcode)
          ErrorMessage<uint8_t>(PSTR("SndData"), rcode);
      SERIAL_OUTPUT.write("Sent f1");
      delay(200);
      strcpy(buf,"o1\n");
      rcode = Ftdi.SndData(bytesIn, (uint8_t*)buf);
      if (rcode)
          ErrorMessage<uint8_t>(PSTR("SndData"), rcode);
      SERIAL_OUTPUT.write("Sent o1");
      delay(200);
      strcpy(buf,"m1\n");
      rcode = Ftdi.SndData(bytesIn, (uint8_t*)buf);
      if (rcode)
          ErrorMessage<uint8_t>(PSTR("SndData"), rcode);
      SERIAL_OUTPUT.write("Sent m1");
      delay(200);
      startingproc=0;
    }

     /* reading USB serial */
    /* buffer size must be greater or equal to max.packet size */
    /* it it set to 64 (largest possible max.packet size) here, can be tuned down
       for particular endpoint */
    uint16_t rcvd = sizeof(buf);
    rcvc[0] = '\0';
    rcode = Ftdi.RcvData(&rcvd, (uint8_t *)buf);
    if (rcode && rcode != USB_ERRORFLOW)
      ErrorMessage<uint8_t>(PSTR("Ret"), rcode);
    else 
    {
      // The device reserves the first two bytes of data
      //   to contain the current values of the modem and line status registers.
      if (rcvd > 2) {
        SERIAL_OUTPUT.write(buf+2, rcvd-2);
        //  here parse the rcvd data ---------------------------
        //strcpy(rcvc,buf+2);
        memcpy(&rcvc,buf+2,rcvd-2);
        // split rcvc based on \n as multiple nisec can be in the buffer 


        char *array[64];
        int ai = 0;
        array[ai] = strtok(rcvc," ");
    
        while(array[ai] != NULL) 
        {
           array[++ai] = strtok(NULL," ");
        } 
    
        if (array[0] != NULL) 
        {
          //SERIAL_OUTPUT.print("_");
          SERIAL_OUTPUT.print(array[0][0]);
          SERIAL_OUTPUT.print(" \t");
          SERIAL_OUTPUT.print(array[2]);
          SERIAL_OUTPUT.print(" \t");
          SERIAL_OUTPUT.println(array[3]);
      
          //char func ;
          //sprintf("%c", array[0], func);

          if ((array[0][0] == 'T') && isDigit(array[1][0])) 
          {
            if (array[2] != NULL) 
              tempout = atof(array[2]);      
            if (array[3] != NULL)    
              humiout = atof(array[3]);
          }
          
          if (array[0][0] == 'B') 
          { 
              // temp = atoi(array[2]);       // Internal temperature
            if (array[2] != NULL) 
            {
              temp = atof(array[1]);            // Internal temperature
              Barometer = atof(array[2]);       // pressure in hPa
            }
          }     
          
          if ((array[0][0] == 'W') && isDigit(array[1][0]))
          {
            if (array[2] != NULL) 
              WindSpeed = atof(array[2]);       // Current Wind speed m/s
            if (array[3] != NULL)  
              WindDirection = atof(array[3]);       // Wind direction in degrees      
          }                   
          
          if ((array[0][0] == 'R') && isDigit(array[1][0]))
          {  // 
              if (array[2] != NULL) 
                 RainClicks = atof(array[2]);       // Rain Clicks
          }     

          if ((array[0][0] == 'P') && isDigit(array[1][0]))
          {  // 
              if (array[2] != NULL) 
                 Battery = atof(array[2]);       // Solar panel
          }     
        } // if (array[0] != NULL)
      
      }  // if (rcvd > 2) {
        // ----------------------------------------------------- 
      //}
    } // else
  }
    
//--------------------------------------------------------------------------------------------------------------------------------------------       
    DateTime now = DateTime(F(__DATE__), F(__TIME__));
    static uint8_t data[5] = { 0x00 };  //Use the data[] to store the values of the sensors    

    if (updateDelay.justFinished()) 
    { // 12 hour loop
        // repeat timer
        updateDelay.repeat(); // repeat
 
        // update rtc time
        devicetime = getNTPtime();
        if (devicetime == 0) {
            SERIAL_OUTPUT.println("Failed to get time from network time server.");
        }
        else {
            rtc.adjust(DateTime(devicetime));
            SERIAL_OUTPUT.println("");
            SERIAL_OUTPUT.println("rtc time updated.");
            // get and print the adjusted rtc time
            now = rtc.now();
            SERIAL_OUTPUT.print("Adjusted RTC time is: ");
            SERIAL_OUTPUT.println(now.timestamp(DateTime::TIMESTAMP_FULL));
        }
    }
    
    //temp = dht.readTemperature();
    //humi = dht.readHumidity();    
 
    now = rtc.now();
    sprintf(temps, "%g *C", temp);
    sprintf(humis, "%g %%", humi);
    sprintf(baros, "%g ", Barometer);
    //sprintf(dz_temps, "%g", temp);
    //sprintf(dz_humis, "%g", humi);    
    // sprintf( str, "%s/%s %s:%s", Day, Month, Hour, Min );
    //now.hour()
    sprintf(clocks, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    //temps = strcat(temps, " *C")
   
    //data_record(temp,humi,Flies,SoilMoisture,data);    
      
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
           //SERIAL_OUTPUT.println("HM330X read result failed!!");
           //SERIAL_OUTPUT.println("HM330X read result failed!!");
      }
      //parse_result_value(buf);
      parse_result(buf);
      //SERIAL_OUTPUT.println("");
      
      //sprintf(ppm2s, "%d ppm", ppm25);
      spr.createSprite(150, 30);
      spr.setFreeFont(&FreeSansBold12pt7b);
      //spr.drawString(ppm2s, 0, 1);
      spr.drawString(baros, 0, 1);
      spr.pushSprite(180, 140);
      spr.deleteSprite();


      SERIAL_OUTPUT.print("Temperature: ");
      SERIAL_OUTPUT.print(temps);    
      SERIAL_OUTPUT.print(" \t");
      SERIAL_OUTPUT.print("Humidity: ");
      SERIAL_OUTPUT.print(humis);
      SERIAL_OUTPUT.print(" \t");
      SERIAL_OUTPUT.print(clocks);    
      SERIAL_OUTPUT.print(" \t Barometer: ");
      SERIAL_OUTPUT.print(Barometer);    
      SERIAL_OUTPUT.print(" \t");
      SERIAL_OUTPUT.println(" ");

      analogSensorTI.setValue(temp);
      analogSensorHI.setValue(humi);
      float ppm25f = ppm25 * 1.0;    
      analogSensorAQ25.setValue(ppm25f);
      analogSensorBPS.setValue(Barometer);

      analogSensorTO.setValue(tempout);
      analogSensorHO.setValue(humiout);
      analogSensorWindSpeed.setValue(WindSpeed);
      analogSensorWindDir.setValue(WindDirection);
      analogSensorRainClicks.setValue(RainClicks);
      analogSensorUVIndex.setValue(UVIndex);
      analogSensorSolarIndex.setValue(SolarIndex);
      analogSensorSolarPanel.setValue(Battery);

      //delay(1000);
    } 
    
    if (now.minute() != prevmin )
    {
        prevmin=now.minute();
        // Send Homeassistant data

        
      //delay(300);         
    } 
    delay(800);
 // delay(1800000);
  mqtt.loop();    
}

/*
void SERCOM3_0_Handler()
{
  Serial3.IrqHandler();
}
void SERCOM3_1_Handler()
{
  Serial3.IrqHandler();
}
void SERCOM3_2_Handler()
{
  Serial3.IrqHandler();
}
void SERCOM3_3_Handler()
{
  Serial3.IrqHandler();
}
*/