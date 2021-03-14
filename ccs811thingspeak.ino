/*
  ccs811thingspeak.ino - Upload (ENS210 improved) CCS811 measurements to ThingSpeak using ESP8266
  Created by Maarten Pennings 2018 nov 02
*/
#define VERSION "v2"


/*
This sketch assumes you have
- an ESP8266 with an ENS210 and CCS811 attached to I2C (SDA/D2 and SCL/D1)
    Core library version 2.4.2
    Arduino   Board: "NodeMCU 1.0 (ESP-12E Module)"
    Arduino   CPU Frequency: "80 MHz"
- installed the I2C bus clear library (wise, since ESP8266 does bad clockstretching which CCS811 needs)
   Goto https://github.com/maarten-pennings/I2Cbus, press Download zipfile
   Click Sketch > Include Library > Add .ZIP Library...  then select downloaded zip file
- installed the ENS210 Arduino library 
   Goto https://github.com/maarten-pennings/ENS210, press Download zipfile
   Click Sketch > Include Library > Add .ZIP Library...  then select downloaded zip file
- installed the CCS811 Arduino library 
   Goto https://github.com/maarten-pennings/CCS811, press Download zipfile
   Click Sketch > Include Library > Add .ZIP Library...  then select downloaded zip file
- installed the ThingSpeak Arduino library 
   Sketch > Include Library > Manage Libraries > ThingSpeak > Install
- a ThingSpeak account
- with a channel for CCS811 measurements
- with fields for that channel
   field 1: eCO2 (ppm)
   field 2: eTVOC (ppb)
   field 3: ERRORID_STATUS
   field 4: resistance (Ω)
   field 5: Temperature (°C)
   field 6: Humidity (%RH)
   field 7: TH_status
   field 8: Error count
- used the "Channel ID" of that channel as initializer for thingspeakChannelId (see below)
- used the "Write API Key" of that channel as initializer for thingspeakWriteApiKey (see below)
*/


#include <Wire.h>        // I2C library
#include <WiFi.h> //  WiFi library
#include <ThingSpeak.h>  // ThingSpeak library
#include <ccs811.h>      // CCS811 library https://github.com/maarten-pennings/CCS811


#define LED_PIN    2    // GPIO2 == D4 == standard BLUE led available on most NodeMCU boards (LED on == D4 low)
#define led_init() pinMode(LED_PIN, OUTPUT)
#define led_on()   digitalWrite(LED_PIN, LOW)
#define led_off()  digitalWrite(LED_PIN, HIGH)
#define led_tgl()  digitalWrite(LED_PIN, (HIGH+LOW)-digitalRead(LED_PIN) );

// Wrapper for blinking 'blink' times, each blink (off+on) taking `ms` ms.
void led_blink(int blink, int ms ) {
  for( int i=0; i<2*blink; i++ ) { led_tgl(); delay(ms/2); } 
}


#if 1
  // Fill out the credentials of your local WiFi Access Point
  const char *  wifiSsid              = ""; // Your WiFi network SSID name
  const char *  wifiPassword          = ""; // Your WiFi network password
  // Fill out the credentials of your ThingSpeak channel
  unsigned long thingspeakChannelId   = ; // Your ThingSpeak Channel ID 
  const char *  thingspeakWriteApiKey = ""; // Your ThingSpeak write api key
#else
  // File that contains (my secret) credentials for WiFi and ThingSpeak
//  #include "credentials.h"
#endif


WiFiClient  client;
CCS811      ccs811(23); // nWAKE on 23


void setup() {
  // Enable LED (switch it on to show user that setup begins)
  led_init();
  led_on();

  // Enable serial
  delay(3000); // Give user some time to connect USB serial
  Serial.begin(115200);
  Serial.printf("\n\nsetup: CCS811 to ThingSpeak %s (lib v%d)\n",VERSION, CCS811_VERSION);

  // Print some version info
//  Serial.printf("setup: core=%s, sdk=%s, freq=%d\n",ESP.getCoreVersion().c_str(),ESP.getSdkVersion(),ESP.getCpuFreqMHz() );

  // I2C bus clear
//  Serial.printf("setup: I2C bus: %s\n", I2Cbus_statusstr(I2Cbus_clear(SDA,SCL)));

  // Enable I2C, e.g. for ESP8266 NodeMCU boards: VDD to 3V3, GND to GND, SDA to D2, SCL to D1, nWAKE to D3 (or GND)
  Wire.begin(); 
  Serial.printf("setup: I2C up\n");
  
  // Enable ENS210
  bool ok;
  //ok= ens210.begin();
 // Serial.printf("setup: ENS210 %s\n", ok?"up":"FAILED" );

  // Enable CCS811
  // The ESP8266 does not handle I2C clock stretch correctly.
  // See the readme for a patch in the ESP8266 si2c driver to fix this.
  // Alternatively enable the below line, but it is less robust.
  //ccs811.set_i2cdelay(50); // Needed for ESP8266 because it doesn't handle I2C clock stretch correctly
  ok= ccs811.begin();
  Serial.printf("setup: CCS811 %s\n", ok?"up":"FAILED" );

  // Start CCS811 (measure every 1 second)
  ok= ccs811.start(CCS811_MODE_60SEC);
  Serial.printf("setup: CCS811 start %s\n", ok?"up":"FAILED" );
  // Print CCS811 versions
  Serial.printf("setup: version: hw=0x%02x, boot=0x%04x, app=0x%04x\n",ccs811.hardware_version(),ccs811.bootloader_version(),ccs811.application_version());

  // Enable WiFi
  Serial.printf("setup: MAC %s\n",WiFi.macAddress().c_str());
  Serial.print("setup: WiFi '");
  Serial.print(wifiSsid);
  Serial.print("' ..");
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid, wifiPassword);
  while( WiFi.status()!=WL_CONNECTED ) {
    led_tgl();
    delay(250);
    Serial.printf(".");
  }
  Serial.printf(" up (%s)\n",WiFi.localIP().toString().c_str());

  // Enable ThingSpeak
  ThingSpeak.begin(client);
  Serial.println("setup: ThingSpeak up");

  // End of setup() - delay helps distinguishing LED flashes
  led_off();
  delay(3000);
  Serial.println("setup: done");
 
}


// Keep track of "last known correct values" for each of the ThingSpeak fields (except status)
int   Field1_eCO2     = 400;    // ppm
int   Field2_eTVOC    = 0;      // ppb
//    Field3 ERROR_ID & STATUS
float Field4_Resist   = 100000; // Ω
float Field5_T        = 25.0;   // °C
float Field6_H        = 50.0;   // %RH
//    Field7  TH status combined
int   Field8_errcount = 0;


void loop() {
  // Read ENS210
  bool pass_ok=true;
  int t_data, t_status, h_data, h_status;
  uint16_t baseline;
  float  humidity,temperature;





  // Read CCS811
  uint16_t eco2, etvoc, errstat, raw;
  humidity=readSensor(&temperature);
  ccs811.set_envdata(temperature,humidity);
  ccs811.read(&eco2,&etvoc,&errstat,&raw); 
  ccs811.get_baseline(&baseline);
  

  // Process CCS811
  if( errstat==CCS811_ERRSTAT_OK ) {
    // We have valid gas data, update "last known correct values"
    Field1_eCO2= eco2;
    Field2_eTVOC= etvoc;
    Field4_Resist= raw; // V/I
    // Print eCO2, eTVOC, and R
    Serial.printf("eco2=%dppm, ",Field1_eCO2);
    Serial.printf("etvoc=%dppb, ",Field2_eTVOC);
    Serial.printf("R=%.0fΩ, ",Field4_Resist);
   // Serial.printf("baseline=%.0, ",baseline);
    Serial.println(baseline);
    Serial.println(humidity);
    Serial.println(temperature);
  } else if( errstat==CCS811_ERRSTAT_OK_NODATA ) {
    Serial.printf("waiting for (new) data, ");
    delay(6000);
  } else if( errstat & CCS811_ERRSTAT_I2CFAIL ) { 
    Serial.printf("I2C error, ");
    // Flash led, count errors
    led_blink(2,100);
    Field8_errcount++;
  } else {
    Serial.printf("error: %s, ",ccs811.errstat_str(errstat) ); 
    // Flash led, count errors
    led_blink(1,100);
    Field8_errcount++;
  }

  // Print cumulative number of read errors
  Serial.printf("errcount=%d, ", Field8_errcount); 
  
  // When was the last ThingSpeak update? We want them 60s apart
  static unsigned long prev;
  unsigned long now = millis();
  if( now-prev>60*1000 ) {
    if (WiFi.status()!=WL_CONNECTED){
    WiFi.reconnect();
    led_tgl();
    delay(250);
    Serial.printf("Wifi reconnect.\n");
 }
    // Prepare ThingSpeak package
    ThingSpeak.setField(1,Field1_eCO2);
    ThingSpeak.setField(2,Field2_eTVOC);
    ThingSpeak.setField(3,baseline); 
    ThingSpeak.setField(4,temperature);
    ThingSpeak.setField(5,humidity);
   // ThingSpeak.setField(6,Field6_H);
    //ThingSpeak.setField(7,t_status*10 + h_status); // Coded in two decimals
    //ThingSpeak.setField(8,Field8_errcount);
  
    // Send to ThingSpeak 
    led_on();
    int http= ThingSpeak.writeFields(thingspeakChannelId, thingspeakWriteApiKey);  
    led_off();
  
    // ThingSpeak upload feedback
    if( http!=200 ) led_blink(20,50); // Flash to show upload failure
    Serial.printf("wifi=%d, http=%d, ",WiFi.status(),http);
    
    // Record upload time
    if( http==200 ) { prev= now= millis(); Field8_errcount= 0; }
  }
  

  
  // End of the iteration
  Serial.printf("%lds\n", (now-prev+500)/1000);

  // Wait (match the CCS811 measurement mode, so that T/H is updated)
  delay(1000); 
}

float readSensor(float* temperature)
{
  //holds 2 bytes of data from I2C Line
  uint8_t Byte[4];

  //holds the total contents of the temp register
  uint16_t temp;

  //holds the total contents of the humidity register
  uint16_t humidity;
  
  //Point to device 0x40 (Address for HDC1080)
  Wire.beginTransmission(0x40);
  //Point to register 0x00 (Temperature Register)
  Wire.write(0x00);
  //Relinquish master control of I2C line
  //pointing to the temp register triggers a conversion
  Wire.endTransmission();
  
  //delay to allow for sufficient conversion time
  delay(20);
  
  //Request four bytes from registers
  Wire.requestFrom(0x40, 4);

  delay(1);
  
  //If the 4 bytes were returned sucessfully
  if (4 <= Wire.available())
  {
    //take reading
    //Byte[0] holds upper byte of temp reading
    Byte[0] = Wire.read();
    //Byte[1] holds lower byte of temp reading
    Byte[1] = Wire.read();
    
    //Byte[3] holds upper byte of humidity reading
    Byte[3] = Wire.read();
    //Byte[4] holds lower byte of humidity reading
    Byte[4] = Wire.read();

    //Combine the two bytes to make one 16 bit int
    temp = (((unsigned int)Byte[0] <<8 | Byte[1]));

    //Temp(C) = reading/(2^16)*165(C) - 40(C)
    *temperature = (double)(temp)/(65536)*165-40;

   //Combine the two bytes to make one 16 bit int
    humidity = (((unsigned int)Byte[3] <<8 | Byte[4]));

    //Humidity(%) = reading/(2^16)*100%
    return (double)(humidity)/(65536)*100;
  }
}

// Field 3 ERROR_ID & STATUS
//   C:\Users\xxx\Documents\Arduino\libraries\CCS811\src\ccs811.h
//   CCS811_ERRSTAT_ERROR               0x0001 // There is an error, the ERROR_ID register (0xE0) contains the error source
//   CCS811_ERRSTAT_I2CFAIL             0x0002 // Bit flag added by software: I2C transaction error
//                                      0x0004 // Bit flag added by software: TH-pass failed
//   CCS811_ERRSTAT_DATA_READY          0x0008 // A new data sample is ready in ALG_RESULT_DATA
//   CCS811_ERRSTAT_APP_VALID           0x0010 // Valid application firmware loaded
//   CCS811_ERRSTAT_FW_MODE             0x0080 // Firmware is in application mode (not boot mode)
//   CCS811_ERRSTAT_WRITE_REG_INVALID   0x0100 // The CCS811 received an I²C write request addressed to this station but with invalid register address ID
//   CCS811_ERRSTAT_READ_REG_INVALID    0x0200 // The CCS811 received an I²C read request to a mailbox ID that is invalid
//   CCS811_ERRSTAT_MEASMODE_INVALID    0x0400 // The CCS811 received an I²C request to write an unsupported mode to MEAS_MODE
//   CCS811_ERRSTAT_MAX_RESISTANCE      0x0800 // The sensor resistance measurement has reached or exceeded the maximum range
//   CCS811_ERRSTAT_HEATER_FAULT        0x1000 // The heater current in the CCS811 is not in range
//   CCS811_ERRSTAT_HEATER_SUPPLY       0x2000 // The heater voltage is not being applied correctly
// Field 7: TH_status
//   T status * 10 + H status
//   C:\Users\xxx\Documents\Arduino\libraries\ENS210\src\ens210.h
//   ENS210_STATUS_I2CERROR    4 // There was an I2C communication error, `read`ing the value.
//   ENS210_STATUS_CRCERROR    3 // The value was read, but the CRC over the payload (valid and data) does not match.
//   ENS210_STATUS_INVALID     2 // The value was read, the CRC matches, but the data is invalid (e.g. the measurement was not yet finished).
//   ENS210_STATUS_OK          1 // The value was read, the CRC matches, and data is valid.
// Serial: http=
//   https://en.wikipedia.org/wiki/List_of_HTTP_status_codes 
//   200 OK
//  -301 Moved Permanently
//  -303 See Other
//  -304 Not Modified 
// Serial: wifi=
//   C:\Users\xxx\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.4.2\libraries\ESP8266WiFi\src\include\wl_definitions.h
//   WL_IDLE_STATUS      = 0,
//   WL_NO_SSID_AVAIL    = 1,
//   WL_SCAN_COMPLETED   = 2,
//   WL_CONNECTED        = 3,
//   WL_CONNECT_FAILED   = 4,
//   WL_CONNECTION_LOST  = 5,
//   WL_DISCONNECTED     = 6
