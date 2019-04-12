#include "Adafruit_FONA.h"
#include <Wire.h>
#include "Adafruit_LSM303.h"
#include "Adafruit_Sensor.h"



#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4
#define TRACKER_ID 1

// this is a large buffer for replies
char replybuffer[255];

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
//HardwareSerial *fonaSerial = &Serial;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

//Adafruit_LSM303 lsm;

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

uint8_t type;

uint16_t readVolt();

void getConnected();

void getGPS();

void post();

char *boolstring( _Bool b );

void setup() {
    //while (!Serial);

  //Serial.begin(115200);
  //Serial.println(F("Initializing....(May take 3 seconds)"));
  //lsm.begin();
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  type = fona.type();
  Serial.println(F("FONA is OK"));
  //Serial.print(F("Found "));
  /*switch (type) {
    case FONA800L:
      //Serial.println(F("FONA 800L")); break;
    case FONA800H:
      //Serial.println(F("FONA 800H")); break;
    case FONA808_V1:
      //Serial.println(F("FONA 808 (v1)")); break;
    case FONA808_V2:
      //Serial.println(F("FONA 808 (v2)")); break;
    case FONA3G_A:
      //Serial.println(F("FONA 3G (American)")); break;
    case FONA3G_E:
      //Serial.println(F("FONA 3G (European)")); break;
    default: 
      //Serial.println(F("???")); break;
  }*/
  
  // Print module IMEI number.
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    //Serial.print("Module IMEI: "); Serial.println(imei);
  }

  // turn GPS on
        while (!fona.enableGPS(true))
          Serial.println(F("Failed to turn on"));

  fona.setGPRSNetworkSettings(F("hologram"));
  getConnected();
}

void loop() {
  uint16_t volt;
  float latitude, longitude, speed_kph, heading, altitude;
  int x, accelx, accely, accelz, con;
  x = 1000;
  delay(x);
  con = checkConnected();
  delay(100);
  volt = readVolt();
  delay(100);
  getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
  delay(100);
  //lsm.read();
  post(latitude, longitude, speed_kph, volt, accelx, accely, accelz, con);
  //post(latitude, longitude, speed_kph, volt, (int)lsm.accelData.x, (int)lsm.accelData.y, (int)lsm.accelData.z);
}

void getConnected(){
  boolean networked = false;
  
  while(!networked){
    uint8_t n = fona.getNetworkStatus();
    if((n == 1) || (n==5)){
      networked = true;
    }
    else{
      networked = false;
    }
  }
 delay(1000);
 fona.enableGPRS(true);
 delay (2000);
}

int checkConnected(){
  boolean networked = false;
  int con = 0;
  while(!networked){
    uint8_t n = fona.getNetworkStatus();
    if((n == 1) || (n==5)){
      networked = true;
    }
    else{
      networked = false;
      con++;
    }
    delay(1300);
  }
 
 delay(50);
 fona.enableGPRS(true);
 delay(100);
}

uint16_t readVolt() {
  uint16_t vbat;

        if (! fona.getBattPercent(&vbat)) {
          return -1;
        } else {
          return vbat;
        }
}

void getGPS(float *lat, float *lon, float *speed_kph, float *heading, float *altitude){
int count = 0;
boolean gps_success = false;
boolean fix = false;
  while(count<10){
        int8_t stat;
        // check GPS fix
        stat = fona.GPSstatus();
        if (stat == 0){
          count = 50;
          fix = false;
        }
        else if (stat == 1){
          fix = false;
        }
        else if (stat == 2) { 
          fix = false;
        }
        else if (stat == 3){
          fix = true;
          count = 50;
        }
        count++;
  }
  if(fix){
    gps_success = fona.getGPS(lat, lon, speed_kph, heading, altitude);
  }
  if(gps_success){
    return;
  }
  else{
      if (fona.getNetworkStatus() == 1 || fona.getNetworkStatus() == 5) {
        boolean gsmloc_success = fona.getGSMLoc(lat, lon);
      }
   }
}

void post(float lat, float lon, float speed_kph, uint16_t volt, int accelx, int accely, int accelz, int con){
   // Post data to website
        uint16_t statuscode;
        int16_t length;
        char url[80] = "http://dmgilmour.herokuapp.com/data";
        char data[120], latS[12], lonS[12];
        String latT, lonT, voltT, idT, movingT, temp, conT;
        boolean moving = (speed_kph > 6 || accelx > 20000 || accelx < -20000 || accely > 20000 || accely < -20000);
        int id = TRACKER_ID;
        
        
        if (lon > 190){
          return;
        }
        movingT = boolstring(moving);
        voltT = String(volt);
        
        dtostrf(lat, 4, 7, latS);
        dtostrf(lon, 4, 7, lonS);
        latT = String(latS);
        lonT = String(lonS);
        idT = String(id);
        conT = String(con);
        //Serial.print("latT: ");Serial.println(latT);
        //Serial.print("lonT: ");Serial.println(lonT);
        //Serial.print("idT: ");Serial.println(idT);
        //Serial.print("movingT: ");Serial.println(movingT);
        //Serial.print("voltT: ");Serial.println(voltT);
        temp = "{\"lat\":\""+lonT+"\",\"lon\":\""+latT+"\",\"id\":\""+idT+"\",\"moving\":\""+movingT+"\",\"battery\":\""+voltT+"\",\"con\":\""+conT+"\"}"; 
        //temp = "{\"lat\":\""+lonT+"\",\"lon\":\""+latT+"\",\"id\":\""+idT+"\",\"moving\":\""+movingT+"\",\"battery\":\""+voltT+"\"}";
        //Serial.println(url);
        //Serial.println(temp);
        temp.toCharArray(data, 80);
        Serial.print("data: ");Serial.println(data);

        //Serial.println(F("****"));
        if (!fona.HTTP_POST_start(url, F("application/json"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *)&length)) {
          //Serial.println("Failed!");
          return;
        }
        while (length > 0) {
          while (fona.available()) {
            char c = fona.read();
            

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
            loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
            UDR0 = c;
#else
            Serial.write(c);
#endif

            length--;
            if (! length) break;
          }
        }
        //Serial.println(F("\n****"));
        fona.HTTP_POST_end();
}

char *boolstring( _Bool b ) { return b ? "1" : "0"; }
