/**
   BasicHTTPClient.ino

    Created on: 24.05.2015

*/

#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#include <ESP8266HTTPClient.h>

#define USE_SERIAL Serial

ESP8266WiFiMulti WiFiMulti;




void setup() {

  USE_SERIAL.begin(9600);
  // USE_SERIAL.setDebugOutput(true);
USE_SERIAL.println("on");
  USE_SERIAL.println();
  USE_SERIAL.println();
  USE_SERIAL.println();
USE_SERIAL.println("on  ");
  for (uint8_t t = 4; t > 0; t--) {
    USE_SERIAL.printf("[SETUP] WAIT %d...\n", t);
    USE_SERIAL.flush();
    delay(1000);
    
  }

  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP("Wlan", "P@s$w0RbOk");

}

int endoffile=0;
int linenumber=0;

void loop() {

  
if (USE_SERIAL.available()) {
    String c = USE_SERIAL.readString();
    if(c.indexOf("Start")>=0){
      c="0";
      linenumber=0;
      endoffile=0;
    while(endoffile!=1){
      getline(linenumber);
      USE_SERIAL.print("ok");
      int next=0;
      while(next==0 && endoffile!=1){
        if (USE_SERIAL.available()) {
           c = USE_SERIAL.readString();
           if(c.indexOf("Next")>=0){
            next=1;
           }
           if(c.indexOf("Start")>=0){
            next=1;
            linenumber=-1;
           }
        }
      }
      linenumber++;
    }

    }else{
      USE_SERIAL.println("Not a start signal");
    }
  
  
}
}

void getline(int linenumber){
  

  if ((WiFiMulti.run() == WL_CONNECTED)) {

    HTTPClient http;
    String link="http://thecloud.dx.am/apps/CNC/line.php?line=";
    link+=linenumber;
    http.begin(link); 
    int httpCode = http.GET();

    if (httpCode > 0) {
      if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        if(payload.indexOf("end")>=0){
        endoffile=1;
        }
        USE_SERIAL.println(payload);
      }
    } else {
      linenumber-1;
      USE_SERIAL.println("error b2");
    }

    http.end();
  }
  
  }


