    #include <NeoSWSerial.h>
    #include <AltSoftSerial.h>
    #include <SoftwareSerial.h>
    int wifipin=17;
    
    AltSoftSerial translator;
    
    NeoSWSerial ss( 7, 6 );
    SoftwareSerial esp(15, 16);
    volatile int r = 0;
    char cr;
    
    static void handleRxChar(char c )
    {
      cr=c;
      r=1;
    }


    bool start=1;
    bool endoffile=0;
    bool done=0;

    
    void setup()
    {
      pinMode(wifipin,INPUT_PULLUP);
      ss.attachInterrupt( handleRxChar );
      ss.begin(9600);
      
      esp.begin(38400);
      
      Serial.begin(9600);
      translator.begin(9600);
    }

  void loop() {
    char c;
    if (Serial.available()) {
      c = Serial.read();
      translator.print(c);
      Serial.print(c);
    }
      if (translator.available()) {
      c = translator.read();
      Serial.print(c);
    }
    if(digitalRead(wifipin)==1){
      
      if (r) {
        Serial.print(cr);
        translator.print(cr);
        r=0;
        Serial.println("char recieved BT");
      }
    }else{
      Serial.println("Wifi Mode");
      String data="";
      if(endoffile==0){
        esp.print("Start");
        Serial.println("Start Signal sent");
        while(done==0){ //start getting the page
          if (esp.available()) {
            data+=esp.read();
            Serial.println("char recieved");
          }
          if(data.indexOf("ok")>=0) done=1;//got the first line OK
          
        }
        Serial.print(data);//display first line
        data="";
        
        while(endoffile==0){ //read line by line till the end
          done=0;
          esp.print("Next");
          Serial.print(" Next signal sent ");
          while(done==0){ //start getting the page
          if (esp.available()) {
            data+=esp.read();
            Serial.println("char recieved");
          }
          if(data.indexOf("ok")>=0) done=1;//got the first line OK
          }
          Serial.print(data);//display the line
          if(data.indexOf("end")>=0) endoffile=1;
          data="";
          }
        }
  
    }
 }
