#include <SPI.h>              // include libraries
#include "LoRa.h"
#include <Wire.h>
#include <DFRobot_QMC5883.h>
#include <MS5611.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <L3G4200D.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

double realTemperature = 0;
long realPressure = 0;

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 115200;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

int16_t ax;
int16_t ay;
int16_t az;
int16_t mx;
int16_t my;
int16_t mz;
int16_t gx;
int16_t gy;
int16_t gz;
double latit = 0.;
double longit = 0.;
double altitud = 0.;

MS5611 ms5611;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
DFRobot_QMC5883 compass(&Wire, 0x0D);
L3G4200D gyro;

const int csPin = 10;          // LoRa radio chip select
const int resetPin = 6;       // LoRa radio reset
const int irqPin = 2;         // change for your board; must be a hardware interrupt pin
//Packet is 35 simbolov, preobrazovat v massiv simvolov i na otpravku
/*
String Pico1 = "1...............................";
String Pico2 = "2...............................";
String Pico3 = "3...............................";
String Pico4 = "4...............................";
*/
const int PicoNumber = 4;
uint8_t packetint[4][39] = {{'1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1'},
                            {'2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2'},
                            {'3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3'},
                            {'4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4'}};
long long REFRESH_INTERVAL = 4997; // ms
unsigned long REFRESH_INTERVAL_1SEC = 1000;
unsigned long lastRefreshTime = 0;
unsigned long lastRefreshTime1sec = 0;
unsigned long nsec = 0;
unsigned long currtime = 0;
unsigned long currirqtime = 0;
unsigned long prevtime = 0;
unsigned long previrqtime = 0;
long delays = 0;
unsigned long START_INTERVAL = 0;
uint8_t isReceived = 0;
uint8_t firstreceive = 1;
uint8_t n = 0;

uint8_t firstbebe = 0;
uint8_t firstskip = 1;
uint8_t tempp[4];
uint8_t latt[4];
uint8_t lonn[4];
uint8_t altit[4];
uint8_t picostatuses[4] = {0, 0, 0, 0};
uint8_t startflag = 0;
void setup() {
  pinMode(5, INPUT);  
  Serial.begin(115200);
  
  while (!Serial);
  /*
  const int csPin = 10;          // LoRa radio chip select
  const int resetPin = 6;       // LoRa radio reset
  const int irqPin = 2; 
  */
  Serial.println("LoRa Receiver Callback");
  //LoRa.setPins(csPin, resetPin, irqPin);
  if (!LoRa.begin(436E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  digitalWrite(9, LOW);
  // Uncomment the next line to disable the default AGC and set LNA gain, values between 1 - 6 are supported
  // LoRa.setGain(6);
  LoRa.setSpreadingFactor(9);  
  //LoRa.setTxPower(int level, int outputPin)
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  //LoRa.setPreambleLength(6);
  LoRa.onReceive(onReceive);
  //xTaskCreate(ManagerTask,"managertask",128,NULL,2,&TaskHandle_Manager);
  //xTaskCreate(Task1,"task1",512,NULL,1,&TaskHandle_1);
  
  ms5611.begin();
  delay(1000);
  accel.begin();
  compass.begin();
  gyro.initialize();
  gyro.setFullScale(2000);
  accel.setRange(ADXL345_RANGE_16_G);
  
  /*
  if(!accel.begin())
  {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
  }
  while (!compass.begin())
  {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
  }
  //gyro.initialize();
  //gyro.setFullScale(2000);
  accel.setRange(ADXL345_RANGE_16_G);
  */
  LoRa.receive();
  Serial.println("LoRa init succeeded.");

}
void loop() {   
  if(millis() - lastRefreshTime >= REFRESH_INTERVAL){
    lastRefreshTime += REFRESH_INTERVAL;
    if (isReceived) {
      prevtime = currtime;
      if (firstbebe){
        firstbebe = 0;
        //lastRefreshTime += 1000;
        //prevtime += 1000;
        firstskip = 1;
      }
    }
    currtime = millis();
    Serial.print("Main currtime "); Serial.println(currtime);
    Serial.print("Main prevtime "); Serial.println(prevtime);
    ///
    Serial.println(firstreceive);
    Serial.println(firstskip);
    Serial.println(n);
    realTemperature = ms5611.readTemperature();
    realPressure = ms5611.readPressure();
    sensors_event_t event; 
    accel.getEvent(&event);
    ax = accel.getX();
    ay = accel.getY();
    az = accel.getZ();
    gyro.getAngularVelocity(&gx, &gy, &gz); 
    float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
    compass.setDeclinationAngle(declinationAngle);
    sVector_t mag = compass.readRaw();
    compass.readRaw();
    mx = mag.XAxis;
    my = mag.YAxis;
    mz = mag.ZAxis;
    Serial.print("Pressure: ");
    Serial.println(realPressure);
    Serial.print("Temperature: ");
    Serial.println(realTemperature);
  
    Serial.print("AX: ");
    Serial.println(ax*1/256.0f);
    Serial.print("AY: ");
    Serial.println(ay*1/256.0f);
    Serial.print("AZ: ");
    Serial.println(az*1/256.0f);
  
    Serial.print("GX: ");
    Serial.println(gx*0.07);
    Serial.print("GY: ");
    Serial.println(gy*0.07);
    Serial.print("GZ: ");
    Serial.println(gz*0.07);

    Serial.print("MX: ");
    Serial.println(mx);
    Serial.print("MY: ");
    Serial.println(my);
    Serial.print("MZ: ");
    Serial.println(mz);
    floatToByte(tempp, realTemperature);
    floatToByte(latt, latit);
    floatToByte(lonn, longit);
    floatToByte(altit, altitud);
    uint8_t packetintcurr[39] = {
      (uint8_t)(PicoNumber + '0'), //'4',
      tempp[0], tempp[1],
      tempp[2], tempp[3],
      latt[0], latt[1],
      latt[2], latt[3],
      lonn[0], lonn[1],
      lonn[2], lonn[3],
      altit[0], altit[1],
      altit[2], altit[3],
      (ax >> 8) & 0xFF, ax, (ay >> 8) & 0xFF, ay, (az >> 8) & 0xFF, az,
      (gx >> 8) & 0xFF, gx, (gy  >> 8) & 0xFF, gy, (gz >> 8) & 0xFF, gz,
      (mx >> 8) & 0xFF, mx, (my >> 8) & 0xFF, my, (mz >> 8) & 0xFF, mz,
      (uint8_t)((realPressure >> 24) & 0xFF),
      (uint8_t)((realPressure >> 16) & 0xFF),
      (uint8_t)((realPressure >> 8) & 0xFF),
      (uint8_t)((realPressure >> 0) & 0xFF)
    };
    memcpy(&packetint[0], packetintcurr, sizeof(packetintcurr));
    if (n < 3) n += 1;
    picostatuses[0] = 1;
    if (!firstreceive){
      if (!firstskip){
        if (n!=0) {
          uint8_t sendin[n*39];
          uint8_t j = 0;
          for (uint8_t i = 0; i < 4; i++){
            if (picostatuses[i] == 1){
              memcpy(&sendin[39*j], packetint[i], sizeof(packetint[i]));
              picostatuses[i] = 0;
              j += 1;
            } 
          }
          LoRa.beginPacket();                
          LoRa.write(sendin, 39*n);              
          LoRa.endPacket();
        }
      } else {
        firstskip = 0;
      }
    }
    n = 0;
    isReceived = 0;
    LoRa.receive();
    
  }
  while (Serial.available() > 0){
    if (gps.encode(Serial.read())){
      latit = gps.location.lat();
      longit = gps.location.lng();
      altitud = gps.altitude.meters();
      Serial.print("GPS DATA: ");
      Serial.println(latit, 6);
      Serial.println(longit, 6);
      Serial.println(altitud);
    } 
  }  
  /*
  if(millis() - lastRefreshTime >= REFRESH_INTERVAL){
    lastRefreshTime += REFRESH_INTERVAL;
    if (isReceived) {
      prevtime = currtime;
      if (firstbebe){
        firstbebe = 0;
        prevtime += 1000;
        firstskip = 1;
      }
    }
    currtime = millis();
    Serial.print("Main currtime "); Serial.println(currtime);
    Serial.print("Main prevtime "); Serial.println(prevtime);
    realTemperature = ms5611.readTemperature();
    realPressure = ms5611.readPressure();
    sensors_event_t event; 
    accel.getEvent(&event);
    ax = accel.getX();
    ay = accel.getY();
    az = accel.getZ();
    gyro.getAngularVelocity(&gx, &gy, &gz); 
    float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
    compass.setDeclinationAngle(declinationAngle);
    sVector_t mag = compass.readRaw();
    compass.readRaw();
    mx = mag.XAxis;
    my = mag.YAxis;
    mz = mag.ZAxis;
    Serial.print("Pressure: ");
    Serial.println(realPressure);
    Serial.print("Temperature: ");
    Serial.println(realTemperature);
  
    Serial.print("AX: ");
    Serial.println(ax*1/256.0f);
    Serial.print("AY: ");
    Serial.println(ay*1/256.0f);
    Serial.print("AZ: ");
    Serial.println(az*1/256.0f);
  
    Serial.print("GX: ");
    Serial.println(gx*0.07);
    Serial.print("GY: ");
    Serial.println(gy*0.07);
    Serial.print("GZ: ");
    Serial.println(gz*0.07);

    Serial.print("MX: ");
    Serial.println(mx);
    Serial.print("MY: ");
    Serial.println(my);
    Serial.print("MZ: ");
    Serial.println(mz);
    floatToByte(tempp, realTemperature);
    floatToByte(latt, latit);
    floatToByte(lonn, longit);
    
    uint8_t packetintcurr[35] = {
      2,
      (ax >> 8) & 0xFF, ax, (ay >> 8) & 0xFF, ay, (az >> 8) & 0xFF, az,
      (gx >> 8) & 0xFF, gx, (gy  >> 8) & 0xFF, gy, (gz >> 8) & 0xFF, gz,
      (mx >> 8) & 0xFF, mx, (my >> 8) & 0xFF, my, (mz >> 8) & 0xFF, mz,
      tempp[0], tempp[1],
      tempp[2], tempp[3],
      (uint8_t)((realPressure >> 24) & 0xFF),
      (uint8_t)((realPressure >> 16) & 0xFF),
      (uint8_t)((realPressure >> 8) & 0xFF),
      (uint8_t)((realPressure >> 0) & 0xFF),
      latt[0], latt[1],
      latt[2], latt[3],
      lonn[0], lonn[1],
      lonn[2], lonn[3],
    };
    memcpy(&packetint[0], packetintcurr, sizeof(packetintcurr));
    n += 1;
    ///
    n = 1;
    if (!firstreceive){
      if (!firstskip){
        Serial.print("Check n:"); Serial.println(n);
        if (n!=0) {
          uint8_t sendin[n*35];
          for (int i = 0; i < n; i++){
            memcpy(&sendin[35*i], packetint[i], sizeof(packetint[i]));
          }
          Serial.print("Formated: ");
          for (int i = 0; i < 35; i++){
            Serial.print(packetintcurr[i]);
          }
          Serial.println();
          Serial.print("To :");
          for (int i = 0; i < 35*n; i++){
            Serial.print(sendin[i]);
          }
          Serial.println();
          LoRa.beginPacket();        
          LoRa.write(packetintcurr, 35);     
          //LoRa.write(sendin, 35*n);              
          LoRa.endPacket();
        }
      } else {
        firstskip = 0;
      }
    }
    n = 0;
    isReceived = 0;
    LoRa.receive();
    
  }
  while (Serial.available() > 0){
    Serial.println("Smth");
    if (gps.encode(Serial.read())){
      latit = gps.location.lat();
      longit = gps.location.lng();
      Serial.println(latit, 6);
      Serial.println(longit, 6);
    } else {
      Serial.println("No gpses?😁");
    }
  }
  */
}
void onReceive(int packetSize) {  
  if (packetSize == 0) return;   
  String incoming = "";                 // payload of packet
  while (LoRa.available()) {            // can't use readString() in callback, so
    incoming += (char)LoRa.read();      // add bytes one by one
  }
  Serial.println(incoming);
  if (incoming[0] == 0 && digitalRead(5) == HIGH){
    startflag = 1;
  }
  if (!startflag) return;
  uint8_t obrabotkaflag = 0;
  if (PicoNumber == 1){
      if (incoming[0] == '1'){
        for(int i = 0; i < 39; i++){
          packetint[0][i] = (uint8_t)(incoming[i]);
        }
      Serial.print("Got first");
      obrabotkaflag = 1;
      if (n < 3) n += 1;
      picostatuses[0] = 1;
    }
    if (incoming[0] == '2'){
      for(int i = 0; i < 39; i++){
        packetint[1][i] = (uint8_t)(incoming[i]);
      }
      Serial.println("Got second");
      obrabotkaflag = 1;
      if (n < 3) n += 1;
      picostatuses[1] = 1;
    }
    if (incoming[0] == '4'){
      for(int i = 0; i < 39; i++){
        packetint[2][i] = (uint8_t)(incoming[i]);
      }
      Serial.println("Got fourth");
      obrabotkaflag = 1;
      if (n < 3) n += 1;
      picostatuses[2] = 1;
    }
    if (incoming[0] == '3'){
      for(int i = 0; i < 39; i++){
        packetint[3][i] = (uint8_t)(incoming[i]);
      }
      Serial.println("Got fourth");
      obrabotkaflag = 1;
      picostatuses[3] = 1;
      if (n < 3) n += 1;
      picostatuses[3] = 1;
    }
  }
  if (PicoNumber == 2){
      if (incoming[0] == '2'){
        for(int i = 0; i < 39; i++){
          packetint[0][i] = (uint8_t)(incoming[i]);
        }
      Serial.print("Got second");
      obrabotkaflag = 1;
      if (n < 3) n += 1;
      picostatuses[0] = 1;
    }
    if (incoming[0] == '1'){
      for(int i = 0; i < 39; i++){
        packetint[1][i] = (uint8_t)(incoming[i]);
      }
      Serial.println("Got first");
      obrabotkaflag = 1;
      if (n < 3) n += 1;
      picostatuses[1] = 1;
    }
    if (incoming[0] == '3'){
      for(int i = 0; i < 39; i++){
        packetint[2][i] = (uint8_t)(incoming[i]);
      }
      Serial.println("Got third");
      obrabotkaflag = 1;
      if (n < 3) n += 1;
      picostatuses[2] = 1;
    }
    if (incoming[0] == '4'){
      for(int i = 0; i < 39; i++){
        packetint[3][i] = (uint8_t)(incoming[i]);
      }
      Serial.println("Got fourth");
      obrabotkaflag = 1;
      picostatuses[3] = 1;
      if (n < 3) n += 1;
      picostatuses[3] = 1;
    }
  }
  if (PicoNumber == 3){
      if (incoming[0] == '3'){
        for(int i = 0; i < 39; i++){
          packetint[0][i] = (uint8_t)(incoming[i]);
        }
      Serial.print("Got first");
      obrabotkaflag = 1;
      if (n < 3) n += 1;
      picostatuses[0] = 1;
    }
    if (incoming[0] == '2'){
      for(int i = 0; i < 39; i++){
        packetint[1][i] = (uint8_t)(incoming[i]);
      }
      Serial.println("Got second");
      obrabotkaflag = 1;
      if (n < 3) n += 1;
      picostatuses[1] = 1;
    }
    if (incoming[0] == '4'){
      for(int i = 0; i < 39; i++){
        packetint[2][i] = (uint8_t)(incoming[i]);
      }
      Serial.println("Got fourth");
      obrabotkaflag = 1;
      if (n < 3) n += 1;
      picostatuses[2] = 1;
    }
    if (incoming[0] == '1'){
      for(int i = 0; i < 39; i++){
        packetint[3][i] = (uint8_t)(incoming[i]);
      }
      Serial.println("Got first");
      obrabotkaflag = 1;
      picostatuses[3] = 1;
      if (n < 3) n += 1;
      picostatuses[3] = 1;
    }
  }
  if (PicoNumber == 4){
      if (incoming[0] == '4'){
        for(int i = 0; i < 39; i++){
          packetint[0][i] = (uint8_t)(incoming[i]);
        }
      Serial.print("Got fourth");
      obrabotkaflag = 1;
      if (n < 3) n += 1;
      picostatuses[0] = 1;
    }
    if (incoming[0] == '3'){
      for(int i = 0; i < 39; i++){
        packetint[1][i] = (uint8_t)(incoming[i]);
      }
      Serial.println("Got third");
      obrabotkaflag = 1;
      if (n < 3) n += 1;
      picostatuses[1] = 1;
    }
    if (incoming[0] == '1'){
      for(int i = 0; i < 39; i++){
        packetint[2][i] = (uint8_t)(incoming[i]);
      }
      Serial.println("Got first");
      obrabotkaflag = 1;
      if (n < 3) n += 1;
      picostatuses[2] = 1;
    }
    if (incoming[0] == '2'){
      for(int i = 0; i < 39; i++){
        packetint[3][i] = (uint8_t)(incoming[i]);
      }
      Serial.println("Got second");
      obrabotkaflag = 1;
      picostatuses[3] = 1;
      if (n < 3) n += 1;
      picostatuses[3] = 1;
    }
  }
  if (incoming[0] != 0) obrabotkaflag = 1;
  Serial.println("--------------");
  if (!obrabotkaflag){
    isReceived = 1;
    previrqtime = currirqtime;
    currirqtime = millis();
  ///
    if (firstreceive){
      firstreceive = 0;
      //lastRefreshTime += 1000;
      lastRefreshTime = millis() - 4997 - (PicoNumber*1000 + (1000-336));
      prevtime = lastRefreshTime;
      currtime = lastRefreshTime;// + 4997;
      //prevtime += 1000;
      firstbebe = 1;
    } else {
      Serial.print("currtime "); Serial.println(currtime);
      Serial.print("prevtime "); Serial.println(prevtime);
      Serial.print("currirqtime "); Serial.println(currirqtime);
      Serial.print("previrqtime "); Serial.println(previrqtime);
      delays = ((currtime-prevtime)%5000) - ((currirqtime-previrqtime)%5000);
      REFRESH_INTERVAL -= delays;
      Serial.print("Delay "); Serial.println(delays);
    }
  }
  obrabotkaflag = 0;
  Serial.println("--------------");
  /*
  String incoming = "";                 // payload of packet
  while (LoRa.available()) {            // can't use readString() in callback, so
    incoming += (char)LoRa.read();      // add bytes one by one
  }
  */
  /*
  if (packetSize == 0) return;   
  Serial.println("--------------");
  isReceived = 1;
  previrqtime = currirqtime;
  currirqtime = millis();
  ///
  if (firstreceive){
    firstreceive = 0;
    lastRefreshTime += 1000;
    firstbebe = 1;
  } else {
    Serial.print("currtime "); Serial.println(currtime);
    Serial.print("prevtime "); Serial.println(prevtime);
    Serial.print("currirqtime "); Serial.println(currirqtime);
    Serial.print("previrqtime "); Serial.println(previrqtime);
    delays = (currtime-prevtime) - (currirqtime-previrqtime);
    REFRESH_INTERVAL -= delays;
    Serial.print("Delay "); Serial.println(delays);
  }
  Serial.println("--------------");

  String incoming = "";                 // payload of packet
  while (LoRa.available()) {            // can't use readString() in callback, so
    incoming += (char)LoRa.read();      // add bytes one by one
  }
  if (incoming[0] == "1"){
    for(int i = 0; i < 35; i++){
      packetint[0][i] = (uint8_t)(incoming[i]);
    }
    Serial.print("Got first");
    n += 1;
  }
  if (incoming[0] == "2"){
    for(int i = 0; i < 35; i++){
      packetint[1][i] = (uint8_t)(incoming[i]);
    }
    Serial.println("Got second");
    n += 1;
  }
  if (incoming[0] == "3"){
    for(int i = 0; i < 35; i++){
      packetint[2][i] = (uint8_t)(incoming[i]);
    }
    Serial.println("Got third");
    n += 1;
  }
  if (incoming[0] == "4"){
    for(int i = 0; i < 35; i++){
      packetint[3][i] = (uint8_t)(incoming[i]);
    }
    Serial.println("Got fourth");
    n += 1;
  }
  */
}
void floatToByte(uint8_t* bytes, float f){
  int length = sizeof(float);
  for(int i = 0; i < length; i++){
    bytes[i] = ((uint8_t*)&f)[i];
  }
}
int32_t GPSToInt(char* arr, uint8_t mode){
    if (mode==0){
        char sad[11] = "0000000000";
        uint8_t counteria = 0;
        for(uint8_t i = 0; i < 11; i++){
            if (arr[i] != '.'){
                sad[counteria] = arr[i];
                counteria += 1;
            } else {
                continue;
            }
        }
        return atol(sad);
    }
    if (mode==1){
        char sad[12] = "00000000000";
        uint8_t counteria = 0;
        for(uint8_t i = 0; i < 12; i++){
            if (arr[i] != '.'){
                sad[counteria] = arr[i];
                counteria += 1;
            } else {
                continue;
            }
        }
        return atol(sad);
    }
}
/*
void loop() {  
  if(millis() - lastRefreshTime >= REFRESH_INTERVAL){
    lastRefreshTime += REFRESH_INTERVAL;
    Serial.println(REFRESH_INTERVAL);
    if (isReceived) prevtime = currtime;
    currtime = millis();
    Serial.print("prevtime "); Serial.println(prevtime);
    Serial.print("currtime "); Serial.println(currtime);
    ///
    if (!firstreceive){
      if (n!=0) {
        LoRa.beginPacket();                
        LoRa.write(p, 32*n);              
        LoRa.endPacket();
      }
    }
    isReceived = 0;
    LoRa.receive();
  }
  
  
}
void onReceive(int packetSize) {
  if (packetSize == 0) return;   
  Serial.println("Got some");
  previrqtime = currirqtime;
  currirqtime = millis();
  if (firstreceive){
    firstreceive = 0;
    Serial.print(millis());
    isReceived = 1;
    lastRefreshTime -= 1000;
   // REFRESH_INTERVAL += 1000; // 1 APPARAT
    //lastRefreshTime = millis() - 1000;
  } else {
    isReceived = 1;
    Serial.print("currtime "); Serial.println(currtime);
    Serial.print("prevtime "); Serial.println(prevtime);
    Serial.print("currirqtime "); Serial.println(currirqtime);
    Serial.print("previrqtime "); Serial.println(previrqtime);
    delays = (currtime-prevtime) - (currirqtime-previrqtime);
    
    Serial.print("Delay "); Serial.println(delays);
    
    REFRESH_INTERVAL -= delays;
  }

  String incoming = "";                 // payload of packet
  while (LoRa.available()) {            // can't use readString() in callback, so
    incoming += (char)LoRa.read();      // add bytes one by one
  }
}
*/
