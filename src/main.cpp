/*

 * File-name: main.cpp
 * Author: Raoby
 * Email: zhk817@vip.qq.com
 * School-info: WUST IEEE 2018-1
 * Create-Date: 2020 12th Dec.
 * Subject: Project Wastesort MCU Controller
 * Operating system: GNU-Linux 5.7.7-amd64-desktop (Deepin 20)
 * IDE: Visual Studio Code WITH PlatformIO
 * PLATFORM: Espressif 8266 (2.6.2) > WeMos D1 R2 and mini
 * HARDWARE: ESP8266 80MHz, 80KB RAM, 4MB Flash

*/

#include <Arduino.h>
#include <Servo.h>
#include <U8g2lib.h>
#include <ArduinoJson.h>
#include <Wire.h>


U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define Servor_Pin_1  D3
#define Servor_Pin_2  D4


#define Sensor_Echo D0

Servo servos[2];
String SerialJson;

int servoAngle[2]={0,135};
int currerentType = 0;
unsigned long startActionTime = 0;

unsigned int Sensor_Trig[4]={D5,D6,D7,D8};
float Sensor_Result[4]={0,0,0,0};
unsigned long lastCheckSensor = 0;


void Measure_Distance(unsigned int SensorId){
  digitalWrite(Sensor_Trig[SensorId-1],HIGH);
  delayMicroseconds(20);
  digitalWrite(Sensor_Trig[SensorId-1],LOW);
  unsigned long measure_temp=0;
  measure_temp = pulseIn(Sensor_Echo, HIGH, 10000); //存储回波等待时间，不超过0.01秒
  if(measure_temp <= 0 || measure_temp >=10000){
    Sensor_Result[SensorId-1] = 0; //读取错误，数据继续储存为0
  }else{
    Sensor_Result[SensorId-1] = (measure_temp * 17 )/1000; //将计算后得到的正确结果写入记录数组
  }
  
}

void uploadAllData(){
  //数据格式: Alldata:类型,角度1,角度2,距离1,距离2,距离3,距离4
  char uploaddata[256] = "";
  sprintf(uploaddata,"Alldata:%d,%d,%d,%.1f,%.1f,%.1f,%.1f\r\n",currerentType,servoAngle[0],servoAngle[1],Sensor_Result[0],Sensor_Result[1],Sensor_Result[2],Sensor_Result[3]);
  if(Serial.availableForWrite()){
    Serial.print(uploaddata);
  }
}

void handleSerial(String json)
{
  DynamicJsonBuffer serialargsbuf;
  JsonObject &serialargs = serialargsbuf.parseObject(json);
  if(serialargs.success())
  {
    if(serialargs.containsKey("type"))
    {
      currerentType = serialargs["type"].as<int>();
    }
    if(serialargs.containsKey("angle1"))
    {
      servoAngle[0] = serialargs["angle1"].as<int>();
    }
    if(serialargs.containsKey("angle2"))
    {
      servoAngle[1] = serialargs["angle2"].as<int>();
    }
    uploadAllData();
  }
}

void setup() {

  // Servo init
  servos[0].attach(Servor_Pin_1); 
  servos[1].attach(Servor_Pin_2); 

  // Sensor init
  pinMode(Sensor_Trig[0], OUTPUT);
  pinMode(Sensor_Trig[1], OUTPUT);
  pinMode(Sensor_Trig[2], OUTPUT);
  pinMode(Sensor_Trig[3], OUTPUT);
  pinMode(Sensor_Echo, INPUT);
  digitalWrite(Sensor_Trig[0], LOW); //给Trig1发送一个低电平
  digitalWrite(Sensor_Trig[1], LOW); //给Trig2发送一个低电平
  digitalWrite(Sensor_Trig[2], LOW); //给Trig3发送一个低电平
  digitalWrite(Sensor_Trig[3], LOW); //给Trig4发送一个低电平

   ////////////////////////////////
  // SERIAL INIT
  Serial.begin(115200);
  //Serial.setDebugOutput(true);
  Serial.print('\n');



  u8g2.begin();
  
  /*
  u8g2log.begin(u8g2, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
  u8g2log.setLineHeightOffset(0);   // set extra space between lines in pixel, this can be negative
  u8g2log.setRedrawMode(0);         // 0: Update screen with newline, 1: Update screen for every char            */
  //u8g2.setFont(u8g2_font_t0_11_tr); // choose a suitable font
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.enableUTF8Print();
  u8g2.setFontDirection(0);
  ////////////////////////////////
  u8g2.sendBuffer();
}

void loop() {
  if(Serial.available())
  {
    SerialJson = "";
    while (Serial.available() > 0)  
    {
        SerialJson += char(Serial.read());
        delay(1);
    }
    handleSerial(SerialJson);
  }
  if(currerentType&&!startActionTime)
  {
    Serial.println(String("Change type -> "+ String(currerentType)));
    startActionTime = millis();
    Serial.println(startActionTime);
  }
  if(startActionTime)
  {
    if(millis()-startActionTime<=2000)
    {
      servoAngle[0] = (currerentType-1)*90;
    }else if(millis()-startActionTime<=4000)
    {
      servoAngle[1] = 180;
    }else if(millis()-startActionTime>=6000)
    {
      currerentType = 0;
      startActionTime = 0;
      servoAngle[0] = 0;
      servoAngle[1] = 135;
      Serial.println(String("Change angle to default"));
    }
  }


  //Control servo's angle
  servos[0].writeMicroseconds(500+2000*servoAngle[0]/270);
  servos[1].writeMicroseconds(500+2000*servoAngle[1]/270);

  u8g2.firstPage(); //Print servo info to oled screen
  do {
    u8g2.setCursor(72,15);
    u8g2.print(String("Time:")+String(millis()/1000));
    u8g2.setCursor(0,15);
    u8g2.print("Servo-1:");
    u8g2.setCursor(0,30);
    u8g2.print("Servo-2:");
    u8g2.setCursor(0,45);
    u8g2.print("Sensor-Result(cm):");
    u8g2.setCursor(50,15);
    u8g2.print(String(servoAngle[0]));
    u8g2.setCursor(50,30);
    u8g2.print(String(servoAngle[1]));
    u8g2.setCursor(0,60);
    u8g2.print(String(int(Sensor_Result[0])));
    u8g2.setCursor(32,60);
    u8g2.print(String(int(Sensor_Result[1])));
    u8g2.setCursor(64,60);
    u8g2.print(String(int(Sensor_Result[2])));
    u8g2.setCursor(96,60);
    u8g2.print(String(int(Sensor_Result[3])));

    /*
      u8g2.setCursor(0,15);
      u8g2.print("Servo-1:");
      u8g2.setCursor(4,30);
      u8g2.print("Angle:"+String(servoAngle[0]));
      u8g2.setCursor(60,30);
      u8g2.print("PWM:"+String(500+2000*servoAngle[0]/270));
      u8g2.setCursor(0,45);
      u8g2.print("Servo-2:");
      u8g2.setCursor(4,60);
      u8g2.print("Angle:"+String(servoAngle[1]));
      u8g2.setCursor(60,60);
      u8g2.print("PWM:"+String(500+2000*servoAngle[1]/270));
      */
  } while ( u8g2.nextPage() );
  //距离上次检测超过1S则检测传感器并上报
  if((millis()-lastCheckSensor)>=2000){
    Measure_Distance(1);
    Measure_Distance(2);
    Measure_Distance(3);
    Measure_Distance(4);
    uploadAllData();
    lastCheckSensor = millis();
  }

}
