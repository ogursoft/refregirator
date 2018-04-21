/*
  refregirator.ino

  Термореле для бытового двухкамерного  холодильника Stinol
  WeMos D1 Mini + Relay Module + DS18B20

  При включении напряжения сети - пауза 10 минут до включения двигателя независимо от температуры.
  При отключении, повторное включение не ранее 10 минут независимо от температуры.
  При перезагрузке - пауза 10 минут до включения двигателя независимо от температуры.
  Если компрессор работает более 1 часа - принудительная остановка независимо от температуры.
  Если температурный датчик оборван или неисправен, то компрессор работает
  в цикле (20 мин работа/40 мин отдых), при этом загорается аварийный светодиод

  Version: 2018-04-20
*/

#include <OneWire.h>
#include <DallasTemperature.h>

#include <PersWiFiManager.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//extension of ESP8266WebServer with SPIFFS handlers built in
#include <SPIFFSReadServer.h> // http://ryandowning.net/SPIFFSReadServer/
// upload data folder to chip with Arduino ESP8266 filesystem uploader
// https://github.com/esp8266/arduino-esp8266fs-plugin

#include <DNSServer.h>
#include <FS.h>

#define DEVICE_NAME "Stinol"


//server objects
SPIFFSReadServer server(80);
DNSServer dnsServer;
PersWiFiManager persWM(server, dnsServer);

// Датчик температуры подключён к 2 пину (D4)
#define ONE_WIRE_BUS 2

// Холодильник
// Температура включения компрессора холодильника 5.0
#define TEMPERATURE1_MAX  5.0
// Температура отключения компрессора холодильника 1.0
#define TEMPERATURE1_MIN  1.0
// Реле холодильника подключено к 5 пину (D1)
#define RELE1 D1
// Светодиод  холодильника подключён к 14,12 и 13 пину (RGB)
#define RED D5 // Присваиваем имя RED для пина 14 (D5)
#define GREEN D6 // Присваиваем имя GREEN для пина 12 (D6)
#define BLUE D7 // Присваиваем имя BLUE для пина 13 (D7)

// Пауза между измерением температуры 10 сек
#define DELAY_TEMP_MEASURE 10000
// Пауза перед включением компрессоров (600000 = 10 минут)
#define DELAY_COMPRESSOR 600000
// Максимальное время работы компрессоров (3600000 = 1 час)
#define MAXTIME_COMPRESSOR 3600000

// Секция аварийного режима
// Время работы компрессора при ошибке датчика температуры (1200000 = 20 минут)
#define ERROR_COMPRESSOR_ON 1200000
// Время паузы компрессора при ошибке датчика температуры (2400000 = 40 минут)
#define ERROR_COMPRESSOR_OFF 2400000

#define DEBUG_SERIAL //uncomment for Serial debugging statements

#ifdef DEBUG_SERIAL
#define DEBUG_BEGIN Serial.begin(115200)
#define DEBUG_PRINT(x) Serial.println(x)
#define DEBUG_PRINTF(x,y) Serial.printf(x,y)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTF(x,y)
#define DEBUG_BEGIN
#endif


float f_tempC1;                        // Текущая температура холодильника
unsigned long ul_newTime;              // Текущее время
unsigned long ul_offTime1;             // Время отключения компрессора холодильника
unsigned long ul_workTime1;            // Время включения компрессора холодильника
unsigned long ul_deltaTime1;           // Разница времени для холодильника
unsigned long ul_lastMeasure;         // Разница времени для холодильника
boolean b_compressor1Off;              // Флаг компрессор холодильника выключен true / включен false

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress Thermometer1;  // адрес датчика холодильника

unsigned long deltamills(unsigned long t_old, unsigned long t_new);
String uptime( unsigned long currentmillis);

// =======================================================================================================================================
void setup() {

  pinMode(RED, OUTPUT); // Используем Pin14 для вывода
  pinMode(GREEN, OUTPUT); // Используем Pin12 для вывода
  pinMode(BLUE, OUTPUT); // Используем Pin13 для вывода

  digitalWrite(RED, LOW);
  digitalWrite(GREEN, HIGH); // Включаем зеленый свет свет (холодильник в рабочем режиме)
  digitalWrite(BLUE, LOW);

  pinMode(RELE1, OUTPUT);
  digitalWrite(RELE1, HIGH);      // При загрузке компрессор холодильника отключен
  b_compressor1Off = true;        // выставляем флаг холодильника

  sensors.begin();                // Инициализация датчиков температуры
  sensors.setResolution(Thermometer1, 9); // Режим точности 9 бит +-0.5 С

  ul_offTime1 = millis();          // Запоминаем время начала работы программы

  DEBUG_BEGIN; //for terminal debugging
  DEBUG_PRINT();

  //optional code handlers to run everytime wifi is connected...
  persWM.onConnect([]() {
    DEBUG_PRINT("wifi connected");
    DEBUG_PRINT(WiFi.localIP());
  });
  //...or AP mode is started
  persWM.onAp([]() {
    DEBUG_PRINT("AP MODE");
    DEBUG_PRINT(persWM.getApSsid());
  });

  //allows serving of files from SPIFFS
  SPIFFS.begin();
  //sets network name for AP mode
  persWM.setApCredentials(DEVICE_NAME);
  //persWM.setApCredentials(DEVICE_NAME, "password"); optional password

  //make connecting/disconnecting non-blocking
  persWM.setConnectNonBlock(true);

  //in non-blocking mode, program will continue past this point without waiting
  persWM.begin();

  //OTA handlers setup ===================================
  ArduinoOTA.setHostname(DEVICE_NAME);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    DEBUG_PRINT("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    DEBUG_PRINT("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DEBUG_PRINTF("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  // Sends Data to web page ==========================================
  server.on("/api", []() {
    DEBUG_PRINT("server.on /api");
    //build json object of program data
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject &json = jsonBuffer.createObject();
    json["temp"] = f_tempC1;
    if (b_compressor1Off == true) {
      json["compressor"] = "Выключен";
    } else {
      json["compressor"] = "Включен";
    }
    json["uptime"] = uptime(ul_newTime);
    json["worktime"] = uptime(ul_deltaTime1);
    char jsonchar[200];
    json.printTo(jsonchar); //print to char array, takes more memory but sends in one piece
    server.send(200, "application/json", jsonchar);
  }); //server.on api
  server.begin();
  DEBUG_PRINT("setup complete.");
}


// ==========================================================================================================================================
// Функция вычисления разницы времени
unsigned long deltamills(unsigned long t_old, unsigned long t_new) {
  unsigned long delta;
  if ( t_old <= t_new ) {
    delta = t_new - t_old;
  } else {
    delta = (4294967295 - t_old) + t_new;
  }
  return delta;
}

// Функция вывода uptime
String uptime( unsigned long currentmillis) {
  String currUptime;
  long days = 0;
  long hours = 0;
  long mins = 0;
  long secs = 0;
  secs = currentmillis / 1000; //convect milliseconds to seconds
  mins = secs / 60; //convert seconds to minutes
  hours = mins / 60; //convert minutes to hours
  days = hours / 24; //convert hours to days
  secs = secs - (mins * 60); //subtract the coverted seconds to minutes in order to display 59 secs max
  mins = mins - (hours * 60); //subtract the coverted minutes to hours in order to display 59 minutes max
  hours = hours - (days * 24); //subtract the coverted hours to days in order to display 23 hours max
  //Display results
  currUptime = "";
  if (days > 0) // days will displayed only if value is greater than zero
  {
    currUptime = String(days) + " дней, ";
  }
  if (hours < 10) {
    currUptime += "0" + String(hours) + ":";
  } else
    currUptime += String(hours) + ":";

  if (mins < 10) {
    currUptime += "0" + String(mins) + ":";
  } else
    currUptime += String(mins) + ":";

  if (secs < 10) {
    currUptime += "0" + String(secs);
  } else
    currUptime += String(secs);

  return currUptime;
}

// ============================================================================================================================================
void loop() {

  ArduinoOTA.handle();

  persWM.handleWiFi();

  dnsServer.processNextRequest();
  server.handleClient();

  ul_newTime =  millis();              // Получаем текущее время

//Делаем замеры температуры раз в 10 сек (DELAY_TEMP_MEASURE) 
  if (ul_newTime - ul_lastMeasure > DELAY_TEMP_MEASURE) {
    sensors.requestTemperatures();

    // для холодильника
    f_tempC1 = sensors.getTempC(Thermometer1); // Получаем температуру с датчика холодильника
    DEBUG_PRINT("Temp C1: " + String(f_tempC1));
    ul_lastMeasure = millis();

    // Проверяем работоспособность датчика температуры холодильника
    if (( f_tempC1 > -20.0 ) && ( f_tempC1 < 50.0 )) { // Температурный датчик исправен

      digitalWrite(RED, LOW);        // гасим индикатор ошибки холодильника если она есть
      digitalWrite(GREEN, HIGH);
      digitalWrite(BLUE, LOW);

      if ( b_compressor1Off ) {                 // Если компрессор выключен
        ul_deltaTime1 = deltamills(ul_offTime1, ul_newTime); // Разница времени простоя
        if (ul_deltaTime1 > DELAY_COMPRESSOR) {  // Если разница времени простоя больше разрешенной
          if ( f_tempC1 > TEMPERATURE1_MAX ) {    // Если температура больше максимально допустимой
            digitalWrite(RELE1, LOW);           // Включаем компрессор
            b_compressor1Off = false;
            ul_workTime1 = millis();             // Записываем время включения

            digitalWrite(RED, LOW);        // Зажигаем синий
            digitalWrite(GREEN, LOW);
            digitalWrite(BLUE, HIGH);
          }
        }
      } else {                                  // Если компрессор включен
        ul_deltaTime1 = deltamills(ul_workTime1, ul_newTime); // Разница времени работы
        // Если температура меньше минимально допустимой или время работы больше допустимого
        if ( ( f_tempC1 < TEMPERATURE1_MIN ) || ( ul_deltaTime1 > MAXTIME_COMPRESSOR ) ) {
          digitalWrite(RELE1, HIGH);              // Выключаем компрессор
          b_compressor1Off = true;
          ul_offTime1 = millis();                // Запоминаем время выключения компрессора
          digitalWrite(RED, LOW);        // Зажигаем зеленый
          digitalWrite(GREEN, HIGH);
          digitalWrite(BLUE, LOW);
        }
      }
    } else {                                    // Температурный датчик неисправен или оборван
      if ( b_compressor1Off ) {                  // Если компрессор выключен при неисправном датчике
        ul_deltaTime1 = deltamills(ul_offTime1, ul_newTime); // Разница времени простоя
        if (ul_deltaTime1 > ERROR_COMPRESSOR_OFF) {  // Если разница времени простоя больше разрешенной
          digitalWrite(RELE1, LOW);           // Включаем компрессор
          b_compressor1Off = false;
          ul_workTime1 = millis();             // Записываем время включения
          digitalWrite(RED, HIGH);        // Зажигаем красный
          digitalWrite(GREEN, LOW);
          digitalWrite(BLUE, LOW);
        }
      } else {                                  // Если компрессор включен при неисправном датчике
        ul_deltaTime1 = deltamills(ul_workTime1, ul_newTime);
        if ( ul_deltaTime1 > ERROR_COMPRESSOR_ON ) {
          digitalWrite(RELE1, HIGH);              // Выключаем компрессор
          b_compressor1Off = true;
          ul_offTime1 = millis();                // Запоминаем время выключения компрессора
          digitalWrite(RED, LOW);        // Зажигаем зеленый
          digitalWrite(GREEN, HIGH);
          digitalWrite(BLUE, LOW);
        }
      }
    }
  }
  yield();
} // END loop
