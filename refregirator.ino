/*
  refregirator.ino

  Термореле для бытового двухкамерного  холодильника
  Arduino Nano + 4 Relay Module + 2xDS18B20

  4 Relay Module 0 - канал ВКЛЮЧЕН, 1 - канал ВЫКЛЮЧЕН (вот такой модуль попался)

  При включении напряжения сети - пауза 10 минут до включения двигателей независимо от температуры.
  При отключении, повторное включение не ранее 10 минут независимо от температуры.
  При перезагрузке - пауза 10 минут до включения двигателей независимо от температуры.
  Если компрессор работает более 1 часа - принудительная остановка независимо от температуры.
  Если температурный датчик оборван или неисправен, то компрессор работает
  в цикле (20 мин работа/40 мин отдых), при этом загорается аварийный светодиод

  Version: 2017-06-12
*/

#include <OneWire.h>
#include <DallasTemperature.h>
//#include "LedControl.h"

/*
 Now we need a LedControl to work with.
 ***** These pin numbers will probably not work with your hardware *****
 pin A0 is connected to the DataIn 
 pin A1 is connected to the CLK
 pin A2 is connected to LOAD
 We have only a single MAX72XX.
 */
//LedControl lc=LedControl(14,15,16,1);

// константы. Потом перезапишу их в EEPROM
// Датчик температуры подключён к 8 пину
#define ONE_WIRE_BUS 8

// Холодильник
// Температура включения компрессора холодильника 5.0
#define TEMPERATURE1_MAX  5.0
// Температура отключения компрессора холодильника 1.0
#define TEMPERATURE1_MIN  1.0
// Реле холодильника подключено к 9 пину
#define RELE1 9
// Светодиод ошибки датчика температуры холодильника подключён к 11 пину
#define LED1_ERROR 11


// Пауза перед включением компрессоров (600000 = 10 минут)
#define DELAY_COMPRESSOR 600000
// Максимальное время работы компрессоров (3600000 = 1 час)
#define MAXTIME_COMPRESSOR 3600000

// Секция аварийного режима
// Время работы компрессора при ошибке датчика температуры (1200000 = 20 минут)
#define ERROR_COMPRESSOR_ON 1200000
// Время паузы компрессора при ошибке датчика температуры (2400000 = 40 минут)
#define ERROR_COMPRESSOR_OFF 2400000


// Светодиод активности подключён к 13 пину
#define LED_ACTIVITY 13
// Свечение светодиода активности в мс
#define LED_ACTIVITY_ON 50
// Пауза светодиода активности в мс
#define LED_ACTIVITY_OFF 3950


float f_tempC1;                        // Текущая температура холодильника

unsigned long ul_newTime;              // Текущее время

unsigned long ul_actHeartLedTime;      // Время светодиода активности
boolean b_actLedOn;                    // Светодиод активности включён


unsigned long ul_offTime1;             // Время отключения компрессора холодильника

unsigned long ul_workTime1;            // Время включения компрессора холодильника

unsigned long ul_deltaTime1;           // Разница времени для холодильника

boolean b_compressor1Off;              // Флаг компрессор холодильника выключен true / включен false

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress Thermometer1 = {0x28, 0xFF, 0xB2, 0x1B, 0x50, 0x15, 0x02, 0xFC };  // адрес датчика холодильника (маркирован одной точкой)

// =======================================================================================================================================
void setup() {
//  wdt_enable(WDTO_2S);

   
  pinMode(LED_ACTIVITY, OUTPUT);
  digitalWrite(LED_ACTIVITY, LOW);
  b_actLedOn = false;
  
  pinMode(RELE1, OUTPUT);
  digitalWrite(RELE1, HIGH);      // При загрузке компрессор холодильника отключен (ИНВЕРСНО)
  b_compressor1Off = true;        // выставляем флаг холодильника
   
  pinMode(LED1_ERROR, OUTPUT);
  digitalWrite(LED1_ERROR, LOW);  // Гасим светодиод ошибки холодильника
 
  sensors.begin();                // Инициализация датчиков температуры
  sensors.setResolution(Thermometer1, 9); // Режим точности 9 бит +-0.5 С
  
  ul_offTime1 = millis();          // Запоминаем время начала работы программы
  ul_actHeartLedTime = millis();
  
  // start serial port для отладки
  //Serial.begin(9600);

  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0,8);
  /* and clear the display */
  lc.clearDisplay(0);
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

// ==========================================================================================================================================
// Функция моргания светодиодом активности
void heartIndication(unsigned long ul_newLedTime) {
  unsigned long ul_deltaHeartTime = deltamills(ul_actHeartLedTime, ul_newLedTime);
  if (b_actLedOn) {
    // Если светодиод активности включён, ждём интервала LED_ACTIVITY_ON мс
    if (ul_deltaHeartTime > LED_ACTIVITY_ON) {
      digitalWrite(LED_ACTIVITY, LOW);
      b_actLedOn = false;
      ul_actHeartLedTime = ul_newLedTime;
    }
  } else {
    // Если светодиод активности выключен, ждём интервала LED_ACTIVITY_OFF мс
    if (ul_deltaHeartTime > LED_ACTIVITY_OFF) {
      digitalWrite(LED_ACTIVITY, HIGH);
      b_actLedOn = true;
      ul_actHeartLedTime = ul_newLedTime;
    }
  }
}
// ============================================================================================================================================
// Функция вывода числа на индикатор
// a - число, n - стартовая позиция
void printNumber(float a, int n) {
    int incoming;
    int v1;
    int v2;
    int v3;
    boolean negative;  

    negative = false;  
    if(a < 0) {
        negative = true;
        a = a * (-1);
    }
    incoming = (int)a * 10 ;
    v1 = incoming / 100;
    v2 = incoming % 100 / 10;
    v3 = incoming % 10;
       
    // теперь печатаем цифры на 1-ой, 2-ой и 3-ей позициях:
    lc.setDigit(0,2+n,(byte)v1,false);
    lc.setDigit(0,1+n,(byte)v2,true);
    lc.setDigit(0,0+n,(byte)v3,false);
    
    
    if(negative) {
       lc.setChar(0,3+n,'-',false);
    }
    else {
       // теперь печатаем знак «пробел» на 4-ой позиции:
       lc.setChar(0,3+n,' ',false);
    }
}
// ============================================================================================================================================
void loop() {
  wdt_reset();                         // Обнуляем WDT (сторожевой таймер)

  ul_newTime =  millis();              // Получаем текущее время
  heartIndication(ul_newTime);         // Моргаем светодиодом активности

  sensors.requestTemperatures();
  
  // для холодильника
  f_tempC1 = sensors.getTempC(Thermometer1); // Получаем температуру с датчика холодильника
  Serial.print("Temp C1: ");
  Serial.println(f_tempC1);
  printNumber (f_tempC1, 0); // выводим температуру в холодильнике
   
  // Проверяем работоспособность датчика температуры холодильника
  if (( f_tempC1 > -20.0 ) && ( f_tempC1 < 50.0 )) { // Температурный датчик исправен
    digitalWrite(LED1_ERROR, LOW);        // гасим индикатор ошибки холодильника
    
    if ( b_compressor1Off ) {                 // Если компрессор выключен
      ul_deltaTime1 = deltamills(ul_offTime1, ul_newTime); // Разница времени простоя
      if (ul_deltaTime1 > DELAY_COMPRESSOR) {  // Если разница времени простоя больше разрешенной
        if ( f_tempC1 > TEMPERATURE1_MAX ) {    // Если температура больше максимально допустимой
          digitalWrite(RELE1, LOW);           // Включаем компрессор
          b_compressor1Off = false;
          ul_workTime1 = millis();             // Записываем время включения
        }
      }
    } else {                                  // Если компрессор включен
      ul_deltaTime1 = deltamills(ul_workTime1, ul_newTime); // Разница времени работы
      // Если температура меньше минимально допустимой или время работы больше допустимого
      if ( ( f_tempC1 < TEMPERATURE1_MIN ) || ( ul_deltaTime1 > MAXTIME_COMPRESSOR ) ) {
        digitalWrite(RELE1, HIGH);              // Выключаем компрессор
        b_compressor1Off = true;
        ul_offTime1 = millis();                // Запоминаем время выключения компрессора
      }
    }
  } else {                                    // Температурный датчик неисправен или оборван
    digitalWrite(LED1_ERROR, HIGH);
    if ( b_compressor1Off ) {                  // Если компрессор выключен при неисправном датчике
      ul_deltaTime1 = deltamills(ul_offTime1, ul_newTime); // Разница времени простоя
      if (ul_deltaTime1 > ERROR_COMPRESSOR_OFF) {  // Если разница времени простоя больше разрешенной
        digitalWrite(RELE1, LOW);           // Включаем компрессор
        b_compressor1Off = false;
        ul_workTime1 = millis();             // Записываем время включения
      }
    } else {                                  // Если компрессор включен при неисправном датчике
      ul_deltaTime1 = deltamills(ul_workTime1, ul_newTime);
      if ( ul_deltaTime1 > ERROR_COMPRESSOR_ON ) {
        digitalWrite(RELE1, HIGH);              // Выключаем компрессор
        b_compressor1Off = true;
        ul_offTime1 = millis();                // Запоминаем время выключения компрессора
      }
    }
  }
  
} // END loop

