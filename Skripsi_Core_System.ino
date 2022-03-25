#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <PMsensor.h>
#include <MQ135.h>
#include <MQUnifiedsensor.h>
#include "DHT.h"
#include "RTClib.h"
#include "LowPower.h"

#define placa "Arduino UNO"
#define Voltage_Resolution 5
#define pin A0  //MQ-135
#define Pin A1  //MQ-9

#define type "MQ-135"
#define Type "MQ-9" //MQ9
#define ADC_Bit_Resolution 10

#define RatioMQ135CleanAir 3.6
#define RatioMQ9CleanAir   9.6

#define DHTPIN 7
#define DHTTYPE DHT11
#define resetButton 2
#define pinReset  9
#define Buzzer  6

int upButton       = 10;
int downButton     = 11;
int SelectButton   = 12;
int menu           = 1;

float h, t, f, hif, hic;

//Variabel untuk menampilkan hari
char daysOfTheWeek[7][12] = { "Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jum'at", "Sabtu" };

void (*sistem_restart) (void) = 0;

const int checkInHour1   = 5;  const int checkInMinute1 = 00; //Reset di jam 5:00
const int checkInHour2   = 9;  const int checkInMinute2 = 00; //Reset di jam 9:00
const int checkInHour3   = 13; const int checkInMinute3 = 05; //Reset di jam 13:00

const int checkInHour4   = 17; const int checkInMinute4 = 00; //Reset di jam 17:00
const int checkInHour5   = 21; const int checkInMinute5 = 00; //Reset di jam 21:00
const int checkInHour6   = 0;  const int checkInMinute6 = 00; //Reset di jam 0:00

const int checkInSeconds1 = 10; const int checkInSeconds2 = 10; //Setting 10 detik
const int checkInSeconds3 = 10; const int checkInSeconds4 = 10; //Setting 10 detik
const int checkInSeconds5 = 10; const int checkInSeconds6 = 10; //Setiing 10 detik

const int checkLowPower1 = 6;
const int checkLowPower2 = 10;
const int checkLowPower3 = 13;
const int checkLowPower4 = 16;
const int checkLowPower5 = 19;
const int checkLowPower6 = 0;
const int checkLowPower7 = 2;

const int checkInMinute7 = 30;

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 20, 4);

RTC_DS1307 rtc;
PMsensor PM;

SoftwareSerial portOne(2, 3); //RX, TX (Port one) DF Player
SoftwareSerial portTwo(4, 5); //RX, TX (Port Two) GPS Satelit

MQ135 mq135_sensor(pin);
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);
MQUnifiedsensor MQ9(placa, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

void AirQuality() {
  MQ135.setRegressionMethod(1); MQ135.init();

  float calcR0 = 0;
  for (int i = 1; i <= 10; i ++)
  {
    MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
  }
  MQ135.setR0(calcR0 / 10);
}

void AirQuality1() {
  MQ9.init();
  MQ9.setRegressionMethod(1);
  float calcR0 = 0;
  for (int i = 1; i <= 10; i ++)
  {
    MQ9.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ9.calibrate(RatioMQ9CleanAir);
  }
  MQ9.setR0(calcR0 / 10);
}

void setup() {
  // put your setup code here, to run once:
  digitalWrite(pinReset, HIGH);
  delay(200);

  Serial.begin(9600); //Serial komunikasi uart Arduino
  while (!Serial) continue;
  portOne.begin(9600); //Serial Komunikasi untuk modul DF Player
  portTwo.begin(9600); //Serial Komunilasi Untuk Modul GPS Satelit

  dht.begin();        //memulai pembacaan sensor DHT
  PM.init(13, A2);
  lcd.init();         //memulai menampilkan data LCD I2C
  lcd.backlight();    //nyalakan lampu backlight Arduino

  pinMode(upButton,     INPUT_PULLUP); //Tombol up untuk menilih menu bagian atas
  pinMode(downButton,   INPUT_PULLUP); //Tombol down untuk memilih menu bagian bawah
  pinMode(SelectButton, INPUT_PULLUP); //Tombol untuk memilih pilihan pada menu
  pinMode(resetButton,  INPUT_PULLUP); //Tombol untuk reset secara hardsoft
  pinMode(pinReset, OUTPUT); //Pin Reset untuk reboot otomatis
  pinMode(Buzzer,   OUTPUT); //Pin buzzer sebagai Output

  digitalWrite(Buzzer, HIGH);
  delay(700);
  digitalWrite(Buzzer, LOW);

  if (! rtc.begin()) {
    Serial.println("RTC tidak terbaca");
    Serial.flush();
    while (1) delay(10);
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, let's set the time!");
    //calibration code with rtc module
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  //rtc.adjust(DateTime(2022, 3, 20, 13, 3, 0));

  AirQuality();
  AirQuality1();
  updateMenu();  //panggil function update menu untuk menampilkan daftar menu
}

void Reset() {
  int buttonStatusSaya = digitalRead(resetButton);
  if (buttonStatusSaya == LOW)
  {
    Serial.println("coba tampilkan tulisan ini kalo bisa");
    digitalWrite(pinReset, LOW);
  }
  else
  {
    //digitalWrite(ledReset, HIGH);
  }
}

void automotion() {

  DateTime now = rtc.now();
  int systemInHour   = now.hour();
  int systemInMinute = now.minute();
  int systemSeconds  = now.second();

  if ( systemInHour  == checkInHour1 && systemInMinute == checkInMinute1 &&
       systemSeconds == checkInSeconds1) sistem_restart();
  if ( systemInHour  == checkInHour2 && systemInMinute == checkInMinute2 &&
       systemSeconds == checkInSeconds2) sistem_restart();
  if ( systemInHour  == checkInHour3 && systemInMinute == checkInMinute3 &&
       systemSeconds == checkInSeconds3) sistem_restart();
  if ( systemInHour  == checkInHour4 && systemInMinute == checkInMinute4 &&
       systemSeconds == checkInSeconds4) sistem_restart();
  if ( systemInHour  == checkInHour5 && systemInMinute == checkInMinute5 &&
       systemSeconds == checkInSeconds5) sistem_restart();
  if ( systemInHour  == checkInHour6 && systemInMinute == checkInMinute6 &&
       systemSeconds == checkInSeconds6) sistem_restart();

  if (systemInHour  == checkLowPower1 && systemInMinute == checkInMinute7 &&
      systemSeconds == checkInSeconds1) {
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER5_OFF, TIMER4_OFF, TIMER3_OFF,
                  TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART3_OFF,
                  USART2_OFF, USART1_OFF, USART0_OFF, TWI_OFF);
  }

  if (systemInHour  == checkLowPower2 && systemInMinute == checkInMinute7 &&
      systemSeconds == checkInSeconds1) {
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER5_OFF, TIMER4_OFF, TIMER3_OFF,
                  TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART3_OFF,
                  USART2_OFF, USART1_OFF, USART0_OFF, TWI_OFF);
  }

  if (systemInHour  == checkLowPower3 && systemInMinute == checkInMinute7 &&
      systemSeconds == checkInSeconds1) {
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER5_OFF, TIMER4_OFF, TIMER3_OFF,
                  TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART3_OFF,
                  USART2_OFF, USART1_OFF, USART0_OFF, TWI_OFF);
  }

  if (systemInHour  == checkLowPower4 && systemInMinute == checkInMinute7 &&
      systemSeconds == checkInSeconds1) {
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER5_OFF, TIMER4_OFF, TIMER3_OFF,
                  TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART3_OFF,
                  USART2_OFF, USART1_OFF, USART0_OFF, TWI_OFF);
  }

  if (systemInHour  == checkLowPower5 && systemInMinute == checkInMinute7 &&
      systemSeconds == checkInSeconds1) {
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER5_OFF, TIMER4_OFF, TIMER3_OFF,
                  TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART3_OFF,
                  USART2_OFF, USART1_OFF, USART0_OFF, TWI_OFF);
  }

  if (systemInHour  == checkLowPower6 && systemInMinute == checkInMinute7 &&
      systemSeconds == checkInSeconds1) {
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER5_OFF, TIMER4_OFF, TIMER3_OFF,
                  TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART3_OFF,
                  USART2_OFF, USART1_OFF, USART0_OFF, TWI_OFF);
  }

  if (systemInHour  == checkLowPower7 && systemInMinute == checkInMinute7 &&
      systemSeconds == checkInSeconds1) {
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER5_OFF, TIMER4_OFF, TIMER3_OFF,
                  TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART3_OFF,
                  USART2_OFF, USART1_OFF, USART0_OFF, TWI_OFF);
  }

}

void buttonControl() {
  if (!digitalRead(downButton)) {
    menu++;
    updateMenu();
    delay(100);
    while (!digitalRead(downButton));
  }
  if (!digitalRead(upButton)) {
    menu--;
    updateMenu();
    delay(100);
    while (!digitalRead(upButton));
  }
  if (!digitalRead(SelectButton)) {
    executeAction();
    updateMenu();
    delay(100);
    while (!digitalRead(SelectButton));
  }
}

void updateMenu() {
  switch (menu) {
    case 0: menu = 1; break;
    case 1:
      lcd.clear();
      lcd.print(">Digital Clock");
      lcd.setCursor(0, 1);
      lcd.print(" Humidity  Rh%");
      lcd.setCursor(0, 2);
      lcd.print(" Hazardous Gas");
      lcd.setCursor(0, 3);
      lcd.print(" Flammable Gas");
      break;
    case 2:
      lcd.clear();
      lcd.print(" Digital Clock");
      lcd.setCursor(0, 1);
      lcd.print(">Humidity  Rh%");
      lcd.setCursor(0, 2);
      lcd.print(" Hazardous Gas");
      lcd.setCursor(0, 3);
      lcd.print(" Flammable Gas");
      break;
    case 3:
      lcd.clear();
      lcd.print(" Digital Clock");
      lcd.setCursor(0, 1);
      lcd.print(" Humidity  Rh%");
      lcd.setCursor(0, 2);
      lcd.print(">Hazardous Gas");
      lcd.setCursor(0, 3);
      lcd.print(" Flammable Gas");
      break;
    case 4:
      lcd.clear();
      lcd.print(" Digital Clock");
      lcd.setCursor(0, 1);
      lcd.print(" Humidity  Rh%");
      lcd.setCursor(0, 2);
      lcd.print(" Hazardous Gas");
      lcd.setCursor(0, 3);
      lcd.print(">Flammable Gas");
      break;
    case 5:
      lcd.clear();
      lcd.print(">Air Polution");
      lcd.setCursor(0, 1);
      lcd.print(" AES-128");
      lcd.setCursor(0, 2);
      lcd.print(" Lora RSSI");
      lcd.setCursor(0, 3);
      lcd.print(" GPS Satelit");
      break;
    case 6:
      lcd.clear();
      lcd.print(" Air Polution");
      lcd.setCursor(0, 1);
      lcd.print(">AES-128");
      lcd.setCursor(0, 2);
      lcd.print(" Lora RSSI");
      lcd.setCursor(0, 3);
      lcd.print(" GPS Satelit");
      break;
    case 7:
      lcd.clear();
      lcd.print(" Air Polution");
      lcd.setCursor(0, 1);
      lcd.print(" AES-128");
      lcd.setCursor(0, 2);
      lcd.print(">Lora RSSI");
      lcd.setCursor(0, 3);
      lcd.print(" GPS Satelit");
      break;
    case 8:
      lcd.clear();
      lcd.print(" Air Polution");
      lcd.setCursor(0, 1);
      lcd.print(" AES-128");
      lcd.setCursor(0, 2);
      lcd.print(" Lora RSSI");
      lcd.setCursor(0, 3);
      lcd.print(">GPS Satelit");
      break;
    case 9:
      menu = 8;
      break;
  }
}

void executeAction() {
  switch (menu) {
    case 1:  action1();  break;
    case 2:  action2();  break;
    case 3:  action3();  break;
    case 4:  action4();  break;
    case 5:  action5();  break;
  }
}

void action1() {
  lcd.clear();
  modulRTC();
  delay(2100);
}

void action2() {
  lcd.clear();
  sensorDHT();
  delay(2100);
}

void action3() {
  lcd.clear();
  Calibration1();
  delay(2100);
}

void action4() {
  lcd.clear();
  Calibration2();
  delay(2100);
}

void action5() {
  lcd.clear();
  AirPoluttion();
  delay(2100);
}

void loop() {
  buttonControl(); // function untuk menjalankan kontrol  menu
  automotion();    // function untuk menjalankan restart sistem secara
  Reset();
}

void modulRTC() {
  DateTime now = rtc.now();

  lcd.setCursor(0, 0);
  lcd.print(now.day(),   DEC); lcd.print('/');
  lcd.print(now.month(), DEC); lcd.print('/');
  lcd.print(now.year(),  DEC);

  lcd.setCursor(10, 0);
  lcd.print(daysOfTheWeek[now.dayOfTheWeek()]);

  lcd.setCursor(0, 1);     lcd.print("Jam :");
  lcd.print(now.hour());   lcd.print(':');
  lcd.print(now.minute()); lcd.print(':');
  lcd.print(now.second()); lcd.print("    ");
}

void sensorDHT() {
  h = dht.readHumidity();
  t = dht.readTemperature();
  f = dht.readTemperature(true);

  hif = dht.computeHeatIndex(f, h);
  hic = dht.computeHeatIndex(t, h, false);

  lcd.print("Humi  : "); lcd.print(h); lcd.print(" RH % ");
  lcd.setCursor(0, 1);
  lcd.print("TempC : "); lcd.print(t); lcd.print(char(223)); lcd.print("C ");
  lcd.setCursor(0, 2);
  lcd.print("TempF : "); lcd.print(f); lcd.print(" J ");
  lcd.setCursor(0, 3);
  lcd.print("HeatC : "); lcd.print(hic); lcd.print(" J ");
}

void Calibration1() {
  MQ135.update();

  MQ135.setA(605.18); MQ135.setB(-3.937);
  float CO = MQ135.readSensor();

  MQ135.setA(110.47); MQ135.setB(-2.862);
  float CO2 = MQ135.readSensor();

  MQ135.setA(102.2 ); MQ135.setB(-2.473);
  float NH4 = MQ135.readSensor();

  MQ135.setA(34.668); MQ135.setB(-3.369);
  float Acetona = MQ135.readSensor();

  lcd.print("Co   : "); lcd.print(CO); lcd.print(" PPM ");
  lcd.setCursor(0, 1);
  lcd.print("NH4  : "); lcd.print(NH4); lcd.print(" PPM ");

  lcd.setCursor(0, 2);
  lcd.print("Co2  : "); lcd.print(CO2); lcd.print(" PPM ");
  lcd.setCursor(0, 3);
  lcd.print("C3H6O: "); lcd.print(Acetona); lcd.print(" PPM ");
}

void Calibration2() {
  MQ135.update();
  MQ9.update();

  MQ135.setA(77.255); MQ135.setB(-3.18);
  float Alcohol = MQ135.readSensor();

  MQ135.setA(44.947); MQ135.setB(-3.445);
  float Tolueno = MQ135.readSensor();

  MQ9.setA(1000.5); MQ9.setB(-2.186);
  float LPG = MQ9.readSensor();

  MQ9.setA(4269.6); MQ9.setB(-2.648);
  float CH4 = MQ9.readSensor();

  lcd.print("C2H5 : "); lcd.print(Alcohol); lcd.print(" PPM ");
  lcd.setCursor(0, 1);
  lcd.print("C7H8 : "); lcd.print(Tolueno); lcd.print(" PPM ");
  lcd.setCursor(0, 2);
  lcd.print("LPG  : "); lcd.print(LPG); lcd.print(" PPM ");
  lcd.setCursor(0, 3);
  lcd.print("CH4  : "); lcd.print(CH4); lcd.print(" PPM ");
}

void AirPoluttion() {
  float filter_Data = PM.read(0.1);
  float noFilter_Data = PM.read();

  float rzero = mq135_sensor.getRZero();
  float correctedRZero = mq135_sensor.getCorrectedRZero(t, h);

  float resistance = mq135_sensor.getResistance();
  float ppm = mq135_sensor.getPPM();

  float correctedPPM = mq135_sensor.getCorrectedPPM(t, h);

  lcd.print("DebuF : "); lcd.print(filter_Data); lcd.print(" ug/m3");
  lcd.setCursor(0, 1);
  lcd.print("DebuN : "); lcd.print(noFilter_Data); lcd.print(" ug/m3");
  lcd.setCursor(0, 2);
  lcd.print("CorFac: "); lcd.print(correctedRZero);
  lcd.setCursor(0, 3);
}

void SatelitGPS() {

}
