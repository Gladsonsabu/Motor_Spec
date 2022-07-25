/*


 *          
 *          
PINOUT to Custom Inbtegrated Board: 
 *        _________________________________________________________________________________
 *       |  ARDUINO UNO    >>>    Custom Development board   |    MEAS   |   ADS1115   |    
 *        ---------------------------------------------------------------------------------
 *            5V          >>>             Vcc                     GND            -          
 *           GND          >>>             GND                      -             -           
 *            A0          >>>             PT(Stmup Male)           -             -         
 *            A4          >>>             SDA                      -             -        
 *            A5          >>>             SCK                      -             -        
 *          GPIO 5        >>>             DHT OUT                  -             -          
 *          GPIO 6        >>>             piezo +                  -             -          
 *            *                             A3              >>>   MEAS_L         -
 *            *                             A4              >>>   MEAS_R         -
 *            *                           EXPANDER PORT     >>>                 ADS PORT 

*/


#include <movingAvg.h>
#include<SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1X15.h>
#include "pitches.h"
#include "DHTStable.h"


DHTStable DHT;
Adafruit_ADS1115 ADS;
LiquidCrystal_I2C lcd(0x27,16,2);
movingAvg MAF(25);
SoftwareSerial usb(3,2); //RX,TX

void Serprint(float,float);
float Irm(void);
void getdht(void);
float getPT100(void);
void lcdsweepL(void);
void lcdsweepR(void);
void PrintMotorTemp(float);
void PrintTH(void);
int Vibration(bool);
void datachk(float,double);

struct
{
  uint32_t total;
  uint32_t ok;
  uint32_t crc_error;
  uint32_t time_out;
  uint32_t connect;
  uint32_t ack_l;
  uint32_t ack_h;
  uint32_t unknown;
} counter = { 0,0,0,0,0,0,0,0};


const float FACTOR = 30;
const float multiplier = 0.2530F;//0.0625F
float IRMS, Power;
float Voltage = 234.0;

float Temperature_DHT = 0;
float Humidity_DHT = 0;

double T_Motor = 0;

const int SensorValueLow = 463; 
const int SensorValueDiff = 36; // differance between high and low sensor value
const int TempValueDiff = 42; // differance between high and low Temp value
const int TempValueLow = 9;

long t_prev;
int lcdptr = 1;
byte degree[8] = { 0x0, 0xe, 0xa, 0xe, 0x0, 0x0, 0x0, 0x0};
bool _resume = true;

int melody[] = {NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4};
int noteDurations[] = {4, 8, 8, 4, 4, 4, 4, 4};

float IRMSthresh = 8;
float Tempthresh = 50;

void setup() 
{
  Serial.begin(9600);
//  usb.begin(115200);//@high speed 115200
  
  lcd.init(); 
  lcd.backlight();
  lcd.createChar(0, degree);
  ADS.setGain(GAIN_TWO);
  ADS.begin();
  MAF.begin();
  lcd.clear();
  lcd.setCursor(1,0);
  lcd.print(F("Setup Properly"));
  lcd.setCursor(2,1);
  lcd.print(F("Initialised"));
  t_prev = millis();
  delay(2000);
}


void loop() {
  IRMS = Irms();
  Power = Voltage * IRMS;
  getdht();
  T_Motor=getPT100();
//  usb.print(IRMS);
//  usb.print("\t");
//  usb.print(Temperature_DHT);
//  usb.print("\t");
//  usb.print(Humidity_DHT);
//  usb.print("\t");
//  usb.println(T_Motor);
//  usb.print("\t");
//  usb.print(Vibration(1));
//  usb.print("\t");
//  usb.println(Vibration(0));
//datachk();
  if(millis() - t_prev > 2500){
    switch(lcdptr){
      case 1:
      {
        lcdsweepL();
        PrintI(IRMS,Power);
        lcdptr++;
        break;
      }
      case 2:
      {
        lcdsweepR();
        PrintTH();
        lcdptr++;
        break;
      }
      case 3:
      {
        lcdsweepL();
        PrintMotorTemp(T_Motor);
        lcdptr++;
        break;
      }
      case 4:
      {
        lcdsweepR();
        datachk(IRMS,T_Motor);
        lcdptr = 1;
        break;
      }
      default:
      {
        lcdptr = 1;
        break;
      }
    }
    t_prev = millis();
  }
  
}


/*--------------------------------------------------------------------------------------------------------------------------------------------*/
/*-----------------------Function Definition----------------------------------------Function Definition---------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------------------------------*/





void lcdsweepL(){
  for(short i=0; i<=16; i++)// Performs the shifting function
  {     
    lcd.print(F(" "));
    lcd.setCursor(i,0);
    lcd.print(F(" "));
    lcd.setCursor(i,1);
    delay(20);
  }
}

void lcdsweepR(){
  for(short i=16; i>=0; i--)
  {     // Performs the shifting function
    lcd.print(F(" "));
    lcd.setCursor(i,0);
    lcd.print(F(" "));
    lcd.setCursor(i,1);
    delay(20);
  }
}

void PrintI(float value1, float value2){
  Serial.print(F("Average RMS Current: "));
  Serial. print(value1, 3);
  Serial.print(F("A.\t\t"));
  Serial.print(F("Power:"));
  Serial.print(value2, 3);
  Serial.println(F("W"));
  lcd.setCursor(2,0);
  lcd.print(F("I RMS :"));
  lcd.print(value1);
  lcd.print(F("A"));
  lcd.setCursor(2,1);
  lcd.print(F("POWER :"));
  lcd.print(value2);
  lcd.print(F("W"));
}

float Irms(){
  float voltage, Curr_temp;
  float Curr_sum = 0;
  long t_start = millis();
  uint8_t counter = 0;
  while (millis() - t_start < 500){
    voltage = ADS.readADC_Differential_0_1() * multiplier;
    Curr_temp = voltage * FACTOR;
    Curr_temp /= 1000.0;
    Curr_sum += sq(Curr_temp);
    counter = counter +1;
    delay(3);
  }
  Serial.print(F("Samples to calculate IRMS:"));
  Serial.println(counter);
  return(sqrt(Curr_sum / counter));
}

void getdht(){
  int chk = DHT.read22(5);
  switch (chk)
    {
    case DHTLIB_OK:
        counter.ok++;
        Serial.println("DHT is OK,\t");
        break;
    case DHTLIB_ERROR_CHECKSUM:
        counter.crc_error++;
        Serial.println("Checksum error,\t");
        lcd.clear();
        lcd.setCursor(3,0);
        lcd.print(F("DHT ERROR"));
        lcd.setCursor(4,1);
        lcd.print(F("CHECKSUM"));
        delay(3000);
        break;
    case DHTLIB_ERROR_TIMEOUT:
        counter.time_out++;
        Serial.println("Time out error,\t");
        lcd.clear();
        lcd.setCursor(3,0);
        lcd.print(F("DHT ERROR"));
        lcd.setCursor(4,1);
        lcd.print(F("TIMEOUT"));
        delay(3000);
        break;
    default:
        counter.unknown++;
        Serial.println("Unknown error,\t");
        lcd.clear();
        lcd.setCursor(3,0);
        lcd.print(F("DHT ERROR"));
        lcd.setCursor(4,1);
        lcd.print(F("UNDEFINED"));
        delay(3000);
        break;
    }
  Temperature_DHT = DHT.getTemperature();
  Humidity_DHT = DHT.getHumidity();
  
}

void PrintTH(){
  lcd.setCursor(0,0);
  lcd.print(F("TEMPER.:"));
  lcd.print(Temperature_DHT);
  lcd.write(0);
  lcd.print(F("C"));
  lcd.setCursor(3,1);
  lcd.print(F("RH:"));
  lcd.print(Humidity_DHT); 
  lcd.print(F("%"));
  Serial.print(F("Ambient temperature: "));
  Serial. print(Temperature_DHT, 3);
  Serial.print(F("C.\t\t"));
  Serial.print(F("Ambient Humidity:"));
  Serial. print(Humidity_DHT, 3);
  Serial.println(F("%"));
}

float getPT100(){
  double Temp;
  int sensorValue = 0;
  sensorValue = analogRead(A0);
  Temp = sensorValue-SensorValueLow;
  Temp = Temp/SensorValueDiff;
  Temp = Temp*TempValueDiff;
  Temp = Temp+TempValueLow;
  Serial.print(F("PT100 temperature readout ="));
  Serial.println(Temp); // printing temperature on the serial monitor
  return(Temp);
}

void PrintMotorTemp(float value1){
  lcd.setCursor(1,0);
  lcd.print(F("MOTOR TEMPER."));
  lcd.setCursor(3,1);
  lcd.print(F(":"));
  lcd.print(value1); 
  lcd.write(0);
  lcd.print(F("C"));
  Serial.print(F("Motor temperature: "));
  Serial.println(value1, 3);
}

int Vibration(bool x){
  int Viber_Magni = abs(ADS.readADC_Differential_2_3());
  int VibMagAvg = MAF.reading(Viber_Magni);
  if(x){return(Viber_Magni);}
  else{return(VibMagAvg);}
}

void datachk(float IRMS, double Mtemp){
  if (IRMS > IRMSthresh){
    _resume = true;
    while(_resume){
      lcd.setCursor(2,0);
      lcd.print(F("OVER CURRENT"));
      lcd.setCursor(4,1);
      lcd.print(F("DETECTED"));
      tone(6,880,1000/8); //play the note "A5" (LA5)
      delay(200);
      tone(6,698,1000/8); //play the note "F6" (FA5)
      delay(200);
      if(Irms() < IRMSthresh)
      {
        _resume = false;
      }
    }
  }
  else if(abs(Mtemp-Temperature_DHT)>Tempthresh){
    _resume = true;
    while(_resume){
      lcd.setCursor(2,0);
      lcd.print(F("OVER-HEATING"));
      lcd.setCursor(4,1);
      lcd.print(F("DETECTED"));
      for (int thisNote = 0; thisNote < 8; thisNote++) 
      {
        int noteDuration = 1000 / noteDurations[thisNote];
        tone(6, melody[thisNote], noteDuration);
        int pauseBetweenNotes = noteDuration * 1.30;
        delay(pauseBetweenNotes);
        noTone(6);
      }
      delay(1000);
      if(abs(getPT100()-Temperature_DHT) < Tempthresh)
      {
        _resume = false;
      }
    }
  }
//  else if(IRMS>Viberthresh){
//    while(IRMS>Viberthresh){
//      lcd.setCursor(1,0);
//      lcd.print(F("OVER VIBRATION"));
//      lcd.setCursor(4,1);
//      lcd.print(F("DETECTED"));
//      delay(2500);
//    }
//  }
  else{
    lcd.setCursor(1,0);
    lcd.print(F("ALL PARAMETERS"));
    lcd.setCursor(5,1);
    lcd.print(F("ARE OK"));
  }
  
}
