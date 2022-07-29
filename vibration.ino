
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ADS;


int voltage = 0;



void setup() 
{
  Serial.begin(9600);
  ADS.setGain(GAIN_TWO);
  ADS.begin();
}


void loop() {
  voltage = ADS.readADC_Differential_2_3();
  voltage = abs(voltage);
  Serial.println(voltage);
//  if(voltage < 0){
//    voltage = (voltage * -1);
//  }
  delay(100);
}



//
//float Irms(){
//  float voltage, Curr_temp;
//  float Curr_sum = 0;
//  long t_start = millis();
//  uint8_t counter = 0;
//  while (millis() - t_start < 1000){
//    voltage = ADS.readADC_Differential_0_1() * multiplier;
//    Curr_temp = voltage * FACTOR;
//    Curr_temp /= 1000.0;
//    Curr_sum += sq(Curr_temp);
//    counter = counter +1;
//    delay(3);
//  }
//  Serial.print("Samples:");
//  Serial.println(counter);
//  return(sqrt(Curr_sum / counter));
//}
