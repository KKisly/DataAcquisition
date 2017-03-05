/*Constantine Kisly
ACS758 Current, Torgue and RPM Measurement Script for Data acqusition station
*/
#include "HX711.h"
//#define DEBUG
//#include <SPI.h>
//#include <SD.h>

//Scale varibales
//HX711.DOUT  - pin #A1 A12 - pins connection order
//HX711.PD_SCK - pin #A0 A13 - - pins connection order
HX711 scale(A12, A13);    // parameter "gain" is ommited; the default value 128 is used by the library

//Average Ampers Variables
//_------------------------------------------------------
const int numReadings = 10;
//motor A
float readingsA[numReadings];      // the readings from the analog input
int indexA = 0;                  // the index of the current reading
float totalA = 0;                  // the running total
float averageA = 0;                // the average
float currentValueA = 0;
//motor B
float readingsB[numReadings];      // the readings from the analog input
int indexB = 0;                  // the index of the current reading
float totalB = 0;                  // the running total
float averageB = 0;                // the average
float currentValueB = 0;

//Momentum readings Ampers Variables
// ----------------------------------------
// currently only partially in use
int mVperAmp = 13.3;
int ACSoffset = 2500; // for our sensor offset is 2.5V
//motor A
int RawValueA = 0;
double VoltageA = 0;
double AmpsA = 0;
//motor B
int RawValueB = 0;
double VoltageB = 0;
double AmpsB = 0;

 //RPM VAriables, read RPM
 //-----------------------------------------
 volatile int half_revolutions = 0;
 volatile int half_revolutions2 = 0;
 int rpm = 0;
 int rpm2 = 0;
 unsigned long lastmillis = 0;

// set up variables using the SD utility library functions:
//Sd2Card card;
//SdVolume volume;
//SdFile root;

//File dataFile;

void setup()
{
 Serial.begin(38400);
 //RPM set-up
 //******************************************************
 attachInterrupt(digitalPinToInterrupt(19), rpm_fan, FALLING);
 attachInterrupt(digitalPinToInterrupt(20), rpm_fan2, FALLING);
 //Current Set-up
 /**
 for (int thisReadingA = 0; thisReadingA < numReadings; thisReadingA++)
  { readingsA[thisReadingA] = 0;}
   //readingsB[thisReading] = 0;
    for (int thisReadingB = 0; thisReadingB < numReadings; thisReadingB++)
   {readingsB[thisReadingB] = 0;}
   */

 //Scale set-up
 //*****************************************************************  
  Serial.println("HX711 Demo");

  Serial.println("Before setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(scale.read());     // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));   // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight (not set yet)

  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided 
            // by the SCALE parameter (not set yet)  

  scale.set_scale(5995.f);                      // this value is obtained by calibrating the scale with known weights; see the README for details. 6044
  scale.tare();               // reset the scale to 0

  Serial.println("After setting up the scale:");

  Serial.print("read: \t\t");
  Serial.println(scale.read());                 // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));       // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight, set with tare()

  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare weight, divided 
            // by the SCALE parameter set with set_scale

  Serial.println("Readings:");
  //SD card Set-up ************************************************************************************************************* 
}

void loop(){
// Ampers measurements************************************
     {
  //First motor A
  //for (int thisReadingA = 0; thisReadingA < numReadings; thisReadingA++)
  //{ readingsA[thisReadingA] = 0;}
   totalA = totalA - readingsA[indexA];
   RawValueA = analogRead(A14);          
   readingsA[indexA] = analogRead(A14); //Raw data reading
   readingsA[indexA] = (readingsA[indexA]-515)*5.0/1024/0.013-0.014;//Data processing:510-raw data from analogRead when the input is 0; 5-5v; the first 0.04-0.04V/A(sensitivity); the second 0.04-offset val; 508.96
   totalA = totalA + readingsA[indexA];       
   indexA = indexA + 1;                    
   if (indexA >= numReadings)              
     indexA = 0;                           
   averageA = totalA/numReadings;   //Smoothing algorithm from (http://www.arduino.cc/en/Tutorial/Smoothing)    
   currentValueA = averageA;
 //tft.print("  Read A:\t");  
 //tft.print(currentValueA);
 //Serial.print(" Read A(a):\t");
 Serial.print(currentValueA);
 Serial.print(",  ");
 //Serial.print(RawValueA + ",");
 //Serial.print(",");
 //delay(10);
     }
// Second motor - B
   {
   //for (int thisReadingB = 0; thisReadingB < numReadings; thisReadingB++)
   //{readingsB[thisReadingB] = 0;}
   totalB = totalB - readingsB[indexB];          
   readingsB[indexB] = analogRead(A15); //Raw data reading
   readingsB[indexB] = (readingsB[indexB]-517)*5.0/1024/0.013-0.014;//Data processing:510-raw data from analogRead when the input is 0; 5-5v; the first 0.04-0.04V/A(sensitivity); the second 0.04-offset val;
   totalB = totalB + readingsB[indexB];       
   indexB = indexB + 1;                    
   if (indexB >= numReadings)              
     indexB = 0;                           
   averageB = totalB/numReadings;   //Smoothing algorithm from (http://www.arduino.cc/en/Tutorial/Smoothing)    
   currentValueB = averageB;
 //tft.print("  Read B:\t");  
 //tft.print(currentValueB);
 //Serial.print(" Read A(b):\t");  
 Serial.print(currentValueB);
 Serial.print(",  ");
 //Serial.print(RawValueB + ",");
 //Serial.print(",");
 //delay(10);
   }
  //Scale Measurements*********************************************************************************************
{
  //Serial.print("one reading:\t");
  Serial.print(scale.get_units(), 1);
  Serial.print(",     ");
  //Serial.print("\t| average:\t");
  Serial.print(scale.get_units(5), 1);
  Serial.print(",   ");

  //scale.power_down(); // put the ADC in sleep mode
  //delay(10);
  //scale.power_up();
}
//delay(450); // Is it the right place for delay?
// RPM Measurements ********************************************************************************************
{
  if (millis() - lastmillis >= 500){ //Uptade every one second, this will be equal to reading frecuency (Hz).
detachInterrupt(digitalPinToInterrupt(19));//Disable interrupt when calculating
detachInterrupt(digitalPinToInterrupt(20));//Disable interrupt when calculating
 rpm = (half_revolutions * 200)/12; // Convert frecuency to RPM, note: this works for one interruption per full rotation. For two interrups per full rotation use half_revolutions * 30. I don't remember why I used 120 Should be 30 sec * 2. and 13 - is the number of motor poles.
 rpm2 = (half_revolutions2 * 200)/12; // Convert frecuency to RPM, note: this works for one interruption per full rotation. For two interrups per full rotation use half_revolutions * 30.
 //Serial.print("RPM_A =\t"); //print the word "RPM" and tab.
 Serial.print(rpm); // print the rpm value.
 Serial.print(",    ");
 //Serial.print("\t Hz_A=\t"); //print the word "Hz".
 //Serial.print(half_revolutions); //print revolutions per second or Hz. And print new line or enter.
 half_revolutions = 0; // Restart the RPM counter
 //Serial.print("RPM_B =\t"); //print the word "RPM" and tab.
 Serial.println(rpm2); // print the rpm value.
 //Serial.print("\t Hz_A=\t"); //print the word "Hz".
 //Serial.println(half_revolutions2); //print revolutions per second or Hz. And print new line or enter.
 half_revolutions2 = 0; // Restart the RPM counter
 lastmillis = millis(); // Uptade lasmillis
 //tft.println(lastmillis);
 attachInterrupt(digitalPinToInterrupt(19), rpm_fan, FALLING); //enateble interrupt
 attachInterrupt(digitalPinToInterrupt(20), rpm_fan2, FALLING); //enable interrupt
 } 
}
}
// Motor A. this code will be executed every time the interrupt 0 (pin2) gets low.
void rpm_fan(){
 half_revolutions++;
}

// Motor B. this code will be executed every time the interrupt 0 (pin2) gets low.
void rpm_fan2(){
 half_revolutions2++;
}


