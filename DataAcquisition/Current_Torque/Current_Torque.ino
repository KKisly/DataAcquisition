#include "HX711.h"
//#define DEBUG
#include <SPI.h>
#include <SD.h>

//Scale varibales
// HX711.DOUT  - pin #A1 A12
// HX711.PD_SCK - pin #A0 A13
HX711 scale(A12, A13);    // parameter "gain" is ommited; the default value 128 is used by the library

//Average Ampers Variables
const int numReadings = 30;
//motor A
float readingsA[numReadings];      // the readings from the analog input
int indexA = 0;                  // the index of the current reading
float totalA = 0;                  // the running total
float averageA = 0;                // the average
float currentValueA = 0;
//motor A
float readingsB[numReadings];      // the readings from the analog input
int indexB = 0;                  // the index of the current reading
float totalB = 0;                  // the running total
float averageB = 0;                // the average
float currentValueB = 0;

//Momentum readings Ampers Variables
int mVperAmp = 13.3;
int ACSoffset = 2500; // See offsets below
//motor A
int RawValueA = 0;
double VoltageA = 0;
double AmpsA = 0;
//motor B
int RawValueB = 0;
double VoltageB = 0;
double AmpsB = 0;

 //RPM VAriables, read RPM
 volatile int half_revolutions = 0;
 volatile int half_revolutions2 = 0;
 int rpm = 0;
 int rpm2 = 0;
 unsigned long lastmillis = 0;

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

File dataFile;

void setup()
{
 Serial.begin(9600);
 //RPM set-up
 attachInterrupt(digitalPinToInterrupt(19), rpm_fan, FALLING);
 attachInterrupt(digitalPinToInterrupt(20), rpm_fan, FALLING);
 //Current Set-up
 for (int thisReadingA = 0; thisReadingA < numReadings; thisReadingA++)
   readingsA[thisReadingA] = 0;
   //readingsB[thisReading] = 0;
    for (int thisReadingB = 0; thisReadingB < numReadings; thisReadingB++)
   readingsB[thisReadingB] = 0;

 //Scale set-up  
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

  scale.set_scale(6044.f);                      // this value is obtained by calibrating the scale with known weights; see the README for details
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
  {
   // see if the card is present and can be initialized:
  if (!SD.begin(10, 11, 12, 13)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1) ;
  }
  Serial.println("card initialized.");
  
  // Open up the file we're going to log to!
  dataFile = SD.open("datalog_A.txt", FILE_WRITE);
  if (! dataFile) {
    Serial.println("error opening datalog.txt");
    // Wait forever since we cant write data
    while (1) ;
  }
  }
  
}

// this code will be executed every time the interrupt 0 (pin2) gets low.
void rpm_fan(){
 half_revolutions++;
}

void loop()
{
     {
   //First motor A
   totalA = totalA - readingsA[indexA];          
   readingsA[indexA] = analogRead(A14); //Raw data reading
   readingsA[indexA] = (readingsA[indexA]-508)*4.9/1024/0.013-0.014;//Data processing:510-raw data from analogRead when the input is 0; 5-5v; the first 0.04-0.04V/A(sensitivity); the second 0.04-offset val;
   totalA = totalA + readingsA[indexA];       
   indexA = indexA + 1;                    
   if (indexA >= numReadings)              
     indexA = 0;                           
   averageA = totalA/numReadings;   //Smoothing algorithm from (http://www.arduino.cc/en/Tutorial/Smoothing)    
   currentValueA = averageA;
 //tft.print("  Read A:\t");  
 //tft.print(currentValueA);
 Serial.print("  Read A(a):\t");  
 Serial.print(currentValueA);
 //  delay(30);
     }
// Second motor - B
   {
   totalB = totalB - readingsB[indexB];          
   readingsB[indexB] = analogRead(A15); //Raw data reading
   readingsB[indexB] = (readingsB[indexB]-510)*4.9/1024/0.013-0.014;//Data processing:510-raw data from analogRead when the input is 0; 5-5v; the first 0.04-0.04V/A(sensitivity); the second 0.04-offset val;
   totalB = totalB + readingsB[indexB];       
   indexB = indexB + 1;                    
   if (indexB >= numReadings)              
     indexB = 0;                           
   averageB = totalB/numReadings;   //Smoothing algorithm from (http://www.arduino.cc/en/Tutorial/Smoothing)    
   currentValueB = averageB;
 //tft.print("  Read B:\t");  
 //tft.print(currentValueB);
 Serial.print("  Read A(b):\t");  
 Serial.print(currentValueB);
 Serial.print(" ");
 //  delay(30);
   
    //lcd.setCursor(0, 1);
//lcd.print(analogRead(A3));
//Serial.println( readVcc(), DEC );
 delay(100);
}
{
RawValueA = analogRead(A14);
 VoltageA = (RawValueA / 1023.0) * 5000; // Gets you mV
 AmpsA = ((VoltageA - ACSoffset) / mVperAmp);
 
 //tft.print("Raw Value = " ); // shows pre-scaled value 
 //tft.print(RawValue); 
 //tft.print("\t mV = "); // shows the voltage measured 
 //tft.print(Voltage,3); // the '3' after voltage allows you to display 3 digits after decimal point
 //tft.print("\t Amps = "); // shows the voltage measured 
 //tft.println(Amps,3); // the '3' after voltage allows you to display 3 digits after decimal point
 
 Serial.print("Raw Value A = " ); // shows pre-scaled value 
 Serial.print(RawValueA); 
 //Serial.print("\t mV A= "); // shows the voltage measured 
 //Serial.print(VoltageA,3); // the '3' after voltage allows you to display 3 digits after decimal point
 Serial.print("\t Amps A = "); // shows the voltage measured 
 Serial.print(AmpsA,3); // the '3' after voltage allows you to display 3 digits after decimal point
 delay(100);
}
{
RawValueB = analogRead(A15);
 VoltageB = (RawValueB / 1023.0) * 5000; // Gets you mV
 AmpsB = ((VoltageB - ACSoffset) / mVperAmp);
 
 //tft.print("Raw Value = " ); // shows pre-scaled value 
 //tft.print(RawValue); 
 //tft.print("\t mV = "); // shows the voltage measured 
 //tft.print(Voltage,3); // the '3' after voltage allows you to display 3 digits after decimal point
 //tft.print("\t Amps = "); // shows the voltage measured 
 //tft.println(Amps,3); // the '3' after voltage allows you to display 3 digits after decimal point
 
 Serial.print("Raw Value B = " ); // shows pre-scaled value 
 Serial.print(RawValueB); 
 //Serial.print("\t mV B = "); // shows the voltage measured 
 //Serial.print(VoltageB,3); // the '3' after voltage allows you to display 3 digits after decimal point
 Serial.print("\t Amps B = "); // shows the voltage measured 
 Serial.print(AmpsB,3); // the '3' after voltage allows you to display 3 digits after decimal point
 delay(100);
 
}
{
  Serial.print("one reading:\t");
  Serial.print(scale.get_units(), 1);
  Serial.print("\t| average:\t");
  Serial.println(scale.get_units(10), 1);

  scale.power_down(); // put the ADC in sleep mode
  delay(5000);
  scale.power_up();
}
{
  if (millis() - lastmillis >= 1000){ //Uptade every one second, this will be equal to reading frecuency (Hz).
detachInterrupt(digitalPinToInterrupt(19));//Disable interrupt when calculating
detachInterrupt(digitalPinToInterrupt(20));//Disable interrupt when calculating
 rpm = (half_revolutions * 120)/15; // Convert frecuency to RPM, note: this works for one interruption per full rotation. For two interrups per full rotation use half_revolutions * 30.
 rpm2 = (half_revolutions2 * 120)/15; // Convert frecuency to RPM, note: this works for one interruption per full rotation. For two interrups per full rotation use half_revolutions * 30.
 Serial.print("RPM_A =\t"); //print the word "RPM" and tab.
 Serial.print(rpm); // print the rpm value.
 Serial.print("\t Hz_A=\t"); //print the word "Hz".
 Serial.print(half_revolutions); //print revolutions per second or Hz. And print new line or enter.
 half_revolutions = 0; // Restart the RPM counter
  Serial.print("RPM_B =\t"); //print the word "RPM" and tab.
 Serial.print(rpm2); // print the rpm value.
 Serial.print("\t Hz_A=\t"); //print the word "Hz".
 Serial.print(half_revolutions); //print revolutions per second or Hz. And print new line or enter.
 half_revolutions = 0; // Restart the RPM counter
 lastmillis = millis(); // Uptade lasmillis
 //tft.println(lastmillis);
 attachInterrupt(digitalPinToInterrupt(19), rpm_fan, FALLING); //enable interrupt
  } 
}
  // make a string for assembling the data to log:
  String dataString = "";
  String dataString1 = "";

dataString += String(lastmillis);
dataString1 += String(rpm);

dataFile.print(dataString += ", ");
dataFile.println(dataString1);


  // print to the serial port too:
  Serial.print(dataString += ", ");
  Serial.println(dataString1);
  
  // The following line will 'save' the file to the SD card after every
  // line of data - this will use more power and slow down how much data
  // you can read but it's safer! 
  // If you want to speed up the system, remove the call to flush() and it
  // will save the file only every 512 bytes - every time a sector on the 
  // SD card is filled with data.
  dataFile.flush();
  
  // Take 1 measurement every 500 milliseconds
  delay(500);
}

// this code will be executed every time the interrupt 0 (pin2) gets low.
//void rpm_fan(){
 //half_revolutions++;
//}


