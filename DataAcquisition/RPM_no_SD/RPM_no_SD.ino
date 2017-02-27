

// This sketch has been Refurbished by BUHOSOFT
// Original code provided by Smoke And Wires
// http://www.smokeandwires.co.nz
// This code has been taken from the Adafruit TFT Library and modified
//  by us for use with our TFT Shields / Modules
// For original code / licensing please refer to
// https://github.com/adafruit/TFTLCD-Library

// adapted sketch by niq_ro from http://arduinotehniq.blogspot.com/
// ver. 1m5 - 13.11.2014, Craiova - Romania



// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
// #define LCD_CS A3 // Chip Select goes to Analog 3
// #define LCD_CD A2 // Command/Data goes to Analog 2
// #define LCD_WR A1 // LCD Write goes to Analog 1
// #define LCD_RD A0 // LCD Read goes to Analog 0

// #define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

//#define DEBUG
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <TouchScreen.h>


// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
// optional
#define LCD_RESET A4


#define YP A1  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 7   // can be a digital pin
#define XP 6   // can be a digital pin

#define TS_MINX 150
#define TS_MINY 120
#define TS_MAXX 920
#define TS_MAXY 940

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
//#define ROZ     0xFD20
#define ROZ     0xFBE0
#define GRI     0xBDF7
// http://stackoverflow.com/questions/13720937/c-defined-16bit-high-color
// http://wiibrew.org/wiki/U16_colors

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);;

#define BOXSIZE 40
#define PENRADIUS 3
int oldcolor, currentcolor;
int ics;
const int chipSelect = 4;
int RawValue= 0;
double Voltage = 0;
double Amps = 0;
int mVperAmp = 40;
int ACSoffset = 2500;
// read RPM
 int half_revolutions = 0;
 int rpm = 0;
 unsigned long lastmillis = 0;
 // this code will be executed every time the interrupt 0 (pin2) gets low.
 
void rpm_fan(){
  half_revolutions++;
}
 

void setup(void) {
#ifdef DEBUG
  Serial.begin(9600);
  Serial.println(F("Paint!"));
   //attachInterrupt(0, rpm_fan, FALLING);
#endif // DEBUG
  tft.reset();

  uint16_t identifier = tft.readID();

if(identifier == 0x9325) {
#ifdef DEBUG
    Serial.println(F("Found ILI9325 LCD driver"));
  } else if(identifier == 0x9328) {

    Serial.println(F("Found ILI9328 LCD driver"));
  } else if(identifier == 0x7575) {

    Serial.println(F("Found HX8347G LCD driver"));
  } else if(identifier == 0x9341) {

    Serial.println(F("Found ILI9341 LCD driver"));
  } else if(identifier == 0x8357) {

    Serial.println(F("Found HX8357D LCD driver"));
#endif // DEBUG
    } else {
    #ifdef DEBUG
    Serial.print(F("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
    Serial.print(F("I try use ILI9341 LCD driver "));
    Serial.println(F("If using the Adafruit 2.8\" TFT Arduino shield, the line:"));
    Serial.println(F("  #define USE_ADAFRUIT_SHIELD_PINOUT"));
    Serial.println(F("should appear in the library header (Adafruit_TFT.h)."));
    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
    Serial.println(F("Also if using the breakout, double-check that all wiring"));
    Serial.println(F("matches the tutorial."));
    #endif // DEBUG
    identifier = 0x9341;
  }
  tft.begin(identifier);

  tft.fillScreen(BLACK);
  tft.fillRect(0, 0, 320, 240, BLACK);
  tft.setRotation(3);
  tft.setCursor(30, 10);
  tft.setTextColor(RED);  tft.setTextSize(1);
  tft.println("LCD driver chip: ");
  tft.setCursor(100, 30);
  tft.setTextColor(GREEN);
  tft.println(identifier, HEX);
  
 Serial.begin(9600); 
attachInterrupt(17, rpm_fan, FALLING);
}

void loop() {
  {
  if (millis() - lastmillis >= 1000){ //Uptade every one second, this will be equal to reading frecuency (Hz).
detachInterrupt(17);//Disable interrupt when calculating
 rpm = half_revolutions * 60; // Convert frecuency to RPM, note: this works for one interruption per full rotation. For two interrups per full rotation use half_revolutions * 30.
 
 tft.print("RPM =\t"); //print the word "RPM" and tab.
 tft.print(rpm); // print the rpm value.
 tft.print("\t Hz=\t"); //print the word "Hz".
 tft.println(half_revolutions); //print revolutions per second or Hz. And print new line or enter.
 half_revolutions = 0; // Restart the RPM counter
 lastmillis = millis(); // Uptade lasmillis
 tft.println(lastmillis);
 attachInterrupt(17, rpm_fan, FALLING); //enable interrupt
  } 
  }
  //tft.print(rpm);
  {
  tft.print("Time: ");
  lastmillis = millis();
  //prints time since program started
  tft.println(lastmillis);
  // wait a second so as not to send massive amounts of data
  delay(1000);
}
}


