/*
Henry's Bench
ACS758 Current Measurement Tutorial
*/
const int analogIn = A13;

// Set your scale factor
int mVperAmp = 13.3; // See Scale Factors Below

/* Scale Factors
50A bi-directional = 40
50A uni-directional = 60
100A bi-directional = 20
100A uni-directional = 40
150A bi-directional = 13.3
150A uni-directioal = 26.7
200A bi-directional = 10
200A uni-directional = 20
*/

// Set you Offset
int ACSoffset = 2500; // See offsets below

/* Offsets
If bi-directional = 2500
If uni- directional = 600
*/

int RawValue= 0;
double Voltage = 0;
double Amps = 0;

void setup(){ 
 Serial.begin(9600);
}

void loop(){
 
 RawValue = analogRead(analogIn);
 Voltage = (RawValue / 1023.0) * 5000; // Gets you mV
 Amps = ((Voltage - ACSoffset) / mVperAmp);
 
 Serial.print("Raw Value = " ); // shows pre-scaled value 
 Serial.print(RawValue); 
 Serial.print("\t mV = "); // shows the voltage measured 
 Serial.print(Voltage,3); // the '3' after voltage allows you to display 3 digits after decimal point
 Serial.print("\t Amps = "); // shows the voltage measured 
 Serial.println(Amps,3); // the '3' after voltage allows you to display 3 digits after decimal point
 delay(2500); 
 
}
