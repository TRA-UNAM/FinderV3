/*
AN128_ardunio_cozir CO2 Demonstration code 11/22/2016 Runs on Ardunio
UNO, MEGA or MEGA2560
Written by: Marv Kausch 11/22/2016 or Co2meter.com
revised 2/24/17 to change comment pin numbers to 12 and 13, below
This sketch connects will connect to a COZIR sensor
and report readings back to the host computer over USB. The value
is
stored in a global variable 'co2' and can be used for any number of
applications.
pin connections:
Arduino________COZIR Sensor
GND ------------------ 1 (gnd)
3.3v------------------- 3 (Vcc)
12 -------------------- 5 (Rx)
13 -------------------- 7 (Tx)
*/

int co2 =0;
double multiplier = 10;// 1 for 2% =20000 PPM, 10 for 20% = 200,000 PPM
uint8_t buffer[25];
uint8_t ind =0;
uint8_t indice =0;
int fill_buffer(); // function prototypes here
int format_output();
void setup() {
Serial.begin(9600);
Serial.print("\n\n");
Serial.println("AN128_ardunio_cozir CO2Demonstration code 11/22/2016\n\n");
Serial2.begin(9600); // Start serial communications with sensor
Serial2.println("K 0"); // Set Command mode

Serial2.println("M 6"); // send Mode for Z and z outputs
// "Z xxxxx z xxxxx" (CO2 filtered and unfiltered)
Serial2.println("K 1");
// set streaming mode
}
void loop() {
fill_buffer();
//buffer
// function call that reacds CO2 sensor and fills
Serial.print("Buffer contains: ");
for(int j=0; j<ind; j++)Serial.print(buffer[j],HEX);
indice = 0;
format_output();
Serial.print(" Raw PPM");
indice = 8; // In ASCII buffer, filtered value is offset from raw by 8 bytes
format_output();
Serial.println(" Filtered PPM\n\n");
}
int fill_buffer(void){
// Fill buffer with sensor ascii data
ind = 0;
while(buffer[ind-1] != 0x0A){ // Read sensor and fill buffer up to0XA = CR
if(Serial2.available()>0){
buffer[ind] = Serial2.read();
ind++;
}
}
// buffer() now filled with sensor ascii data
// ind contains the number of characters loaded into buffer up to 0xA = CR
ind = ind -2; // decrement buffer to exactly match last numericalcharacter
}

int format_output(void){ // read buffer, extract 6 ASCII chars,convert to PPM and print
co2 = buffer[15-indice]-0x30;
co2 = co2+((buffer[14-indice]-0x30)*10);
co2 +=(buffer[13-indice]-0x30)*100;
co2 +=(buffer[12-indice]-0x30)*1000;
co2 +=(buffer[11-indice]-0x30)*10000;
Serial.print("\n CO2 = ");
Serial.print(co2*multiplier,0);
// Serial.print(" PPM,");
//
Serial.print("\n");
delay(200);
}
