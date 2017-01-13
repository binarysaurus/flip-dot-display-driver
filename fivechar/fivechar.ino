#include <stdint.h>
#include "SparkFunBME280.h"
#include "Wire.h"
#include "SPI.h"
#include <math.h>
BME280 mySensor;
int incomingByte = 0;
float pressure = 0;
int count = 0;
int flag = 0;
int brightness;
const byte alphabet[127][5] = {
    {},{},{},{},{},{},{},{},
    {},{},{},{},{},{},{},{},
    {},{},{},{},{},{},{},{},
    {},{},{},{},{},{},{},{},
    {0x00,0x00,0x00,0x00,0x00}, //   0x20 32
    {0x00,0x00,0x6f,0x00,0x00}, // ! 0x21 33
    {0x00,0x07,0x00,0x07,0x00}, // " 0x22 34
    {0x14,0x7f,0x14,0x7f,0x14}, // # 0x23 35
    {0x24,0x2a,0xff,0x2a,0x12}, // $ 0x24 36
    {0x23,0x13,0x08,0x64,0x62}, // % 0x25 37
    {0x36,0x49,0x56,0x20,0x50}, // & 0x26 38
    {0x00,0x00,0x07,0x00,0x00}, // ' 0x27 39
    {0x00,0x1c,0x22,0x41,0x00}, // ( 0x28 40
    {0x00,0x41,0x22,0x1c,0x00}, // ) 0x29 41
    {0x14,0x08,0x3e,0x08,0x14}, // * 0x2a 42
    {0x08,0x08,0x3e,0x08,0x08}, // + 0x2b 43
    {0x00,0x00,0x00,0x00,0x00}, // , 0x2c 44 ////  Changed to clear display command
    {0x08,0x08,0x08,0x08,0x08}, // - 0x2d 45
    {0x00,0x60,0x60,0x00,0x00}, // . 0x2e 46
    {0x20,0x10,0x08,0x04,0x02}, // / 0x2f 47
    {0x3e,0x51,0x49,0x45,0x3e}, // 0 0x30 48
    {0x00,0x42,0x7f,0x40,0x00}, // 1 0x31 49
    {0x42,0x61,0x51,0x49,0x46}, // 2 0x32 50
    {0x21,0x41,0x45,0x4b,0x31}, // 3 0x33 51
    {0x18,0x14,0x12,0x7f,0x10}, // 4 0x34 52
    {0x27,0x45,0x45,0x45,0x39}, // 5 0x35 53
    {0x3c,0x4a,0x49,0x49,0x30}, // 6 0x36 54
    {0x01,0x71,0x09,0x05,0x03}, // 7 0x37 55
    {0x36,0x49,0x49,0x49,0x36}, // 8 0x38 56
    {0x06,0x49,0x49,0x29,0x1e}, // 9 0x39 57
    {0x00,0x00,0x36,0x00,0x00}, // : 0x3a 58
    {0x00,0x56,0x36,0x00,0x00}, // ; 0x3b 59
    {0x08,0x14,0x22,0x41,0x00}, // < 0x3c 60
    {0x14,0x14,0x14,0x14,0x14}, // = 0x3d 61
    {0x00,0x41,0x22,0x14,0x08}, // > 0x3e 62
    {0x02,0x01,0x51,0x09,0x06}, // ? 0x3f 63
    {0x23,0x13,0x08,0x64,0x62}, // @ 0x40 64 /// changed to % - original = {0x3e,0x41,0x5d,0x49,0x4e}
    {0x7e,0x09,0x09,0x09,0x7e}, // A 0x41 65
    {0x7f,0x49,0x49,0x49,0x36}, // B 0x42 66
    {0x3e,0x41,0x41,0x41,0x22}, // C 0x43 67 /// alternitave font is  {0x7f,0x41,0x41,0x41,0x63}
    {0x7f,0x41,0x41,0x41,0x3e}, // D 0x44 68
    {0x7f,0x49,0x49,0x49,0x41}, // E 0x45 69
    {0x7f,0x09,0x09,0x09,0x01}, // F 0x46 70
    {0x3e,0x41,0x49,0x49,0x7a}, // G 0x47 71
    {0x7f,0x08,0x08,0x08,0x7f}, // H 0x48 72
    {0x41,0x41,0x7f,0x41,0x41}, // I 0x49 73
    {0x20,0x40,0x41,0x3f,0x01}, // J 0x4a 74
    {0x7f,0x08,0x14,0x22,0x41}, // K 0x4b 75
    {0x7f,0x40,0x40,0x40,0x40}, // L 0x4c 76
    {0x7f,0x02,0x0c,0x02,0x7f}, // M 0x4d 77
    {0x7f,0x04,0x08,0x10,0x7f}, // N 0x4e 78
    {0x3e,0x41,0x41,0x41,0x3e}, // O 0x4f 79
    {0x7f,0x09,0x09,0x09,0x06}, // P 0x50 80
    {0x3e,0x41,0x51,0x21,0x5e}, // Q 0x51 81
    {0x7f,0x09,0x19,0x29,0x46}, // R 0x52 82
    {0x46,0x49,0x49,0x49,0x31}, // S 0x53 83
    {0x01,0x01,0x7f,0x01,0x01}, // T 0x54 84
    {0x3f,0x40,0x40,0x40,0x3f}, // U 0x55 85
    {0x0f,0x30,0x40,0x30,0x0f}, // V 0x56 86
    {0x3f,0x40,0x30,0x40,0x3f}, // W 0x57 87
    {0x63,0x14,0x08,0x14,0x63}, // X 0x58 88
    {0x07,0x08,0x70,0x08,0x07}, // Y 0x59 89
    {0x61,0x51,0x49,0x45,0x43}, // Z 0x5a 90
    {0x3c,0x4a,0x49,0x29,0x1e}, // [ 0x5b 91 //// Changed to theta
    {0x02,0x04,0x08,0x10,0x20}, // \ 0x5c 92
    {0x00,0x41,0x7f,0x00,0x00}, // ] 0x5d 93
    {0x04,0x02,0x01,0x02,0x04}, // ^ 0x5e 94
    {0x40,0x40,0x40,0x40,0x40}, // _ 0x5f 95
    {0x07,0x05,0x07,0x00,0x00}, // ` 0x60 96 ///// Reworked as degree symbol for temperature display (was: {0x00,0x00,0x03,0x04,0x00})
    {0x20,0x54,0x54,0x54,0x78}, // a 0x61 97
    {0x7f,0x48,0x44,0x44,0x38}, // b 0x62 98
    {0x38,0x44,0x44,0x44,0x20}, // c 0x63 99
    {0x38,0x44,0x44,0x48,0x7f}, // d 0x64 100
    {0x38,0x54,0x54,0x54,0x18}, // e 0x65 101
    {0x08,0x7e,0x09,0x01,0x02}, // f 0x66 102
    {0x0c,0x52,0x52,0x52,0x3e}, // g 0x67 103
    {0x7f,0x08,0x04,0x04,0x78}, // h 0x68 104
    {0x00,0x44,0x7d,0x40,0x00}, // i 0x69 105
    {0x20,0x40,0x44,0x3d,0x00}, // j 0x6a 106
    {0x00,0x7f,0x10,0x28,0x44}, // k 0x6b 107
    {0x00,0x41,0x7f,0x40,0x00}, // l 0x6c 108
    {0x7c,0x04,0x18,0x04,0x78}, // m 0x6d 109
    {0x7c,0x08,0x04,0x04,0x78}, // n 0x6e 110
    {0x38,0x44,0x44,0x44,0x38}, // o 0x6f 111
    {0x7c,0x14,0x14,0x14,0x08}, // p 0x70 112
    {0x08,0x14,0x14,0x18,0x7c}, // q 0x71 113
    {0x7c,0x08,0x04,0x04,0x08}, // r 0x72 114
    {0x48,0x54,0x54,0x54,0x20}, // s 0x73 115
    {0x04,0x3f,0x44,0x40,0x20}, // t 0x74 116
    {0x3c,0x40,0x40,0x20,0x7c}, // u 0x75 117
    {0x1c,0x20,0x40,0x20,0x1c}, // v 0x76 118
    {0x3c,0x40,0x30,0x40,0x3c}, // w 0x77 119
    {0x44,0x28,0x10,0x28,0x44}, // x 0x78 120
    {0x0c,0x50,0x50,0x50,0x3c}, // y 0x79 121
    {0x44,0x64,0x54,0x4c,0x44}, // z 0x7a 122
    {0x00,0x08,0x36,0x41,0x41}, // { 0x7b 123
    {0x00,0x00,0x7f,0x00,0x00}, // | 0x7c 124
    {0x41,0x41,0x36,0x08,0x00}, // } 0x7d 125
    {0xff,0xff,0xff,0xff,0xff}, // ~ 0x7e 126 ///// {0x04,0x02,0x04,0x08,0x04}
};

void setup() {
 pinMode(11, OUTPUT); //reset (green)
 pinMode(12, OUTPUT); //clock (black)
 pinMode(13, OUTPUT); //select (red)
 pinMode(3, INPUT);   //light detector
 pinMode(6, OUTPUT);  //LED lights
 attachInterrupt(digitalPinToInterrupt(3), dark, CHANGE);   //set up interrupt, calls dark when pin 3 is low

//***********************************************//Operation settings for BME280
   mySensor.settings.commInterface = I2C_MODE;
   mySensor.settings.I2CAddress = 0x76;
   mySensor.settings.runMode = 3;   //continious sample mode
   mySensor.settings.tStandby = 4;  //500ms update rate
   mySensor.settings.filter = 3;
   mySensor.settings.tempOverSample = 1;
   mySensor.settings.pressOverSample = 2;
   mySensor.settings.humidOverSample = 1;
   delay(10);          //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
   Serial.begin(9600); //96k Baud

//***********************************************//disables the internal pullup resistors for the I2C
   Wire.begin();
   #ifndef cbi
   #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
   #endif
   #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
   cbi(PORTC, 4);
   cbi(PORTC, 5);
   #else
   cbi(PORTD, 0);
   cbi(PORTD, 1);
   #endif
//***********************************************//

   Serial.print("Starting BME280... result of .begin(): 0x");
   Serial.println(mySensor.begin(), HEX);   //starts sensor up
   printdd("          ");                   //clears display
}

void loop() {
 int Temp = 1;   //defines values to pass to sensor print function to select temp, humidity, or pressure
 int Humidity = 2;
 int Pressure = 3;
 if (digitalRead(3) == 0)
   brightness = 1;
 else if (digitalRead(3) == 1)
   brightness = 120;
 analogWrite(6, brightness);

//***********************************************//temp loop (3 min)
 for (int i=0; i<9; i++) {
//    analogWrite(6, brightness);
   flag = 0;                //reset interrupt flag
   printSensor(Temp);       //prints the temp on display
   while (count < 10000) {  //20 sec delay loop
     count = count + 1;     //inc counter by 1
     delay(2);              //delay 2ms
   }
   count = 0;  //reset counter after the loop has finished
 }

//***********************************************//humidity loop (2 min)
 for (int j=0; j<6; j++) {
 //  analogWrite(6, brightness);
   flag = 0;                //reset interrupt flag
   printSensor(Humidity);   //prints the humidity on display
   while (count < 10000) {  //20 sec delay loop
     count = count + 1;     //inc counter by 1
     delay(2);              //delay 2ms
   }
   count = 0;   //reset counter after the loop has finished
 }

//***********************************************//pressure loop (2 min)
 for (int k=0; k<6; k++) {
//    analogWrite(6, brightness);
   flag = 0;                //reset interrupt flag
   printSensor(Pressure);   //prints the pressure on display
   while (count < 10000) {  //20 sec delay loop
     count = count + 1;     //inc counter by 1
     delay(2);              //delay 2ms
   }
   count = 0;   //reset counter after the loop has finished
 }
}

void printSensor(int type) {     //reads requested sensor value and converts to char array
 int previous = incomingByte;   //record previous value
 char * buffer;                 //pointer that points to array of char called buffer
 buffer = (char *)malloc(10*sizeof(char));   //allocates memory for buffer

//***********************************************//Tempature
 if (type == 1) {
   incomingByte = round((int)mySensor.readTempF());   //rounds temp value and stores it
   if (incomingByte != previous) {                    //prevents unnecessary display updates
     sprintf(buffer, "Temp %d`F", incomingByte);      //converts incomingByte + `F to char array
     printdd(buffer);                                 //prints array to display
   }
 }

//***********************************************//Humidity
 else if (type == 2) {
   incomingByte = mySensor.readFloatHumidity();   //reads humidity value
   if (incomingByte != previous) {                //prevents unnecessary display updates
     sprintf(buffer, " %d@ Humid", incomingByte); //converts incomingByte + % to char array (@ represents % in this case)
     printdd(buffer);                             //prints array to display
   }
 }

//***********************************************//Pressure
 else if (type == 3) {
   float prevPressure = floorf(pressure*100)/100;   //records previous value
   char partial[6];                 //array to stor the pressure reading in not including the lable
   Serial.print("Tempature: ");
   Serial.println(mySensor.readTempF());   //for some reason must read temp first for the pressre to be accurate
   pressure = floorf(((mySensor.readFloatPressure())/3386.375258)*100)/100; //convert from pascal to inHg
   Serial.print(prevPressure);
   Serial.print("     ");
   Serial.println(pressure);
   if (pressure != prevPressure) {        //keeps display from updating when there is no change in value
     dtostrf(pressure, 5, 2, partial);    //stores float pressure in char array partial (5 = size of message, 2 = prescision)
     sprintf(buffer, "%s inHg", partial); //stores partial + inHg in buffer
     printdd(buffer);                     //calls printdd function
   }
 }
 free(buffer);   //frees memory allocated to buffer
}

void printdd(char * str){           //pulls correct vaule of alphabet array
 byte letter;                      //stores 8 bit pattern for each columb of the character being printed
 for (int i = 0; i < 10; i++){     //runs 10 times, once for each of the 10 digits
   for (int j = 0; j < 5; j++){    //runs 5 times, once for each of the 5 columbs in a digit
     letter = alphabet[str[i]][j]; //pulls array value
     writeletter(letter);          //sends value to be printed
   }
 }
 digitalWrite(11, HIGH);   //reset cursor after message
 delayMicroseconds(200);
 digitalWrite(11,LOW);
}

void writeletter(byte letter) { // converts hex number to binary and writes value
 boolean value;
 byte shift = letter << 1;     // display requires MSB be a 0
 for (int n=0; n<8; n++) {     // repeats 8 times for 8 bit message
   value = bitRead(shift,n);   // reads each bit out separately
   digitalWrite(13, value);    // 1 sets dot to yellow, 0 sets black
   digitalWrite(12, HIGH);     // clock in selection
   delayMicroseconds(2500);    // delay so dot can flip
   digitalWrite(12, LOW);      // set clock low
 }
   digitalWrite(13, LOW);      // reset "select" to low
}

void dark() {        //interrupt that blanks display at night
 if (digitalRead(3) == 0)
   brightness = 1;
 else if (digitalRead(3) == 1)
   brightness = 120;
 analogWrite(6, brightness);
 /*
 incomingByte = 0;  //resets incomingByte so display will update after interrupt
 count = 7500;      //sets counter for a 5 sec delay
   if (flag == 0) { //blanks display once
   printdd("          ");   //blanks display
   flag = 1;                //indicates display has been blanked
   }
 */
}

/*
void texastech(int x){
   printdd("Texas Tech");
   delay(x);
   printdd("exas Tech ");
   delay(x);
   printdd("xas Tech  ");
   delay(x);
   printdd("as Tech  T");
   delay(x);
   printdd("s Tech  Te");
   delay(x);
   printdd(" Tech  Tex");
   delay(x);
   printdd("Tech  Texa");
   delay(x);
   printdd("ech  Texas");
   delay(x);
   printdd("ch  Texas ");
   delay(x);
   printdd("h  Texas T");
   delay(x);
   printdd("  Texas Te");
   delay(x);
   printdd(" Texas Tec");
   delay(x);
}
*/