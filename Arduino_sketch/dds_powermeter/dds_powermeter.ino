//AD9850 DDS and AD8307 power meter
// sept 2013 RH

#include "FastPins.h"

#define DDS_CLOCK 125000000

// Pin connections for DDS
#define  DDSPort PORTB
#define  DDSioReg DDRB
#define  RESET  PB0    // PB0  Pin8
#define  LOAD   PB1    // PB1  Pin9
#define  CLOCK  PB2    // PB2  Pin10
#define  DATA   PB3    // PB3  Pin11

int AD8307 = A1;    // select the analog input pin for AD8307

#define arduinoLED 13   // Arduino LED on board
#define LED PB5

char inputcmd[100];  // serial data input
int cmdindex=0;
  
void setup()
{
  pinMode(arduinoLED, OUTPUT);      // Configure the onboard LED for output
  digitalWrite(arduinoLED, LOW);    // default to LED off
  analogReference(EXTERNAL);  // 3.3v regulator output used as reference - 3.38V on this NANO board
  Serial.begin(57600);
  
 // init dds 
  AD9850_init();
  AD9850_reset();

  SetFrequency(1000000); // 1 mhz default

}

void loop()   // Arduino superloop - where everything gets done
{

  char ch;

  long int temp;
  
// serial command interpreter 
// enter a number to set the frequency, anything else shows power
  while (Serial.available()) {
    ch=(char)Serial.read();
    if (((ch >= '0') && (ch <= '9')) || ((ch >= 'A') && (ch <= 'Z'))) inputcmd[cmdindex++]=ch; // ignore everything other than numbers and uppercase letters, save to command string
    if (ch == '\n') {    // parse command if its a newline
      inputcmd[cmdindex]=0; // terminate the string
      if ((temp=atol(inputcmd)) > 0) {
        SetFrequency(temp);
      }
      else
      {
        dBm_power();
      }
      //else raw_power();  // python has trouble with floats
      cmdindex=0; // reset command line      
    }
  }
}

// AD8307 outputs 25mv per db. 0db is 2.1v out
// if we use a 2.56v ref this works out to
// 2.5mv per bit, or 0.1dbm
// the NANO has only 1.1v internal ref or the 5V supply so I used the 3.3v regulator as a ref
// with the 3.38V ref the output will be 3.38V/.025V/1024 or 0.13203125db/bit

void dBm_power() {
digitalFastWrite (DDSPort, LED, HIGH);
  float dBm=0;
  int i;
  for (i=0;i<10;++i)  dBm+=(float)analogRead(AD8307); // average a few raw readings
  dBm=dBm/i;
//  0db = 2.1v = 636 with 3.38V ref
// -74dbm = .25v, 1dbm = .025v. if the AD8307 has an offset error we correct that here as well
   dBm = dBm - 679;
   dBm = dBm * 0.12891 ;    
digitalFastWrite (DDSPort, LED, LOW);
  Serial.print(" "); // may help Python parser
  Serial.print(dBm);
  Serial.println(" "); // may help Python parser
}

void raw_power() {
  Serial.print(" "); // may help Python parser
  Serial.print(analogRead(AD8307));
  Serial.println(" "); // may help Python parser
}

void SetFrequency(unsigned long frequency)
{
  unsigned long tuning_word;
  float tuneword=(frequency * pow(2, 32)) / (unsigned long) DDS_CLOCK; // hack to avoid overflow on top term
  tuning_word=(unsigned long) tuneword;
  digitalFastWrite (DDSPort, LOAD, LOW); 
/*  
  shiftOut(DATA, CLOCK, MSBFIRST, 0x0);
  shiftOut(DATA, CLOCK, MSBFIRST, tuning_word >> 24);
  shiftOut(DATA, CLOCK, MSBFIRST, tuning_word >> 16);
  shiftOut(DATA, CLOCK, MSBFIRST, tuning_word >> 8);
  shiftOut(DATA, CLOCK, MSBFIRST, tuning_word);
*/
 
  shiftFastOut(&DDSPort, DATA, CLOCK, LSBFIRST, tuning_word);
  shiftFastOut(&DDSPort, DATA, CLOCK, LSBFIRST, tuning_word >> 8);
  shiftFastOut(&DDSPort, DATA, CLOCK, LSBFIRST, tuning_word >> 16);
  shiftFastOut(&DDSPort, DATA, CLOCK, LSBFIRST, tuning_word >> 24);
  shiftFastOut(&DDSPort, DATA, CLOCK, LSBFIRST, 0x0);

  
  digitalFastWrite (DDSPort, LOAD, HIGH); 
}

void AD9850_init()
{
  // Set the IO
  digitalFastWrite(DDSioReg, RESET, OUTPUT);
  digitalFastWrite(DDSioReg, CLOCK, OUTPUT);
  digitalFastWrite(DDSioReg, LOAD, OUTPUT);
  digitalFastWrite(DDSioReg, DATA, OUTPUT);

  digitalFastWrite(DDSPort, RESET, LOW);
  digitalFastWrite(DDSPort, CLOCK, LOW);
  digitalFastWrite(DDSPort, LOAD, LOW);
  digitalFastWrite(DDSPort, DATA, LOW);
}

void AD9850_reset()
{
  //reset sequence is:
  // CLOCK & LOAD = LOW
  //  Pulse RESET high for a few uS (use 5 uS here)
  //  Pulse CLOCK high for a few uS (use 5 uS here)
  //  Set DATA to ZERO and pulse LOAD for a few uS (use 5 uS here)

  // data sheet diagrams show only RESET and CLOCK being used to reset the device, but I see no output unless I also
  // toggle the LOAD line here.

  digitalFastWrite(DDSPort, CLOCK, LOW);
  digitalFastWrite(DDSPort, LOAD, LOW);

  digitalFastWrite(DDSPort, RESET, LOW);
  delay(1);
  digitalFastWrite(DDSPort, RESET, HIGH);  //pulse RESET
  delay(1);
  digitalFastWrite(DDSPort, RESET, LOW);
  delay(1);

  digitalFastWrite(DDSPort, CLOCK, LOW);
  delay(1);
  digitalFastWrite(DDSPort, CLOCK, HIGH);  //pulse CLOCK
  delay(1);
  digitalFastWrite(DDSPort, CLOCK, LOW);
  delay(1);
  digitalFastWrite(DDSPort, DATA, LOW);    //make sure DATA pin is LOW

  digitalFastWrite(DDSPort, LOAD, LOW);
  delay(1);
  digitalFastWrite(DDSPort, LOAD, HIGH);  //pulse LOAD
  delay(1);
  digitalFastWrite(DDSPort, LOAD, LOW);
  // Chip is RESET now
}
