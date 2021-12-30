/*
   Si5351 bandwitched signal source used as an antenna tuning
   aid. Simply connect connect the unit to an antenna tuner or
   magnetic loop antenna and tune for a bargraph null on the
   OLED display.

   Additional frequency accuracy may be realized by calculating
   a calibration factor (see below) and entering it in the
   "CalFactor" variable.

   Permission is granted to use, copy, modify, and distribute this software
   and documentation for non-commercial purposes.

    Copyright (C) 2019,  Gene Marcus W3PM GM4YRE
    25 September, 2019

    8 December, 2019  v1_1 Optimized algoithm for a 1n34 diode detector 
    13 March, 2020    v1_2 Included option to display SWR values on line 4 of the
                           OLED display. Uncomment lines 190-192 if the SWR display 
                           is desired. Note: The SWR display is a very rough approximation
                           of actual SWR values.
    30 December 2021  v1_3 Newer baatches of Si5351's default to spread spectrum if
                           not set to disable. This version includes code to disable
                           the spread spectrum mode.    

                           

    COMPILER NOTE:  Uses library SSD1306Ascii by Bill Greiman. 
                    Load from: Sketch > Include Library > Manage Libraries
                    (Windows users will find the menu bar on top of the active window.  
                    Linux and MAC users will find the menu bar at the top of the primary 
                    display screen.)

*/

/*
  -----------------------------------------------------------------------
  Four pushbuttons are used to control the functions.
  Pushbutton pin allocations follow:
  -----------------------------------------------------------------------
  PB1  Decrease frequency
  PB2  Increase frequency
  PB3  Band Select
  PB4  Frequency step resolution select


  ------------------------------------------------------------------------
  Nano Digital Pin Allocations follow:
  ------------------------------------------------------------------------
  D0/RX
  D1
  D2
  D3
  D4
  D5
  D6 PB1  Decrease frequency
  D7 PB2  Increase frequency
  D8 PB3  Band Select
  D9 PB4  Frequency step resolution select
  D10
  D11
  D12
  D13

  A0/D14
  A1/D15
  A2/D16
  A3/D17 Voltage input from external SWR detector
  A4/D18 Si5351 & OLED SDA
  A5/D19 Si5351 & OLED SCL

*/

//----------------------------------------------------------------------------------------------------
//                   MCU  variables/configuration data follows:
//----------------------------------------------------------------------------------------------------
// include the library code:
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include <EEPROM.h>

// Set up MCU pins
#define FreqDown                 6
#define FreqUp                   7
#define BandSelect               8
#define Resolution               9

// Set sI5351A I2C address
#define Si5351A_addr          0x60

// Define OLED address
#define I2C_ADDRESS           0x3C

// initialize oled display
SSD1306AsciiAvrI2c oled;

// Define Si5351A register addresses
#define CLK_ENABLE_CONTROL       3
#define CLK0_CONTROL            16
#define CLK1_CONTROL            17
#define CLK2_CONTROL            18
#define SYNTH_PLL_A             26
#define SYNTH_PLL_B             34
#define SYNTH_MS_0              42
#define SYNTH_MS_1              50
#define SYNTH_MS_2              58
#define SSC_EN                 149
#define PLL_RESET              177
#define XTAL_LOAD_CAP          183


//___________________________Enter stand-alone calibration factor:______________________________
//    This entry is not required for general use.
//  - Connect the tuning aid to a frequency counter
//  - Set to 25 MHz
//  - Annotate counter frequency in Hz
//  - Subtract 25 MHz from counter reading
//  - Enter the difference in Hz (i.e. -245)
int CalFactor = 0;


//The following is set up for WSPR transmit frquencies. You
//may change the frequency plan to better suit your needs.
//Any number of frequencies may be entered, just end the list
//with a zero.
const unsigned long Freq_array [] =
{
  137500      ,
  475700      ,
  1838100     ,
  3570100     ,
  5288700     ,
  7040100     ,
  10140200    ,
  14097100    ,
  18105100    ,
  21096100    ,
  24926100    ,
  28126100    ,
  50294500    ,

  0
};

byte band, fStepcount;
int SWRval;
unsigned long fStep = 1, XtalFreq = 25000000, frequency, time_now, writeTime_now;
static bool HFflag = true;
String resolution = "1 Hz  ";

//----------------------------------------------------------------------------------------------------
//                         System set up  follows:
//----------------------------------------------------------------------------------------------------
void setup()
{
  Wire.begin(1);                  // join I2C bus (address = 1)
  si5351aStart();

  XtalFreq += CalFactor;

  // Set up push buttons
  pinMode(Resolution, INPUT);
  digitalWrite(Resolution, HIGH);         // internal pull-up enabled
  pinMode(BandSelect, INPUT);
  digitalWrite(BandSelect, HIGH);         // internal pull-up enabled
  pinMode(FreqUp, INPUT);
  digitalWrite(FreqUp, HIGH);             // internal pull-up enabled
  pinMode(FreqDown, INPUT);
  digitalWrite(FreqDown, HIGH);           // internal pull-up enabled

  // Retrieve stored data
  EEPROM.get (10, band);                  // Get stored VFO band
  if (band > 20 | band < 0) band = 0;     //Ensures valid EEPROM data - if invalid will default to band 0

  EEPROM.get (12, fStepcount);           // Get stored VFO step resolution
  if (fStepcount > 6 | fStepcount < 0) fStepcount = 0;    //Ensures valid EEPROM data - if invalid will default to 0

  EEPROM.get (14, frequency);               // Get stored VFO CLK1 frequency
  if (frequency > 1200000000 | frequency < 0) frequency = 10000000;  //Ensures valid EEPROM data - if invalid will default to 1 MHz

  Si5351_write(CLK_ENABLE_CONTROL, 0b00000011); // Enable CLK2

  si5351aSetFreq(SYNTH_MS_2, frequency);          // Set CLK2 frequency

  // Set oled font size and type
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);

  /*
   * Note: The display SWR are at best a very rough guidline to SWR valus when using a 1N34 diode
   * The builder may display the SWR graph values on line 4 of the OLED display by
   * uncommenting lines 190-192.
   */
  // Display approximate SWR values
  //oled.setCursor(0, 6);
  //oled.print("  1.5  2     3   4");
  //oled.setFont(fixed_bold10x15);
  

  setfreq();
  setResolution();
}


//----------------------------------------------------------------------------------------------------
//                         Loop starts here:
//        Loops consecutively to check MCU pins for activity
//----------------------------------------------------------------------------------------------------
void loop()
{
  setfreq();
  resDisplay();
  si5351aSetFreq(SYNTH_MS_2, frequency);
  SWRval = analogRead(A3);  // read the input pin
  oled.setCursor(0, 4);
  SWRval = SWRval / 9;
  oled.setFont(Adafruit5x7);
  for (int i = 0; i < 21; i++)
  {
    if (i < SWRval) oled.print(F("*"));
    else oled.print(F(" "));
  }
  oled.setFont(fixed_bold10x15);
  altDelay(100);
  oled.setCursor(0, 4);
  oled.print(F("                      "));

  // The frequency step resolution selection begins here:
  if (digitalRead(Resolution) == LOW)
  {
    altDelay(500);
    fStepcount++;
    if (fStepcount > 6) fStepcount = 0;
    setResolution();     // Call the set resolution subroutine
  }

  // Band selection
  if (digitalRead(BandSelect) == LOW)
  {
    altDelay(500);
    band++;
    if (Freq_array [band] == 0)band = 0;
    //altDelay(500);
    frequency = Freq_array [band];
    setfreq();
  }

  // Frequency Up/Down pushbutton algorithm begins here:
  if (digitalRead(FreqUp) == LOW) // Check for frequency up pushbutton A1 LOW
  {
    altDelay(500);
    // Increase frequency by the selected frequency step
    frequency += fStep;           // Increase CLK1 by frequency step
    setfreq();                    // Set and display new frequency
    resDisplay();                 // Call the resolution display subroutine
  }

  if (digitalRead(FreqDown) == LOW)// Check for frequency up pushbutton A1 LOW
  {
    altDelay(500);
    // Decrease frequency by the selected frequency step and check for 1-80 MHz limits
    frequency -= fStep;            // Decrease CLK1 by frequency step
    setfreq();                     // Set and display new frequency
    resDisplay();                  // Call the resolution display subroutine
  }

  // per Arduino data: EEPROM.put only writes if data is changed, however,
  // a 10 second delay is used to prevent unnecessary EEPROM updates during
  // short frequency changes
  if (millis() > writeTime_now + 10000)
  {
    EEPROM.put(14, frequency);
    EEPROM.put(12, fStepcount);
    EEPROM.put(10, band);
    writeTime_now = millis();
  }
}


//******************************************************************
// Step resolution function follows:
// Determines the step resolution
// Called by loop() and setup()
//******************************************************************
void setResolution()
{
  switch (fStepcount)
  {
    case 0:
      fStep = 1;
      resolution = "1 Hz  ";
      break;
    case 1:
      fStep = 10;
      resolution = "10 Hz  ";
      break;
    case 2:
      fStep = 100;
      resolution = "100 Hz ";
      break;
    case 3:
      fStep = 1000;
      resolution = "1 kHz  ";
      break;
    case 4:
      fStep = 10000;
      resolution = "10 kHz ";
      break;
    case 5:
      fStep = 100000;
      resolution = "100 kHz";
      break;
    case 6:
      fStep = 1000000;
      resolution = "1 MHz  ";
      break;
  }
  resDisplay;
}


//******************************************************************
// Step resolution display function follows:
// Display the frequency step resolution
// Called by loop() and step resolution functions
//******************************************************************
void resDisplay()
{
  oled.setCursor(0, 2);
  oled.print(F("Step "));
  oled.setCursor(54, 2);
  oled.print(resolution);
}


//******************************************************************
// Set frequency function follows:
// Calculates the frequency data to be sent to the Si5351 clock generator
// and displays the frequency on the olded display
//
// Called by loop() and setup()
//******************************************************************
void setfreq()
{
  unsigned long  temp_frequency = frequency; // Temporarily store frequency

  oled.setCursor(0, 0);
  char buf[11];
  if (temp_frequency >= 1000000L)
  {
    int MHz = int(temp_frequency / 1000000L);
    int kHz = int ((temp_frequency - (MHz * 1000000L)) / 1000L);
    int Hz = int (temp_frequency % 1000);
    snprintf(buf, sizeof(buf), "%2u,%03u,%03u", MHz, kHz, Hz);
  }

  else if (temp_frequency >= 1000L & temp_frequency < 1000000L)
  {
    int kHz = int (temp_frequency / 1000L);
    int Hz = int (temp_frequency % 1000L);
    snprintf(buf, sizeof(buf), "%6u,%03u", kHz, Hz);
  }
  else if (temp_frequency < 1000L)
  {
    int Hz = int (temp_frequency);
    snprintf(buf, sizeof(buf), "%10u", Hz);
  }
  oled.print(buf);
}

//******************************************************************
// Alternate delay function follows:
// altDelay is used because the command "delay" causes critical
// timing errors.
//
// Called by all functions
//******************************************************************
unsigned long altDelay(unsigned long delayTime)
{
  time_now = millis();
  while (millis() < time_now + delayTime) //delay 1 second
  {
    __asm__ __volatile__ ("nop");
  }
}


//******************************************************************
//  Si5351 Multisynch processing follows:
//  Generates the Si5351 clock generator frequency message
//
//  Called by sketch setup() and loop()
//******************************************************************
void si5351aSetFreq(int synth, unsigned long  freq)
{
  if (freq < 500000)
  {
    HFflag = false;
    freq = freq * 4;
  }
  else
  {
    HFflag = true;
  }
  unsigned long long CalcTemp;
  unsigned long  a, b, c, p1, p2, p3;

  c = 0xFFFFF;  // Denominator derived from max bits 2^20

  a = (XtalFreq * 36) / freq; // 36 is derived from 900/25 MHz
  CalcTemp = round(XtalFreq * 36) % freq;
  CalcTemp *= c;
  CalcTemp /= freq ;
  b = CalcTemp;  // Calculated numerator


  // Refer to Si5351 Register Map AN619 for following formula
  p3  = c;
  p2  = (128 * b) % c;
  p1  = 128 * a;
  p1 += (128 * b / c);
  p1 -= 512;

  // Write data to multisynth registers
  Si5351_write(synth, 0xFF);
  Si5351_write(synth + 1, 0xFF);
  if (HFflag == true)
  {
    Si5351_write(synth + 2, (p1 & 0x00030000) >> 16);
  }
  else
  {
    Si5351_write(synth + 2, 0b00100000 ^ ((p1 & 0x00030000) >> 16));
  }
  Si5351_write(synth + 3, (p1 & 0x0000FF00) >> 8);
  Si5351_write(synth + 4, (p1 & 0x000000FF));
  Si5351_write(synth + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  Si5351_write(synth + 6, (p2 & 0x0000FF00) >> 8);
  Si5351_write(synth + 7, (p2 & 0x000000FF));

}


//******************************************************************
//  Si5351 initialization functions follow:
//  Used to set up Si53351 clock generator parameters and PLLs
//
//  Called by sketch setup()
//******************************************************************
void si5351aStart()
{
  // Initialize Si5351A
  Si5351_write(XTAL_LOAD_CAP, 0b11000000);     // Set crystal load to 10pF
  Si5351_write(CLK_ENABLE_CONTROL, 0b00000000); // Enable all outputs
  Si5351_write(CLK0_CONTROL, 0b00001111);      // Set PLLA to CLK0, 8 mA output
  Si5351_write(CLK1_CONTROL, 0b00001111);      // Set PLLA to CLK1, 8 mA output
  Si5351_write(CLK2_CONTROL, 0b00101111);      // Set PLLB to CLK2, 8 mA output
  Si5351_write(PLL_RESET, 0b10100000);         // Reset PLLA and PLLB
  Si5351_write(SSC_EN,0b00000000);             // Disable spread spectrum


  // Set PLLA and PLLB to 900 MHz
  unsigned long  a, b, c, p1, p2, p3;

  a = 36;           // Derived from 900/25 MHz
  b = 0;            // Numerator
  c = 0xFFFFF;      // Denominator derived from max bits 2^20

  // Refer to Si5351 Register Map AN619 for following formula
  p3  = c;
  p2  = (128 * b) % c;
  p1  = 128 * a;
  p1 += (128 * b / c);
  p1 -= 512;

  // Write data to PLL registers
  Si5351_write(SYNTH_PLL_A, 0xFF);
  Si5351_write(SYNTH_PLL_A + 1, 0xFF);
  Si5351_write(SYNTH_PLL_A + 2, (p1 & 0x00030000) >> 16);
  Si5351_write(SYNTH_PLL_A + 3, (p1 & 0x0000FF00) >> 8);
  Si5351_write(SYNTH_PLL_A + 4, (p1 & 0x000000FF));
  Si5351_write(SYNTH_PLL_A + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  Si5351_write(SYNTH_PLL_A + 6, (p2 & 0x0000FF00) >> 8);
  Si5351_write(SYNTH_PLL_A + 7, (p2 & 0x000000FF));

  Si5351_write(SYNTH_PLL_B, 0xFF);
  Si5351_write(SYNTH_PLL_B + 1, 0xFF);
  Si5351_write(SYNTH_PLL_B + 2, (p1 & 0x00030000) >> 16);
  Si5351_write(SYNTH_PLL_B + 3, (p1 & 0x0000FF00) >> 8);
  Si5351_write(SYNTH_PLL_B + 4, (p1 & 0x000000FF));
  Si5351_write(SYNTH_PLL_B + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  Si5351_write(SYNTH_PLL_B + 6, (p2 & 0x0000FF00) >> 8);
  Si5351_write(SYNTH_PLL_B + 7, (p2 & 0x000000FF));

}

//******************************************************************
// Write I2C data function for the Si5351A follows:
// Writes data over the I2C bus to the appropriate device defined by
// the address sent to it.
// Called by sketch setup(), loop(), si5351aSetFreq, and
// si5351aStart functions.
//******************************************************************
uint8_t Si5351_write(uint8_t addr, uint8_t data)
{
  Wire.beginTransmission(Si5351A_addr);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}
//------------------------------------------------------------------------------------------------------
