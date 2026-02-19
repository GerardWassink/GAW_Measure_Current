/* ------------------------------------------------------------------------- *
 * Name   : GAW_Measure_Current
 * Author : Gerard Wassink
 * Date   : January 2026
 * Purpose: Measure current for model railroad (5V, 12V & DCC)
 * Versions:
 *   0.1  : Initial code base, code for INA219's
 *   0.2  : Built in I2C LCD display
 *   0.3  : Debugging switch coded
 *						DCC measurement commented out, need different method
 *   0.4  : DCC measurement built in with ACS712 (5Amp)
 *   1.0  : Everything works, glitches ironed out
 *   1.1  : Improved average readings
 *            ditched ACS712 library
 *
 *------------------------------------------------------------------------- */
#define progVersion "1.1"                   // Program version definition 
/* ------------------------------------------------------------------------- *
 *             GNU LICENSE CONDITIONS
 * ------------------------------------------------------------------------- *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation version 2
 * of the License.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see
 * <https://www.gnu.org/licenses/>.
 * 
 * ------------------------------------------------------------------------- *
 *       Copyright (C) January 2025 Gerard Wassink
 * ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- *
 *       Debugging ON / OFF
 * ------------------------------------------------------------------------- */
#define DEBUG 0


/* ------------------------------------------------------------------------- *
 *                                                  Include functional files
 * ------------------------------------------------------------------------- */
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include "Arduino_INA219_I2C_No_Library.h"
#include "LibPrintf.h"                      // use printf, not Serial.print


/* ------------------------------------------------------------------------- *
 *                                      INA219 Registers used in this sketch
 * ------------------------------------------------------------------------- */
#define INA219_REG_CALIBRATION (0x5)
#define INA219_REG_CONFIG (0x0)
#define INA219_REG_CURRENT (0x04)


/* ------------------------------------------------------------------------- *
 *                        INA219 Config values used to measure current in mA
 * ------------------------------------------------------------------------- */
#define INA219_CONFIG_BVOLTAGERANGE_16V 		    (0x0000)  // 0-16V Range
#define INA219_CONFIG_BVOLTAGERANGE_32V 		    (0x2000)  // 0-32V Range
#define INA219_CONFIG_GAIN_8_320MV 				      6144	    // 8 x Gain
#define INA219_CONFIG_BADCRES_12BIT 			      384 	    // Bus ADC resolution bits
#define INA219_CONFIG_SADCRES_12BIT_1S_532US 	  24 		    // 1 x 12=bit sample
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS 7		      // Continuous conversion (not triggered)
#define INA219_CONFIG_SADCRES_12BIT_8S_4260US 	(0x0058)  // 8 x 12-bit shunt samples averaged together


/* ------------------------------------------------------------------------- *
 *     I2C address of the INA219 devices (can be changed by soldering board)
 * ------------------------------------------------------------------------- */
byte response;
byte address1 = 0x40;                       //  5V current measurement
byte address2 = 0x41;                       // 12V current measurement


/* ------------------------------------------------------------------------- *
  *                                                    Create display object
  * ------------------------------------------------------------------------ */
LiquidCrystal_I2C display1(0x25, 16, 2);    // Display1 object


/* ------------------------------------------------------------------------- *
  *                                                 ACS712 related variables
  * ------------------------------------------------------------------------ */
float offset = 0;
const float sensitivity = 0.185;            // 5A versie
#define sensorPin A0                        // analog pin for ACS712


/* ------------------------------------------------------------------------- *
  *                                    Number of iterations for measurements
  * ------------------------------------------------------------------------ */
float numSamples = 10;
int count = 1;


/* ------------------------------------------------------------------------- *
  *                                               Variables for measurements
  * ------------------------------------------------------------------------ */
float current1 = 0, current2 = 0, current3 = 0;
float sumcurr1 = 0, sumcurr2 = 0, sumcurr3 = 0;


/* ------------------------------------------------------------------------- *
  *                                                 Various global variables
  * ------------------------------------------------------------------------- */
char buffer[50];                            // for displaying on LCD


/* ------------------------------------------------------------------------- *
 * Initial setup                                                     setup()
 * ------------------------------------------------------------------------- */
void setup() {

  Serial.begin(57600);
  while (!Serial) {
    delay(1);
  }
  delay(500);
  printf("GAW_Measure_Current v%s\n", progVersion);

  Display_init();                           // Initialize I2C_display

/* ----------------------------------------------- Initialize INA219 modules */
  INA219_init(address1, INA219_CONFIG_BVOLTAGERANGE_16V);  // 5V power supply
	delayMicroseconds(4260);                                 // conversion delay
   INA219_init(address2, INA219_CONFIG_BVOLTAGERANGE_16V); // 12V power supply
	delayMicroseconds(4260);                                 // conversion delay

/* ---------------------------------------------------- Prepare ACS12 module */
  printf("Determining ACS712 offset\n");    // Measure offset (no trains driving!)
  long sum = 0;
  for (int i = 0; i < 2000; i++) {
    sum += analogRead(sensorPin);
    delayMicroseconds(200);
  }
  offset = sum / 2000.0;

	Serial.println("\nSetup completed, measuring starts.\n");

}



/* ------------------------------------------------------------------------- *
 * Perpetual loop                                                     loop()
 * ------------------------------------------------------------------------- */
void loop() {

  current1 = readCurrent(address1);         // get 5V pwr current
  if (current1 < 0) current1 *= -1;         // flip when negative
  sumcurr1 += current1;                     // add to total

  current2 = readCurrent(address2);         // get 12V pwr current
  if (current2 < 0) current2 *= -1;         // flip when negative
  sumcurr2 += current2;                     // add to total

  count++;                                  // bump up count

  if (count > numSamples) {                 // do we have numSamples?
    count = 1;                              // Reset counter

    current1 = sumcurr1 / numSamples;       // Calculate
    current2 = sumcurr2 / numSamples;       //   mean values
    current3 = measure_DCC();               // Measure DCC current
  
    sumcurr1 = 0; sumcurr2 = 0;             // Reset totals

                                            // Display currents on LCD
    sprintf(buffer, "%4.0f  %4.0f  %4.0f ", current1, current2, current3);
    LCD_display(display1, 1, 0, buffer);

    delay(100);                             // Wait a moment

  }

}



/* ------------------------------------------------------------------------- *
 * Use ACS712 to measure DCC square wave RMS current           measure_DCC()
 * ------------------------------------------------------------------------- */
float measure_DCC() {

  long sumSquare = 0;                       // hold sum total
  int samples = 0;                          // counter for samples
  const unsigned long meetTijd = 100;       // measure for 100 ms
  unsigned long startTime = millis();       // start time

  while (millis() - startTime < meetTijd) {
    int raw = analogRead(sensorPin);        // read raw values
    float diff = raw - offset;              // subtract determined offset
    sumSquare += diff * diff;               // increase sum with diff squared
    samples++;                              // count samples
  }

  float meanSquare = sumSquare / (float)samples;  // Calculate mean
  float rmsCounts = sqrt(meanSquare);       // Calculate square root

  float voltage = rmsCounts * (5.0 / 1023.0); // convert to volts

  float current = (voltage / sensitivity) * 1000; // convert to milli Amps

  return(current);
}



/* ------------------------------------------------------------------------- *
 * Initialize an INA219 module                                 INA219_init()
 * ------------------------------------------------------------------------- */
void INA219_init(byte address, int range) {

  printf("Configuring INA219 module 0x%02x - ", address);

	Wire.beginTransmission(address);          // Can we communicate?
	Wire.write(INA219_REG_CALIBRATION);       // Calibrate the module
	Wire.write((4096 >> 8) & 0xFF);
	Wire.write(4096 & 0xFF);
	Wire.endTransmission();

	Wire.beginTransmission(address);          // Configure the module
	Wire.write(INA219_REG_CONFIG);
                                          	// Set Config for what we want:
	uint16_t config = range                   // voltage, amp range
						| INA219_CONFIG_GAIN_8_320MV    // 8 x Gain
						| INA219_CONFIG_BADCRES_12BIT   // 12-bit bus ADC resolution
						| INA219_CONFIG_SADCRES_12BIT_8S_4260US // number of averaged samples
						| INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS; // Continuouis conversion
	Wire.write((config >> 8) & 0xFF);
	Wire.write(config & 0xFF);
	response = Wire.endTransmission();        // did we get an OK ?

	if (response == 0) {                      // Report result
		printf("found and initialized\n");
	} else {
		printf("Unknown response: %d,\t\texecution stops.\n", response);
    delay(2000);
    exit(0);                                // HALT when error
	}

}



/* ------------------------------------------------------------------------- *
 * Read current from INA219 module                             readCurrent()
 * ------------------------------------------------------------------------- */
float readCurrent(byte address) {

	Wire.beginTransmission(address);          // Initiate transmission
	Wire.write(INA219_REG_CURRENT);           // Register we want to write to (Current)
	Wire.endTransmission();                   // Finish this "conversation"

	Wire.beginTransmission(address);          // Initiate transmission
	Wire.requestFrom((int) address, 2);       // Request the current in mA
	Wire.endTransmission();                   // Finish this "conversation"

	delayMicroseconds(4260);                  // allow INA219 to do the conversion 

	float value = ((Wire.read() << 8) | Wire.read()) / 10;

  return(value);
}



/* ------------------------------------------------------------------------- *
 * Initialize the display                                     Display_init()
 * ------------------------------------------------------------------------- */
void Display_init() {
  Wire.begin();                           // Start I2C

  display1.init();                        // Initialize display backlight 
  display1.backlight();                   //   on by default

  LCD_display(display1, 0, 0, F("Current measure "));
  LCD_display(display1, 1, 0, F("version "));
  LCD_display(display1, 1, 8, progVersion);

  delay(1500);                            // show for 1,5 second

  LCD_display(display1, 0, 0, F("--5V  -12V  -DCC"));
  LCD_display(display1, 1, 0, F("                "));

}


/* ------------------------------------------------------------------------- *
  *       Routine to display stuff on the display of choice     LCD_display()
  * ------------------------------------------------------------------------- */
void LCD_display(LiquidCrystal_I2C screen, int row, int col, String text) {
  screen.setCursor(col, row);
  screen.print(text);
}
