// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

/*
  Additions by Jared Hamilton
  jchamilt@eagle.fgcu.edu
*/

//#include <avr/wdt.h>
//#include <avr/sleep.h>
//#include <avr/interrupt.h>
#include <XBee.h>
#include <SoftwareSerial.h>
#include <DallasTemperature.h>
#include <OneWire.h> // Get here: http://www.arduino.cc/playground/Learning/OneWire
#include <TinyGPS++.h>
#include <math.h>       // Conversion equation from resistance to %
// #include <dht.h>
#include <LowPower.h>   // Needed for sleep mode
// #include <Wire.h>       // I2C communication to RTC

#define TEMP_MODULE 1 // Enable One Wire Temperature Module.
#define SOIL_MODULE 1 // Enable Soil Read.
#define VOLT_MODULE 1 // Enable Voltage Check.
#define XBEE_MODULE 1 // Enable Xbee
#define SLEEP_MODULE 1 // Enable Sleep Module
#define GPS_MODULE 1 // Enable GPS Module
#define DEBUG_MODULE 0 //Enable Debug Output via Serial
#define DEBUG_MODULE_RESISTANCE 0 //Enable Resistance Debug Output via Serial
#define NUM_READS 13    // Number of sensor reads for filtering

// Set our Node value; an integer from 1 to 255 ( 1 is reserved for the CORE_NODE )
int Node = 96;

#if XBEE_MODULE // xbee
uint8_t text[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // this array will be used to transport data inbetween the XBees, id = 0
XBee xbee = XBee();
XBeeAddress64 remoteAddress = XBeeAddress64(0x00000000, 0x00000000); // Broadcast Address for use with 2.4 ghz use actuall address for production & 900mhz radios Below
// XBeeAddress64 remoteAddress = XBeeAddress64(0x13A200, 0x40D44C82); // this is the network address of the XBee cordinator which is where we want this node to send it's data sets
ZBTxRequest zbTx = ZBTxRequest(remoteAddress, text, sizeof(text));
#endif

// const int ledPin = 13; - using pin to power GPS now
// const int DS18B20Ground = 5;           // DS18B20 Pin 1
const int DS18B20Data = 3;  // DS18B20 Pin 2 NOTE: 4.7k pull-up resistor required between data and power
const int Soil_Bus = 4;   // const int DS18B20Power = 7;            // DS18B20 Pin 3
const int XBeeSleep = 2;               // Connect to XBee DTR
const int waitPeriod = 12;              // Number of 8 second cycles before waking up XBee and sending data (8*8 = 64 seconds) //8=3.03 minutes //39=13.02 minutes
const int waitPeriodGPS = 1145;          // 80 = 31 minutes // 3700 = approx 20 hours  4300 = approx 24 hours
const int waitPeriodSoil = 12;
const int gpsWakePin = 13;              // Turn GPS on via NPC Transistor
OneWire oneWire(DS18B20Data);  // Setup DS18S20 Temperature chip I/O
OneWire oneWire_Soil(Soil_Bus);
DallasTemperature sensors(&oneWire);        // Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors_soil(&oneWire_Soil);
long mVolt = 0;
int Fahr = 0;
int flag;
long checkVolt = 0;

// Variable Definition
volatile int timeKeeper = 0;   // set close to waitperiod to get temp reading faster
volatile int timeKeeperGPS = 0;
volatile int timeKeeperSoil = 0;
volatile float averageVcc = 0.0;
volatile float averageTemp = 0.0;

/*
  This sample code demonstrates just about every built-in operation of TinyGPS++ (TinyGPSPlus).
  It requires the use of SoftwareSerial, and assumes that you have a
  4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
// static const int RXPin = 11, TXPin = 7;  // Node 5
static const int RXPin = 12, TXPin = 11; // Node 9
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

int gpsSend = 1;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// For stats that happen every 5 seconds
unsigned long last = 0UL;
long lat1 = 0;
long lat2 = 0;
long long1 = 0;
long long2 = 0;
String stringOne = "";
// set up variables for battery voltage measurement
int analoginput = 0; // our analog pin
int analogamount = 0; // stores incoming value
float voltage = 0; // used to store voltage value

#if SOIL_MODULE
typedef struct {        // Structure to be used in percentage and resistance values matrix to be filtered (have to be in pairs)
  int moisture;
  long resistance;
} values;
const long knownResistor = 4700;  // Constant value of known resistor in Ohms
int supplyVoltage;                // Measured supply voltage
int sensorVoltage;                // Measured sensor voltage
int unsigned long moisture_pct = 0;
int unsigned long moisture_pct1 = 0;
int unsigned long moisture_pct2 = 0;
int unsigned long moisture_pct3 = 0;
float moisture_kpa1 = 0;
float moisture_kpa2 = 0;
float moisture_kpa3 = 0;
float compkPa = 0;
values valueOf[NUM_READS];        // Calculated  resistances to be averaged
long buffer[NUM_READS];
int index;
int i;                            // Simple index variable
// Equation One Constants
float eq1a = -3.213;
float eq1b = 4.093;

// Equation Constants
float soilC = 27.0;
float sensorR = 0.0;
float eq1 = -2.246;
float eq2 = 5.239;
float eq3 = (1 + 0.018 * (soilC - 24));
float eq4 = 0.06756;
float eq5 = pow(float(1 + 0.018 * (soilC - 24)), 2);
// equation variables
float R1 = 0;
float eqR = 0;
float eqR2 = 0;
#endif

void setup() {
  Serial.begin(9600);  // initialize serial communications at 9600 bps:
#if DEBUG_MODULE
  Serial.println("Booting......");
#endif
  // initialize the digital pins as an output.
  // Pin 4,5 is for sensor 1
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  // Pin 6,7 is for sensor 2
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  //Pin 8,9 is for sensor 3
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  text[0] = Node;
  // pinMode(DS18B20Ground, OUTPUT);
  // digitalWrite(DS18B20Ground, 0);      // Ground the DS18B20 GND pin
  pinMode(XBeeSleep, OUTPUT);
  digitalWrite(XBeeSleep, 0);          // Enable XBee
  //digitalWrite(ledPin, 1);             // Turn on Notification LED
  delay(20000);                         // 20 second LED blink, good for wireless programming
  //digitalWrite(ledPin, 0);             // Turn off Notification LED
  digitalWrite(XBeeSleep, 1);          // Disable XBee
#if TEMP_MODULE
  readTemp();
#endif

#if SOIL_MODULE
  readSoil();
#endif
#if GPS_MODULE
  pinMode(gpsWakePin, OUTPUT);
  digitalWrite(gpsWakePin, HIGH); // GPS On
  ss.begin(GPSBaud); //GPS
#if DEBUG_MODULE
  delay(500);
  Serial.println("Starting UP GPS - Please Wait 90 Seconds");
  delay(500);
#endif
  smartDelay(90000);
#endif
}



void loop() {

  // read sensor 1-3, filter, and calculate resistance value
  // Noise filter: median filter

#if TEMP_MODULE
  if (timeKeeper == (waitPeriod - 1)) { // Transmit every 8*8 (64) seconds
    readTemp();
  }
  else {                             // Add a reading to the average
    timeKeeper++;
  }
#endif
#if SOIL_MODULE
  if (timeKeeperSoil == (waitPeriodSoil - 1)) { //
    timeKeeperSoil = 0;
    readSoil();
    delay(500);
  }
  else {
    timeKeeperSoil++;
  }
#endif
#if VOLT_MODULE
  checkVolt = 100 * (float)readVcc();
#endif
#if GPS_MODULE
  if (timeKeeperGPS >= (waitPeriodGPS - 1) && checkVolt >= 320) {
    timeKeeperGPS = 0;
    gpsSend = 1;
  }
  else {
    timeKeeperGPS++;
  }
  while (gpsSend == 1) {
    GPSloop();
  }
#if DEBUG_MODULE
  Serial.print("GPS count at:");
  Serial.println(timeKeeperGPS);

#endif
  delay(500);

#endif
  Sleep();
}
#if TEMP_MODULE
void readTemp()
{
  averageVcc = (float)readVcc();
  averageTemp = (float)readTempDS18B20();

  Fahr = averageTemp * 100;
  mVolt = averageVcc * 1000;
#if DEBUG_MODULE
  Serial.println(Fahr);
  Serial.println(mVolt);
#endif
  stringOne += Node;    //assemble string to send
  stringOne += ",T&V,";
  stringOne += (averageTemp);
  stringOne += ",";
  stringOne += (averageVcc);
  stringOne += ",##";
  char charBuf[stringOne.length() + 1];
  stringOne.toCharArray(charBuf, (stringOne.length() + 1));
#if DEBUG_MODULE
  Serial.println(charBuf);
  Serial.println(stringOne.length() + 1);
#endif
  ConvertAndSend(stringOne);
  stringOne = "";
  averageVcc = 0;                    // Reset voltage for new measurements
  averageTemp = 0;                   // Reset temperature for new measurements
  timeKeeper = 0;
}
// See: http://www.arduino.cc/playground/Learning/OneWire
float readTempDS18B20() {
  // digitalWrite(DS18B20Power, HIGH);    // Power up the DS18B20
  sensors.begin();
  delay(5000);
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  sensors.requestTemperatures();
  float resultTempFloat = (DallasTemperature::toFahrenheit(sensors.getTempCByIndex(0)));
  delay(5000);
  sensors.requestTemperatures(); // Send the command to get temperatures
  resultTempFloat = (DallasTemperature::toFahrenheit(sensors.getTempCByIndex(0)));
  delay(100);
#if SOIL_MODULE
  sensors_soil.begin();
  delay(1000);
  sensors_soil.requestTemperatures();
  delay(1000);
  soilC = sensors_soil.getTempCByIndex(0);
#if DEBUG_MODULE
  Serial.print("Soil Temp in C: ");
  Serial.println(soilC);
#endif
#endif
  // digitalWrite(DS18B20Power, LOW);    // Turn off DS18B20
  return resultTempFloat;
}
#endif
#if SOIL_MODULE
void measure(int sensor, int phase_b, int phase_a, int analog_input)
{

  // read sensor, filter, and calculate resistance value
  // Noise filter: median filter

  for (i = 0; i < NUM_READS; i++) {

    // Read 1 pair of voltage values
    digitalWrite(phase_a, HIGH);                 // set the voltage supply on
    delayMicroseconds(25);
    supplyVoltage = analogRead(analog_input);   // read the supply voltage
    delayMicroseconds(25);
    digitalWrite(phase_a, LOW);                  // set the voltage supply off
    delay(1);
#if DEBUG_MODULE_RESITANCE
    Serial.print("A - ");
    Serial.println(supplyVoltage);
    Serial.println();
#endif
    digitalWrite(phase_b, HIGH);                 // set the voltage supply on
    delayMicroseconds(25);
    sensorVoltage = analogRead(analog_input);   // read the sensor voltage
    delayMicroseconds(25);
    digitalWrite(phase_b, LOW);                  // set the voltage supply off
#if DEBUG_MODULE_RESITANCE
    Serial.print("B - ");
    Serial.println(sensorVoltage);
    Serial.println();
#endif
    // Calculate resistance
    // the 0.5 add-term is used to round to the nearest integer
    // Tip: no need to transform 0-1023 voltage value to 0-5 range, due to following fraction
    long resistance = (knownResistor * (supplyVoltage - sensorVoltage) / sensorVoltage);

    delay(1);
    addReading(resistance);

  }
}


// calculate the kPa via resistance
float Resitance2kPa(float sensorR) {
  float R1 = sensorR;
#if DEBUG_MODULE
  Serial.print("Resistannce in ohms = ");
  Serial.println(sensorR);
#endif
  if (sensorR < 1000) {
    eqR = (R1 / 1000);
    compkPa = -20 * (eqR * (1 + 0.018 * (soilC - 24)) - 0.55);
#if DEBUG_MODULE
    Serial.println("Solved with equation one...");
#endif
  }
  else if (sensorR < 8000) {
    eqR = R1;
    float eqTOP = ((eq1a * (eqR / 1000)) - eq1b);
    float eqBOT = (1 - 0.009733 * (eqR / 1000) - 0.01205 * soilC);
    compkPa = eqTOP / eqBOT;
#if DEBUG_MODULE
    Serial.print("eqR = ");
    Serial.println(eqR / 1000, 5);
    Serial.print("TOP of EQuation = ");
    Serial.println(eqTOP, 5);
    Serial.print("Bottom of Equation = ");
    Serial.println(eqBOT, 5);
    Serial.println("Solved with equation two...");
#endif
  }
  else {
    eqR = (R1 / 1000);
    eqR2 = (eqR * eqR);
    compkPa = eq1 - eq2 * eqR * eq3 - eq4 * eqR2 * eq5;
    // moisture_kpa1 = - 2.246 - 5.239 * (sensor1 / 1000) * (1 + 0.018 *(27 - 24)) - 0.06756 * ((sensor1/1000) * (sensor1/1000)) * pow(float(1 + 0.018 * (27 - 24)), 2);
#if DEBUG_MODULE
    Serial.println("Solved with equation three...");
#endif
  }
  if (compkPa > 0) {
    compkPa = 0;
  }
  if (compkPa < -400) {
    compkPa = -400;
  }
  return compkPa;
}

// Averaging algorithm

void addReading(long resistance) {
  buffer[index] = resistance;
  index++;
  if (index >= NUM_READS) index = 0;
}

long average() {
  long sum = 0;
  for (int i = 0; i < NUM_READS; i++) {
    sum += buffer[i];
  }
  return (long)(sum / NUM_READS);
}

void readSoil() {
  measure(1, 6, 5, 1);
  long read1 = average();
  measure(1, 5, 6, 2);
  long read2 = average();
  long sensor1 = (read1 + read2) / 2;
  sensorR = sensor1;
  moisture_kpa1 = Resitance2kPa(sensorR);
  moisture_pct = pow(float(sensor1 / 31.65), float(1 / -1.695)) * 400;
  if (moisture_pct > 100)
  {
    moisture_pct1 = 100;
  }
  else
  {
    moisture_pct1 = moisture_pct;
  }
#if DEBUG_MODULE
  Serial.print("sensor resistance = ");
  Serial.println(sensor1);
  Serial.print("Soil Moisture = ");
  Serial.print(moisture_pct1);
  Serial.println(" %");
  Serial.print("Soil Moisture Sensor 1 kPa = ");
  Serial.println(moisture_kpa1);
  Serial.println();
#endif
  delay(100);
  measure(2, 8, 7, 3);
  long read3 = average();
  measure(2, 7, 8, 4);
  long read4 = average();
  long sensor2 = (read3 + read4) / 2;
  sensorR = sensor2;
  moisture_kpa2 = Resitance2kPa(sensorR);
  moisture_pct = pow(float(sensor2 / 31.65), float(1 / -1.695)) * 400;
  if (moisture_pct > 100)
  {
    moisture_pct2 = 100;
  }
  else

  {
    moisture_pct2 = moisture_pct;
  }
#if DEBUG_MODULE
  Serial.print("sensor resistance = ");
  Serial.println(sensor2);
  Serial.print("Soil Moisture = ");
  Serial.print(moisture_pct2);
  Serial.println(" %");
  Serial.print("Soil Moisture Sensor 2 kPa = ");
  Serial.println(moisture_kpa2);
  Serial.println();
#endif
  delay(100);
  measure(3, 10, 9, 5);
  long read5 = average();
  measure(3, 9, 10, 6);
  long read6 = average();
  long sensor3 = (read5 + read6) / 2;
  sensorR = sensor3;
  moisture_kpa3 = Resitance2kPa(sensorR);
  moisture_pct = pow(float(sensor3 / 31.65), float(1 / -1.695)) * 400;
  if (moisture_pct > 100)
  {
    moisture_pct3 = 100;
  }
  else
  {
    moisture_pct3 = moisture_pct;
  }
#if DEBUG_MODULE
  Serial.print("sensor resistance = ");
  Serial.println(sensor3);
  Serial.print("Soil Moisture = ");
  Serial.print(moisture_pct3);
  Serial.println(" %");
  Serial.print("Soil Moisture Sensor 3 kPa = ");
  Serial.println(moisture_kpa3);
  Serial.println();
#endif
  /*
  */
  // measure and print battery voltage
  float Vcc = (readVcc2());
  Vcc = Vcc / 1000;
#if DEBUG_MODULE
  Serial.println();
  Serial.print("Voltage: ");
  Serial.print(Vcc, 2);
  Serial.println(" V");
  Serial.println();
#endif
  delay(100);
  stringOne += Node;
  stringOne += ",SMR,";
  stringOne += (- moisture_kpa1);
  stringOne += ",";
  stringOne += (- moisture_kpa2);
  stringOne += ",";
  stringOne += (- moisture_kpa3);
  char charBuf[stringOne.length() + 1];
  stringOne.toCharArray(charBuf, (stringOne.length() + 1));
#if DEBUG_MODULE
  Serial.println(charBuf);
  Serial.println(stringOne.length() + 1);
#endif
  ConvertAndSend(stringOne);
  stringOne = "";
}
#endif


#if GPS_MODULE
void GPSloop() {
  {
    // Dispatch incoming characters
    if (digitalRead(gpsWakePin) == 0) {
      digitalWrite(gpsWakePin, HIGH); // Turn GPS on
      delay(5000);
    }
    while (ss.available() > 0)
      gps.encode(ss.read());

    if (gps.location.isValid())
    {
      lat1 = gps.location.rawLat().deg;
      lat2 = gps.location.rawLat().billionths;
      long1 = gps.location.rawLng().deg;
      long2 = gps.location.rawLng().billionths;
      stringOne += Node;
      stringOne += ",GPS,";
      stringOne += lat1;
      stringOne += ".";
      stringOne += lat2;
      stringOne += ",-";
      stringOne += long1;
      stringOne += ".";
      stringOne += long2;
      stringOne += ",##";
#if DEBUG_MODULE
      Serial.print(lat1);
      Serial.print(".");
      Serial.print(lat2);
      Serial.print(", -");
      Serial.print(long1);
      Serial.print(".");
      Serial.println(long2);
#endif
      char charBuf[stringOne.length() + 1];
      stringOne.toCharArray(charBuf, (stringOne.length() + 1));
      Serial.println(charBuf);
      Serial.println(stringOne.length() + 1);
      ConvertAndSend(stringOne);
      stringOne = "";
      gpsSend = 0; // turn off GPS
      pinMode(gpsWakePin, LOW); // turn Off GPD
    }
    else {
#if DEBUG_MODULE
      Serial.print(F("Fix Age="));
      Serial.println(gps.date.age());
      Serial.println("No fix, waiting 15 seconds");
#endif
      smartDelay(15000);
      if (gps.satellites.isUpdated())
      {
#if DEBUG_MODULE
        Serial.print(F("SATELLITES Fix Age="));
        delay(10);
        Serial.print(gps.satellites.age());
        delay(10);
        Serial.print(F("ms Value="));
        delay(10);
        Serial.println(gps.satellites.value());
        delay(10);
        Serial.println();
#endif
      }
    }
    if (gps.charsProcessed() < 10)
      Serial.println(F("WARNING: No GPS data.  Check wiring."));
    last = millis();
  }
}




void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
#endif

// See: http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
float readVcc() {
  signed long resultVcc;
  float resultVccFloat;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(10);                           // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);                 // Convert
  while (bit_is_set(ADCSRA, ADSC));
  resultVcc = ADCL;
  resultVcc |= ADCH << 8;
  resultVcc = 1126400L / resultVcc;    // Back-calculate AVcc in mV
  resultVccFloat = (float)resultVcc / 1000.0; // Convert to Float
  return resultVccFloat;
}

long readVcc2() {
  long result;                       // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = (1125300L / result); // Back-calculate AVcc in mV
  return result;
}




void ConvertAndSend(String stringOne) {

  uint8_t charArray[36] = {};
  for (int i = 0; i < (stringOne.length() + 1); i++) { // subroutine to convert to hex
    charArray[i] = stringOne.charAt(i);
#if DEBUG_MODULE
    Serial.print(charArray[i]);
    delay(10);
    Serial.print(", ");
#endif
  }
  Serial.println();
  Serial.println("Packet to Follow Below");
  ZBTxRequest TmpVltzbTx = ZBTxRequest(remoteAddress, charArray, sizeof(charArray));
  digitalWrite(XBeeSleep, 0);        // Enable XBee
  delay(24000);                         // Wait for XBee Wakeup  may need to be tuned
  xbee.send(TmpVltzbTx); //this sends our XBee payload array to the coordinator
  delay(10000);
  digitalWrite(XBeeSleep, 1);        // Disable XBee
  stringOne = "";
#if DEBUG_MODULE
  Serial.println();
#endif
}


void Sleep() {
  int sleepcount = 8;
  while (sleepcount > 0) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);   // Sleep for 8 x 8 Seconds
    sleepcount--;
  }
}
