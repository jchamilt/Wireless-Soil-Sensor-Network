//-----------------------
// Remote One Wire Tempertaure Monitor
// Version 1.0
// Sensor Array
// Jared Hamilton
//-----------------------

#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <XBee.h>
#include <Sleep_n0m1.h>


// Set our Node value; an integer from 1 to 255 ( 1 is reserved for the CORE_NODE )
int Node = 5;


#define TEMP_MODULE 1 // Enable One Wire Temperature Module.
#define VOLT_MODULE 1 // Enable Voltage Check.
#define XBEE_MODULE 1 // Enable Xbee
#define SLEEP_MODULE 1 // Enable Sleep Module

#if TEMP_MODULE
long MasterTempSensor;
// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
int TmpDStemp, SignBit, Tc_100, Whole, Fract;
#endif



uint8_t text[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // this array will be used to transport data inbetween the XBees, id = 0


#if XBEE_MODULE // xbee
XBee xbee = XBee();
XBeeAddress64 remoteAddress = XBeeAddress64(0x00000000, 0x00000000); // this is the network address of the XBee cordinator which is where we want this node to send it's data sets
ZBTxRequest zbTx = ZBTxRequest(remoteAddress, text, sizeof(text));
uint32_t INTERVAL_XBEE_SEND = 5;   // default at 800,000 or about 50 seconds
long XbeeBeat = 3;
#endif

#if TEMP_MODULE
  uint32_t INTERVAL_TEMP_SENSOR = 800000;   // default at 800,000 or about 50 seconds
  long TempBeat = 0 ;
  int Fahr = 0;
#endif

#if VOLT_MODULE
	uint32_t INTERVAL_VOLT_SENSOR = 800000;   // default at 800,000 or about 50 seconds
	long VoltBeat = 0 ;
	long mVolt = 0;
	// const float referenceVolts = 5;        // the default reference on a 5-volt board
	const float referenceVolts = 3.3;  // use this for a 3.3-volt board

	const float R1 = 2200; // value for a maximum voltage of 10 volts
	const float R2 = 1000;
	// determine by voltage divider resistors, see text
	const float resistorFactor = 1023*(R2/(R1 + R2));  
	const int batteryPin = 5;         // +V from battery is connected to analog pin 0
#endif

#if SLEEP_MODULE
	Sleep sleep;
	unsigned long sleepTime; //how long you want the arduino to sleep
#endif


int flag;


void setup(void) {
  Serial.begin(9600);
  text[0] = Node;
#if SLEEP_MODULE
  sleepTime = 50000; //set sleep time in ms, max sleep time is 49.7 days
#endif
}
 
void loop(void) {

#if XBEE_MODULE
	XbeeBeat ++;
	Serial.println(XbeeBeat);
#endif 



 #if TEMP_MODULE

  int Fahr = getTemperature(); // reads the temperature from the tmp102 into celsius
  Serial.print("temperature in Fahrenheit: "); //debugging info, sent to the serial monitor
  Serial.println(Fahr);
	
#endif

#if VOLT_MODULE
   int val = analogRead(batteryPin);  // read the value from the sensor
   float volts = (val / resistorFactor) * referenceVolts ; // calculate the ratio
    // Serial.println(volts);  // print the value in volts
    long mVolt = volts * 10 ;     // Reads Voltage
  // Serial.print("Voltage is: "); //debugging info, sent to the serial monitor
  // Serial.print(mVolt);
  // Serial.println(" mv's");
	
#endif
  


 #if TEMP_MODULE
 // this simply fills the readings we acquired above into our XBee payload array
 //Fahr INTO PAYLOAD
 //break down 10-bit reading into two bytes and place in payload
 //we are sending this flag as part of the XBee payload to indicate if the temperature value is above or beyond freezing

 if (Fahr < 32) 
     {flag = 0;}
 else
 {flag = 1;}

 text[1] = flag;
 text[2] = Fahr >> 8 & 0xff;
 text[3] = Fahr & 0xff;
        #endif

 #if VOLT_MODULE
 // this simply fills the readings we acquired above into our XBee payload array
 //mVolt INTO PAYLOAD
 //break down 10-bit reading into two bytes and place in payload

 text[4] = mVolt >> 8 & 0xff;
 text[5] = mVolt & 0xff;

		

#endif




#if XBEE_MODULE
  if (XbeeBeat >= INTERVAL_XBEE_SEND) {
	  XbeeBeat = 0;
	  Serial.print("Node: ");
	  Serial.println(text[0]); 
	  Serial.println("..."); //Print temp being sent
	  Serial.print("Temp: ");
      float A = (text[3]+(text[2]*256));
	  A = A/100;
	  Serial.println(A);
	  Serial.print("mv: "); //Print voltage being sent
	  long B = (text[5]+(text[4]*256));
	  Serial.println(B);
	  delay(100);
	xbee.send(zbTx); //this sends our XBee payload array to the coordinator
	  delay(100);

  }
#endif

#if SLEEP_MODULE
  sleep.pwrDownMode(); //set sleep mode
  sleep.sleepDelay(sleepTime); //sleep for: sleepTime
#endif
}

#if TEMP_MODULE
float getTemperature()
{
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  sensors.requestTemperatures(); // Send the command to get temperatures
  float Fahr = (DallasTemperature::toFahrenheit(sensors.getTempCByIndex(0))*100);
  Serial.println((DallasTemperature::toFahrenheit(sensors.getTempCByIndex(0))));
  delay(100);
  return Fahr;
  
}
#endif
#if VOLT_MODULE
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long mVolt = (high<<8) | low;
 
  mVolt = 1125300L / mVolt; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  delay(100);
  return mVolt; // Vcc in millivolts
  
}
#endif
