//----------------------------------------------
// Jared Hamilton
// Temperature Probe Network Head End
// version 0.5 Beta
//----------------------------------------------

#include <XBee.h>
#include <SPI.h>
#include <Ethernet.h>
// #include <ICMPPing.h> // Currently has issues with Xbee Library
#include <LED.h>
#include <Client.h>
// #include <Mail.h>  // Not Working as of 11/2015
#include <SMTPClient.h>
#include <avr/wdt.h>
#include <mysql.h>
#include <sha1.h>

/*
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte ip[] = { 192, 168, 9, 3 };
byte gateway[] = { 192, 168, 9, 1 };
byte subnet[] = { 255, 255, 255, 0 };
byte server[] = { 192, 168, 9, 1 };
byte pingAddr[] = { 192, 168, 9, 1 }; // ip address to ping
byte smtprelay[] = { 172, 16, 200, 241 }; // IP for SMTP Relay
*/

byte mac[] = { 0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte ip[] = { 172, 16, 222, 12 };
byte gateway[] = { 172, 16, 1, 1 };
byte subnet[] = { 255, 255, 0, 0 };
byte server[] = { 172, 16, 200, 100 };
byte pingAddr[] = { 172, 16, 1, 1 }; // ip address to ping
// byte smtprelay[] = { 172, 16, 200, 254 }; // IP for SMTP Relay

EthernetClient ethClient;
// SmtpClient client(&ethClient, smtprelay, 25); // SMTP Client

// User defined variables for Exosite reporting period and averaging samples
#define REPORT_TIMEOUT 900000 //milliseconds period for reporting to Exosite.com <15 Minutes>
#define SENSOR_READ_TIMEOUT 900000 //milliseconds period for reading sensors in loop <15 Minutes>

//global variables
// byte macData[] = { 0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; // <-- Fill in your MAC here! (e.g. {0x90, 0xA2, 0xDA, 0x00, 0x22, 0x33})
static unsigned long sendPrevTime = 0;


//Ping Variables
// SOCKET pingSocket = 0;
// int delayMS = 60 * 1000; // delay between successive pings (60 * 1000 = 60 seconds)

XBee xbee = XBee();
ZBRxResponse zbRx = ZBRxResponse();
uint8_t text[36] = {}; //initialises the data array with 0s

//local variables to submit the readings from the payload to ExoSite
int A = 0; // Temperature Variable
int B = 0; // Voltage Variable
int C = 0; 
int D = 0;
int E = 0; // End Node ID Variable
int F = 0;
int I = 0;
int flag;

char buffer[256];

// Define LED Objects

LED led1 = LED(24);
LED led2 = LED(26);
LED led3 = LED(28);
LED led4 = LED(30);
int LEDstatus = 0;

void(*resetFunc) (void) = 0; //declare reset function @ address 0

// MYSQL Connector Setup
/*
CREATE TABLE test_arduino.node_ID (id MEDIUMINT NOT NULL AUTO_INCREMENT, node float, node_date_time timestamp, primary key (id));
CREATE TABLE test_arduino.IOT_data (IOT_data_id MEDIUMINT NOT NULL AUTO_INCREMENT,id MEDIUMINT, IOT_data_type VARCHAR(20), IOT_value float, primary key (IOT_data_id));
*/

int num_fails;
#define MAX_FAILED_CONNECTS 5

IPAddress server_addr(172, 16, 222, 221); // dev Naples
// IPAddress server_addr(192, 168, 9, 22); //dev NorthGrove
char user[] = "uno";
char password[] = "uno";

static unsigned long sendPrevTimeSQL = 0;

Connector my_conn;        // The Connector/Arduino reference

const char INSERT_DATA_ID[] = "INSERT INTO test_arduino.node_id (node, node_date_time) VALUES (%s, NULL)";
const char INSERT_DATA_T[] = "INSERT INTO test_arduino.iot_data (IOT_value, id, IOT_data_type) VALUES (%s, %ld, 'Temp')";
const char INSERT_DATA_V[] = "INSERT INTO test_arduino.iot_data (IOT_value, id, IOT_data_type) VALUES (%s, %ld, 'Volt')";
const char INSERT_DATA_L[] = "INSERT INTO test_arduino.iot_data (IOT_value, id, IOT_data_type) VALUES (%s, %ld, 'Lat')";
const char INSERT_DATA_G[] = "INSERT INTO test_arduino.iot_data (IOT_value, id, IOT_data_type) VALUES (%s, %ld, 'Long')";
const char INSERT_DATA_S1[] = "INSERT INTO test_arduino.iot_data (IOT_value, id, IOT_data_type) VALUES (%s, %ld, 'SM1')";
const char INSERT_DATA_S2[] = "INSERT INTO test_arduino.iot_data (IOT_value, id, IOT_data_type) VALUES (%s, %ld, 'SM2')";
const char INSERT_DATA_S3[] = "INSERT INTO test_arduino.iot_data (IOT_value, id, IOT_data_type) VALUES (%s, %ld, 'SM3')";
// const char INSERT_DATA_IOT[] = "INSERT INTO test_arduino.iot_data (id, IOT_data_type, IOT_value ) VALUES (%ld, %s, %s)"; 
// const char SELECT_LAST_INSERT[] = "SELECT LAST_INSERT_ID()";
const char SELECT_LAST_INSERT[] = "SELECT MAX(id) FROM test_arduino.node_id where node = %s;";
const char QUERY_SQL[] = "select @@version";
 

//// Digest New GPS Packet Stuff
String myString = "11,GPS,26.173467333,-81.770491167,##";
int commaIndex = myString.indexOf(',');
int secondCommaIndex = myString.indexOf(',', commaIndex + 1);
int thirdCommaIndex = myString.indexOf(',', secondCommaIndex + 1);
int fourthCommaIndex = myString.indexOf(',', thirdCommaIndex + 1);
int fifthCommaIndex = myString.indexOf(',', fourthCommaIndex + 1);

String firstValue = "";
String secondValue = "";
String thirdValue = ""; // To the end of the string
String fourthValue = ""; // To the end of the string
String fifthValue = ""; // To the end of the string
// Declare Packet Types for comparison
const char typeGPS[] = "GPS"; //Type GPS
const char typeTnV[] = "T&V"; //Type Temp & Voltage
const char typeSMR[] = "SMR"; //Type Temp & Voltage

int get_free_memory()
{
	extern char __bss_end;
	extern char *__brkval;
	int free_memory;
	if ((int)__brkval == 0)
		free_memory = ((int)&free_memory) - ((int)&__bss_end);
	else
		free_memory = ((int)&free_memory) - ((int)__brkval);
	return free_memory;
}

int wdt_counter = 0;
int wdt_variable = 60;
// This function is called upon a HARDWARE RESET:
void wdt_first(void) __attribute__((naked)) __attribute__((section(".init3")));

// Clear SREG_I on hardware reset.
void wdt_first(void)
{
	// Note that for newer devices (any AVR that has the option to also
	// generate WDT interrupts), the watchdog timer remains active even
	// after a system reset (except a power-on condition), using the fastest
	// prescaler value (approximately 15 ms). It is therefore required
	// to turn off the watchdog early during program startup.
	MCUSR = 0; // clear reset flags
	wdt_disable();
	// http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_softreset.html
}

void wdt_init() {
	cli();
	WDTCSR = (1 << WDCE) | (1 << WDE);   			// Enable the WD Change Bit - configure mode
	WDTCSR = (1 << WDIE) | (1 << WDP2) | (1 << WDP1);		// Enable WDT Interrupt, and Set Timeout to ~1 seconds ,or use WDTO_1S
	sei();							// enable interrupts
}

// This is the interrupt code, called every second, unless wdt_reset() was called sooner
ISR(WDT_vect)
{
	wdt_counter++;
	if (wdt_counter < wdt_variable) { // 60 seconds limit , that is ONE MINUTE
		// start timer again (we are still in interrupt-only mode)
		wdt_reset();
	}
	else {
		// go for immediate reset
		WDTCSR = (1 << WDCE) | (1 << WDE);	// Enable the WD Change Bit - configure mode
		WDTCSR = (1 << WDE) | (1 << WDP0);	// set reset flag (WDE) and 16ms (WDP0)
	}
}

// Use this custom function to reset, to also set the seconds counter back to 0
// The watchdog will reset only if one full minute is reached , without your code calling my_wdt_reset() before that
void my_wdt_reset() {
	wdt_counter = 0;
	wdt_reset();
}


void setup() {
	led1.blink(250, 4);
	led2.blink(250, 4);
	led3.blink(250, 4);
	led4.blink(250, 4);
	wdt_init();
	Serial.begin(9600);
	Serial1.begin(9600);
	xbee.setSerial(Serial1);
	xbee.begin(Serial1);
	Serial.println("Ethernet Start");
	// start the Ethernet connection:
	if (Ethernet.begin(mac) == 0) {
		Serial.println("Failed to configure Ethernet using DHCP");
		// no point in carrying on, so do nothing forevermore:
		for (;;)
			;
	}
	led1.on();
	printIPAddress();
	// Setup MySQL Connector
	if (my_conn.mysql_connect(server_addr, 3306, user, password)) {
		led3.blink(100, 10);
	}
	else {
		led3.off();
	}
	firstValue = myString.substring(0, commaIndex);
	secondValue = myString.substring(commaIndex + 1, secondCommaIndex);
	thirdValue = myString.substring(secondCommaIndex + 1, thirdCommaIndex); // To the end of the string
	fourthValue = myString.substring(thirdCommaIndex + 1, fourthCommaIndex); // To the end of the string
	fifthValue = myString.substring(fourthCommaIndex + 1, fifthCommaIndex); // To the end of the string
	Serial.print(firstValue);
	Serial.print(".");
	Serial.print(secondValue);
	Serial.print(".");
	Serial.print(thirdValue);
	Serial.print(".");
	Serial.print(fourthValue);
	Serial.print(".");
	Serial.print(fifthValue);
	Serial.println();
}

void loop() {
	my_wdt_reset();
	checkSQL();// make sure mysql is connected
	xbee.readPacket();
	// check if a packet was received: 
	if (xbee.getResponse().isAvailable()) {
		led2.blink(100, 10);
		led2.on();
		if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
			xbee.getResponse().getZBRxResponse(zbRx);
			// read available message into text
			for (int i = 0; i < zbRx.getDataLength(); i++) {

				text[i] = zbRx.getData(i);
			}
			String myString="";
			for (int i = 0; i < 36; i++) {
				myString += ((char)text[i]);
				delay(10);
			}
			commaIndex = myString.indexOf(',');
			secondCommaIndex = myString.indexOf(',', commaIndex + 1);
			thirdCommaIndex = myString.indexOf(',', secondCommaIndex + 1);
			fourthCommaIndex = myString.indexOf(',', thirdCommaIndex + 1);
			fifthCommaIndex = myString.indexOf(',', fourthCommaIndex + 1);
			// 
			firstValue = myString.substring(0, commaIndex);
			secondValue = myString.substring(commaIndex + 1, secondCommaIndex);
			thirdValue = myString.substring(secondCommaIndex + 1, thirdCommaIndex); // To the end of the string
			fourthValue = myString.substring(thirdCommaIndex + 1, fourthCommaIndex); // To the end of the string
			fifthValue = myString.substring(fourthCommaIndex + 1, fifthCommaIndex); // To the end of the string 
			char charBuf1[firstValue.length() + 1];
			firstValue.toCharArray(charBuf1, (firstValue.length() + 1));
			char charBuf2[secondValue.length() + 1];
			secondValue.toCharArray(charBuf2, (secondValue.length() + 1));
			char charBuf3[thirdValue.length() + 1];
			thirdValue.toCharArray(charBuf3, (thirdValue.length() + 1));
			char charBuf4[fourthValue.length() + 1];
			fourthValue.toCharArray(charBuf4, (fourthValue.length() + 1));
			char charBuf5[fifthValue.length() + 1];
			fifthValue.toCharArray(charBuf5, (fifthValue.length() + 1));
			//
			String ComparePacketType = charBuf2;
			InsertSQL01(charBuf1, charBuf2, charBuf3, charBuf4, charBuf5);
			sendPrevTimeSQL = millis(); //reset report period timer
			sendPrevTime = millis(); //reset report period timer
				}
		}
	delay(500);
}

// each send data works slightly differently as each node submitts different readings but all data gets stored in the same table.
// all unused fields are simply filled with 0s



void ClearMYSQL() // Clear Buffers to Free Memory
{
	my_conn.free_columns_buffer();
	my_conn.free_row_buffer();
}


void InsertSQL01(char charBuf1[], char charBuf2[], char charBuf3[], char charBuf4[], char charBuf5[])
{
	if (my_conn.is_connected() == 1) {
		led3.blink(100, 10);
		// MySQL insert
		char query[128];
		// char node_id[10];
		// char data_value[10];
		// dtostrf(E, 1, 1, node_id);
		sprintf(query, INSERT_DATA_ID, charBuf1);
		my_conn.cmd_query(query);
		led3.blink(100, 10);
		sprintf(query, SELECT_LAST_INSERT, charBuf1);
		my_conn.cmd_query(query);
		// show Last Insert Value
		// EXAMPLE 3
		// SELECT query for lookup value (1 row returned)
		// Here we get a value from the database and use it.
		long I_D = 0;
		// We ignore the columns but we have to read them to get that data out of the queue
		my_conn.get_columns();
		// Now we read the rows.
		row_values *row = NULL;
		row = my_conn.get_next_row();
		// We use the first value returned in the row - population of NYC!
		if (row != NULL) {
			I_D = atol(row->values[0]);
		}
		ClearMYSQL;
		String ComparePacketType = charBuf2;
		if (ComparePacketType == typeTnV){
			sprintf(query, INSERT_DATA_T, charBuf3, I_D);
			my_conn.cmd_query(query);
			led3.blink(100, 10);
			sprintf(query, INSERT_DATA_V, charBuf4, I_D);
			my_conn.cmd_query(query);
			led3.blink(100, 10);
			my_conn.clear_ok_packet(); // ensures select works correctly
		}

		if (ComparePacketType == typeGPS){
			sprintf(query, INSERT_DATA_L, charBuf3, I_D);
			my_conn.cmd_query(query);
			led3.blink(100, 10);
			sprintf(query, INSERT_DATA_G, charBuf4, I_D);
			my_conn.cmd_query(query);
			led3.blink(100, 10);
			my_conn.clear_ok_packet(); // ensures select works correctly
		}

		if (ComparePacketType == typeSMR){
			sprintf(query, INSERT_DATA_S1, charBuf3, I_D);
			my_conn.cmd_query(query);
			led3.blink(100, 10);
			sprintf(query, INSERT_DATA_S2, charBuf4, I_D);
			my_conn.cmd_query(query);
			led3.blink(100, 10);
			sprintf(query, INSERT_DATA_S3, charBuf5, I_D);
			my_conn.cmd_query(query);
			led3.blink(100, 10);
			my_conn.clear_ok_packet(); // ensures select works correctly
		}

		ClearMYSQL;
		my_conn.clear_ok_packet();
	}
	else
	{
		led3.off();
	}
}



void checkSQL()
{
	// Serial.println("WD Disabled - Checking SQL Connection");
	if (my_conn.is_connected()) {
		led3.blink(200,2);
/*		Serial.println("Executing SQL Connection Now");
		wdt_enable(WDTO_8S);
		my_conn.cmd_query(QUERY_SQL);
		wdt_disable();
		my_conn.show_results();
		delay(1000);
		Serial.println("Checking Free RAM");
		Serial.print(" RAM: ");
		Serial.println(get_free_memory());
		wdt_enable(WDTO_8S);
		Serial.println("WD Enabled = 8's");
		Serial.println("Back to Main Loop");
		*/
		num_fails = 0;
		led3.on();
	}
	else {
		led3.off();
		ClearMYSQL();
		my_conn.disconnect();
		Serial.println("Connecting...");
		if (my_conn.mysql_connect(server_addr, 3306, user, password)) {
			delay(500);
			Serial.println("Success!");
			num_fails = 0;
		}
		else {
			num_fails++;
			Serial.println("Connect failed!");
			if (num_fails == MAX_FAILED_CONNECTS) {
				Serial.println("Ok, that's it. I'm outta here. Rebooting...");
				delay(2000);
				NoActivityReset();

			}
		}
	}
}
void NoActivityReset()
{

	Serial.println("Reboot - Cant Write SQL or Recieve XBEE");
	delay(1000);
	resetFunc();  //call reset if no data sent to exosite for 1/2 hour
}
/*
void PingIT()
{
	if (I == 60000){
	
		bool pingRet; // pingRet stores the ping() success (true/false)
		ICMPPing ping(pingSocket);
		wdt_reset();
		pingRet = ping(4, pingAddr, buffer);
		if (pingRet) { // Failure
			Serial.println(buffer);
			I = 0;
			// wdt_reset();
		}
		else{
		wdt_reset();
		resetFunc();  //call reset 
	}
	}
	I++;
}
*/
void printIPAddress()
{
	Serial.print("My IP address: ");
	for (byte thisByte = 0; thisByte < 4; thisByte++) {
		// print the value of each byte of the IP address:
		Serial.print(Ethernet.localIP()[thisByte], DEC);
		Serial.print(".");
	}

	Serial.println();
}

