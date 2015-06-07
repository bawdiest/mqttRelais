#include <Adafruit_CC3000.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#define LOGGING_FREQ_SECONDS   88  // Seconds to wait before a new sensor reading is logged.
#define MAX_SLEEP_ITERATIONS   LOGGING_FREQ_SECONDS / 8  // Number of times to sleep (for 8 seconds) before
int sleepIterations = 0;

int maxLoopIterations;
// a sensor reading is taken and sent to the server.
// Don't change this unless you also change the 
// watchdog timer configuration.

#define WLAN_SSID       "bawdiestNet"
#define WLAN_PASS       "nkensleg8"
#define WLAN_SECURITY WLAN_SEC_WPA2 

#define MQTT_TOPIC "BalconyWaterPump/%s"

#define ADAFRUIT_CC3000_IRQ   3
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10

#define REG_SHUTDOWN_PIN       9
#define RELAIS                 4

Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS,
ADAFRUIT_CC3000_IRQ,
ADAFRUIT_CC3000_VBAT,
SPI_CLOCK_DIVIDER);

Adafruit_CC3000_Client cc3000_client = Adafruit_CC3000_Client();
PubSubClient client("mikmak.cc", 1883, mqtt_callback, cc3000_client);


// replace xxxxxxxxx with your Adafruit IO key
#define AIO_KEY "xxxxxxxxx"


volatile bool watchdogActivated = false;

// watchdog interrupt
ISR (WDT_vect) 
{
  watchdogActivated = true;
}  // end of WDT_vect

void setup() {
  Serial.begin(115200);
  Serial.println(F("Initializing....(May take a few seconds)"));

  // Configure digital output connected to 3.3V regulator shutdown.
  pinMode(REG_SHUTDOWN_PIN, OUTPUT);
  delay(100);
  digitalWrite(REG_SHUTDOWN_PIN, LOW);
  delay(100);
  pinMode(RELAIS, OUTPUT);
  delay(100);
  digitalWrite(RELAIS, LOW);
  delay(100);


  // Disable CC3000 now that it is initialized.
  //shutdownWiFi();




  setMode(1); //Set Power Saving

  Serial.println(F("Setup complete."));
}

//**************************************
//======================================
// Loop
//--------------------------------------

void loop() {
  wdt_enable(WDTO_8S);

  digitalWrite(REG_SHUTDOWN_PIN, HIGH);
  Serial.println(F("Power on WiFi..."));
  delay(100);
  // Initialize the CC3000.
  Serial.println(F("\nInitializing CC3000..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  }

  wdt_reset();

  enableWiFi();
  // reconnect if we lost connection to AIO

  if(! client.connected()) {
    Serial.print(F("AIO connection dropped. Attempting reconnect"));
    delay(100);
    mqtt_connect();
    delay(3000);
  }

  wdt_reset();
  int loopIterations = 0;

  while(1) {
    wdt_reset();

    Serial.print(maxLoopIterations);
    delay(100);
    Serial.print(loopIterations);
    delay(100);

    if(maxLoopIterations != 0) {
      if (loopIterations >= maxLoopIterations) {
        Serial.print(F("Maximum Loops Exceeded!"));
        break;
      }
    }

    if(!client.connected()) {
      Serial.print(F("Connection to MQTT Broker lost!"));
      delay(100);
      break;
    }

    // required for MQTT connection
    client.loop();
    delay(100);

    sendMessage("InputVoltage", convertInt2Char(readVCC()));
    delay(100);
    loopIterations += 1;
  }
  client.disconnect();
  delay(100);

  shutdownWiFi();
  Serial.println(F("\nSleep first..."));
  delay(100);
  sleepLoop();

  digitalWrite(RELAIS, LOW);

  //  Serial.print(F("Waiting for Reset..."));
  //  while(1) {
  //
  //  }; //Do not reset WDT, so that Interrupt appear, which will cause rebooting Arduino
}    

//======================================
// Loop Functions
//--------------------------------------

void sleepLoop() {

  // Setup the watchdog timer to run an interrupt which
  // wakes the Arduino from sleep every 8 seconds.

  // Note that the default behavior of resetting the Arduino
  // with the watchdog will be disabled.

  // This next section of code is timing critical, so interrupts are disabled.
  // See more details of how to change the watchdog in the ATmega328P datasheet
  // around page 50, Watchdog Timer.
  noInterrupts();

  // Set the watchdog reset bit in the MCU status register to 0.
  MCUSR &= ~(1<<WDRF);

  // Set WDCE and WDE bits in the watchdog control register.
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  // Set watchdog clock prescaler bits to a value of 8 seconds.
  WDTCSR = (1<<WDP0) | (1<<WDP3);

  // Enable watchdog as interrupt only (no reset).
  WDTCSR |= (1<<WDIE);

  // Enable interrupts again.
  interrupts();


  Serial.println(F("\nstart sleep loop..."));
  delay(100);
  // Don't do anything unless the watchdog timer interrupt has fired.
  if (watchdogActivated)
    Serial.println(F("\nwatchdog is activated..."));
  delay(100);
  {
    watchdogActivated = false;

    while(1) {
      // Increase the count of sleep iterations and take a sensor
      // reading once the max number of iterations has been hit.
      sleepIterations += 1;
      if (sleepIterations >= MAX_SLEEP_ITERATIONS) {
        // Reset the number of sleep iterations.
        sleepIterations = 0;
        break;
        // Log the sensor data (waking the CC3000, etc. as needed)
      }
      sleep();
    }
  }
}


//======================================
// MQTT Functions
//--------------------------------------
void mqtt_callback (char* topic, byte* payload, unsigned int length) {

  // convert the MQTT payload to int
  int value = to_int(payload, length);

  // make sure we have a 0 or 1
  //sendMessage("actionLog", m);
  sendMessage("actionLog", convertInt2Char(value));

  if (strcmp (topic,"BalconyWaterPump/Relais") == 0) turnWaterPump(value);
  if (strcmp (topic,"BalconyWaterPump/RelaisState") == 0) sendRelaisState();
  if (strcmp (topic,"BalconyWaterPump/PowerMode") == 0) setMode(value);

  // dump topic and payload from subscriptions
  //  Serial.print(F("Received: "));
  //  Serial.println(topic);
  //  Serial.write(payload, length);
  //  Serial.println(topic);

}

void mqtt_connect() {
  char client_name[30];
  // generate new client name
  //sprintf(client_name, "adafruit-cc3000-%ul", micros());
  // attempt connection
  if (client.connect("arduinoClient")) {
    sendMessage("actionLog","hello world");
    client.subscribe("BalconyWaterPump/Relais");
    client.subscribe("BalconyWaterPump/PowerMode");
    client.subscribe("BalconyWaterPump/RelaisState");
  } 
  else {
  }
}

bool sendMessage(char *feed, char *value) {

  // allows for ~60 char feed name
  char topic[85];

  // build mqtt topic
  sprintf(topic, MQTT_TOPIC, feed);

  // push data
  client.publish(topic, value);
  delay(1000);

}

void sendRelaisState() {
  sendMessage("RelaisStatus", convertInt2Char(digitalRead(RELAIS)));
}

//======================================
// Control Functions
//--------------------------------------

void turnWaterPump(unsigned int s) {
  if(s != 0 && s != 1) return;
  digitalWrite(RELAIS, s == 1 ? HIGH : LOW);
}

void turnWaterPumpOn(unsigned long sec) {
  sendMessage("actionLog","Turn Relais ON");
  digitalWrite(RELAIS, HIGH);
  for(uint8_t i=0; i<sec; i++) {
    delay(1000);
  }
  digitalWrite(RELAIS, LOW);
  sendMessage("actionLog","Turn Relais OFF");
}

void setMode(int mode) {
  if(mode == 0) {
    Serial.print(F("Entering Online Mode ..."));
    maxLoopIterations = 0; //Online Mode
  }
  if(mode == 1) {
    Serial.print(F("Entering PowerSaving Mode ..."));
    maxLoopIterations = 16; //PowerSaving Mode
  }
}

// Put the Arduino to sleep.
void sleep()
{
  // Set sleep to full power down.  Only external interrupts or 
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Disable the ADC while asleep.
  power_adc_disable();

  // Enable sleep.
  sleep_enable();

  // Disable brown-out detection during sleep.  This is timing critical and
  // must be done right before entering sleep mode.
  MCUCR |= (1<<BODS) | (1<<BODSE);
  MCUCR &= ~(1<<BODSE);

  // Enter sleep mode.
  sleep_cpu();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point.

  // When awake, disable sleep mode and turn on all devices.
  sleep_disable();
  power_all_enable();
}

// Disconnect from wireless network and shut down the CC3000.
void shutdownWiFi() {
  // Disconnect from the AP.
  // This might not be strictly necessary, but I found
  // it was sometimes difficult to quickly reconnect to
  // my AP if I shut down the CC3000 without first
  // disconnecting from the network.
  if (cc3000.checkConnected()) {
    cc3000.disconnect();
  }

  // Wait for the CC3000 to finish disconnecting before
  // continuing.
  while (cc3000.checkConnected()) {
    delay(100);
  }

  // Shut down the CC3000.
  wlan_stop();

  // Turn off the 3.3V voltage regulator powering the CC3000.
  digitalWrite(REG_SHUTDOWN_PIN, LOW);
  delay(100);

  Serial.println(F("CC3000 shut down.")); 
}

// Enable the CC3000 and connect to the wifi network.
// Return true if enabled and connected, false otherwise.
boolean enableWiFi() {
  Serial.println(F("Turning on CC3000."));

  // Turn on the 3.3V regulator and wait a small period for the voltage
  // to stabilize before continuing.
  digitalWrite(REG_SHUTDOWN_PIN, HIGH);
  delay(100); 

wdt_reset();
  // Turn on the CC3000.
  wdt_disable();
  wlan_start(0);
  wdt_enable(WDTO_8S);

  // Connect to the AP.
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    return false;
  }
  Serial.println(F("Connected!"));

  // Wait for DHCP to be complete.
  Serial.println(F("Request DHCP"));
  int attempts = 0;
  while (!cc3000.checkDHCP())
  {
    wdt_reset();
    if (attempts >= 20) {
      Serial.println(F("DHCP didn't finish!"));
      return false;
    }
    attempts += 1;
    Serial.println(F("Attempt Nr.: "));
    Serial.println(attempts);
    delay(1000);
  }

  // Return success, the CC3000 is enabled and connected to the network.
  return true;
}

//======================================
// Help Functions
//--------------------------------------

char* convertInt2Char(int i) {
  char stringTemperatureF[10];
  dtostrf(i, 0, 3, stringTemperatureF);
}

long readVCC() {
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

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

  //  result = result / 100;
  //  result = result * 5;

  return result; // Vcc in percent
}

int to_int(byte* payload, int length) {

  int i;
  char val[10];

  // convert payload to char
  for(i = 0; i < length; i++) {
    val[i] = payload[i];
  }

  val[i] = '\0';

  // convert char to int and return
  return atoi(val);

}















