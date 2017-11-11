/*
  ToscoEmiliano 2.0
  Elettroserratura comandata da Arduino (EZControl.it board)
*/
#include <PubSubClient.h>

#include <Adafruit_CC3000.h>
#include <ccspi.h>

// Interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   2
#define ADAFRUIT_CC3000_VBAT  8
#define ADAFRUIT_CC3000_CS    10

// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY WLAN_SEC_WPA2

#define DOORLOCK_OPEN_FEED_PATH "doorlock/open"
#define DOORLOCK_LOG_FEED_PATH "doorlock/log"
#define TEMPERATURE_FEED_PATH "doorlock/temperature"

// Constants: pin numbers
const int BUTTON_PIN = 5;     // the number of the pushbutton pin
const int RELAY_PIN =  7;      // the number of the relay pin

const float ARDUINO_VOLTAGE = 3.3;  // EZControl board uses 3.3V instead of 5V

const int RELAY_CLOSED_STATE_DURATION = 500;  // relay closed state duration in ms

// variables
int buttonState;         // variable for reading the pushbutton status
int previousButtonState = HIGH;
bool isPushing = false;

// the follow variables are longs because the time, measured in miliseconds,
// will quickly become a huge number than can't be stored in an int.
long timeButtonStateLow = 0;         // the last time the output pin was toggled
long debounce = 100;   // the debounce time, increase if the output flickers

const unsigned long AP_CONNECT_TIMEOUT = 12L * 1000L; // max time to wait for AP Connection
const unsigned long DHCP_TIMEOUT = 12L * 1000L; // max time to wait for DHCP
const unsigned long DISPLAY_DETAILS_TIMEOUT = 4L * 1000L; // max time to wait for DHCP

long timeLastMqttReconnectRetry = 0; // the last time we tried to reconnect to mqtt
char MQTT_CLIENT_ID[] = "ArduinoDoorlock";
char temperatureMessage[10];
long timeLastTemperatureMessage = 0; // the last time we sent a temperature message
char logMessage[10];

// Use hardware SPI for the remaining pins (on an UNO, SCK = 13, MISO = 12, and MOSI = 11)
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT, SPI_CLOCK_DIVIDER);

Adafruit_CC3000_Client client = Adafruit_CC3000_Client();

/* ---------------- */
/* WIFI CREDENTIALS */
/* ---------------- */
char WLAN_SSID[] = "<YOUR-SSID>";      // * WiFi network name (cannot be longer than 32 characters)
char WLAN_PASS[] = "<YOUR-PASSWORD>";  // * WiFi password (leave it empty on open networks)

// MQTT callback function header
void mqtt_callback (char* topic, byte* payload, unsigned int length);

IPAddress server(10, 1, 168, 192);  // Important: note the "reverse" ordering!

PubSubClient mqttclient(server, 1883, mqtt_callback, client);

void setup() {
  Serial.begin(57600);

  // initialize the relay pin as an output:
  pinMode(RELAY_PIN, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH);  // enable pullup

  Serial.println(F("[CC3000] Hi there"));
  delay(500);

  Serial.println(F("[CC3000] Init the WiFi connection"));
  if (!cc3000.begin()) {
    Serial.println(F("[CC3000] Fail init CC3000"));
    for(;;);
  }

  /*Serial.println(F("[CC3000] Deleting old profiles"));
  if (!cc3000.deleteProfiles()) {
    Serial.println(F("[CC3000] Fail deleting old profiles"));
    while(1);
  }*/

  char *ssid = WLAN_SSID;
  Serial.print(F("[CC3000] Setting SSID to "));
  Serial.println(ssid);
  Serial.print(F("\n"));

  // (note: secure connections are not available in 'Tiny' mode)
  doConnectToAP();

  doDHCP();

  /* Display the IP address DNS, Gateway, etc. */
  doDisplayConnectionDetails();

  timeLastMqttReconnectRetry = 0;

  Serial.println("Setup completato");
}

float convertTMP36Input(int rawAnalogicInput) {
  float voltage = rawAnalogicInput*ARDUINO_VOLTAGE;
  voltage /= 1024;
  return (voltage - 0.5)*100; // converting from 10 mv per degree to degrees ((voltage - 500mV) timesM
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
boolean mqtt_reconnect() {
  if (mqttclient.connect(MQTT_CLIENT_ID)) {
    Serial.println(F("MQTT Connected"));
    // (re)subscribe to feed
    mqttclient.subscribe(DOORLOCK_OPEN_FEED_PATH);
  }
  return mqttclient.connected();
}

/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/**************************************************************************/
bool displayConnectionDetails(void) {
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;

  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv)) {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  } else {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}

void close_relay() {
  // close relay contact:
  digitalWrite(RELAY_PIN, HIGH);
  delay(RELAY_CLOSED_STATE_DURATION);
  // reopen relay contact
  digitalWrite(RELAY_PIN, LOW);
  String(millis(), DEC).toCharArray(logMessage, 10);
  mqttclient.publish(DOORLOCK_LOG_FEED_PATH, logMessage);
}

void mqtt_callback (char* topic, byte* payload, unsigned int length) {
  Serial.println(topic);
  Serial.write(payload, length);
  Serial.println("");
  if (strcmp(topic, DOORLOCK_OPEN_FEED_PATH) == 0) {
    close_relay();
  }
}

void loop() {
  //delay(200);
  // read the state of the pushbutton value:
  buttonState = digitalRead(BUTTON_PIN);

  //Serial.println(buttonState);

  // check if the pushbutton is pressed. If it is, the buttonState is LOW:
  if (isPushing || buttonState == LOW && previousButtonState == HIGH) {
    timeButtonStateLow = millis();
  }
  if (buttonState == LOW && millis() - timeButtonStateLow > debounce) {
    isPushing = true;
    Serial.println("pushed!");
    Serial.println(convertTMP36Input(analogRead(3)));
    close_relay();
  }
  if (buttonState == HIGH) {
    isPushing = false;
    // open relay contact:
    digitalWrite(RELAY_PIN, LOW);
  }
  previousButtonState = buttonState;

  if (!mqttclient.connected()) {
    long now = millis();
    if (now - timeLastMqttReconnectRetry > 5000) {
      Serial.println("Attempting MQTT connection");
      timeLastMqttReconnectRetry = now;
      // try to reconnect
      if (mqtt_reconnect()) {
        timeLastMqttReconnectRetry = 0;
      }
    }
  } else {
    // mqtt client is connected
    mqttclient.loop();
  }

  long now = millis();
  if (now - timeLastTemperatureMessage > 60000) { // we send a temperature message every minute
    timeLastTemperatureMessage = now;
    //snprintf(temperatureMessage, 10, "%5.2f", convertTMP36Input(analogRead(3)));
    dtostrf(convertTMP36Input(analogRead(3)), 10, 2, temperatureMessage);
    mqttclient.publish(TEMPERATURE_FEED_PATH, temperatureMessage);
  }

}

void doConnectToAP() {
  Serial.println(F("[CC3000] Connecting to AP"));
  unsigned long start = millis();
  // (note: secure connections are not available in 'Tiny' mode)
  while (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY) && (millis() - start < AP_CONNECT_TIMEOUT)) {
    Serial.println(F("[CC300] Connection failed"));
    delay(100);
  }
}

void doDHCP() {
    /* Wait for DHCP to complete */
  Serial.println(F("[CC3000] Setting DHCP"));
  unsigned long start = millis();
  while (!cc3000.checkDHCP() && (millis() - start < DHCP_TIMEOUT)) {
    delay(100);
  }
}

void doDisplayConnectionDetails() {
  unsigned long start = millis();
  while(!displayConnectionDetails() && (millis() - start < DISPLAY_DETAILS_TIMEOUT)) {
    delay(500);
  }
}
