/*
  ToscoEmiliano 2.0
  Elettroserratura comandata da Arduino (EZControl.it board)
*/

#include <MyPubSubClient.h>

#include <Adafruit_CC3000.h>
#include <ccspi.h>

#include <utility/netapp.h>

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
const int BUTTON_PIN = 5;  // the number of the pushbutton pin
const int RELAY_PIN =  7;  // the number of the relay pin
const int LED_PIN = 9;     // the number of the LED pin for status

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

const unsigned long DELAY_FOR_FIRST_CONNECTION = 1L * 60L * 1000L; // time to wait before attempting a wifi connection
boolean wifiSetup = false;  // we try to setup wifi for the first time after the delay set above
const unsigned long AP_CONNECT_TIMEOUT = 20L * 1000L; // max time to wait for AP Connection
const unsigned long DHCP_TIMEOUT = 25L * 1000L; // max time to wait for DHCP
const unsigned long STATIC_IP_TIMEOUT = 5L * 1000L; // max time to wait for setting static IP
bool staticIpConfSet = false;
const unsigned long DISPLAY_DETAILS_TIMEOUT = 4L * 1000L; // max time to wait for DHCP
const unsigned long TEMPERATURE_READ_EVERY = 1L * 60L * 1000L; // frequency we read the temperature and send through MQTT

unsigned long timeLastWiFiConnectRetry = 0; // the last time we tried to reconnect to AP
unsigned long wifiConnectConsecutiveFailures = 0; // how many wifi connection issues in a row
unsigned long timeLastMqttReconnectRetry = 0; // the last time we tried to reconnect to mqtt
unsigned long mqttConnectConsecutiveFailures = 0; // how many mqtt connection issues in a row
char MQTT_CLIENT_ID[] = "ArduinoDoorlock";
char temperatureMessage[10];
unsigned long timeLastTemperatureMessage = 0; // the last time we sent a temperature message
char logMessage[10];
unsigned long theNow;
unsigned long lastWifiConnectLoop = 0;
unsigned long lastMqttConnectLoop = 0;

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
  Serial.begin(115200);

  // initialize the relay pin as an output:
  pinMode(RELAY_PIN, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH);  // enable pullup
  // initialize the LED pin
  pinMode(LED_PIN, OUTPUT);

  Serial.println(F("[CC3000] Hi there"));
  //delay(500);

  /*Serial.println(F("[CC3000] Deleting old profiles"));
  if (!cc3000.deleteProfiles()) {
    Serial.println(F("[CC3000] Fail deleting old profiles"));
    while(1);
  }*/

  //char *ssid = WLAN_SSID;
  Serial.print(F("[CC3000] Setting SSID to "));
  Serial.println(WLAN_SSID);
  Serial.print(F("\n"));

  //setupWiFiConnection();

  timeLastWiFiConnectRetry = DELAY_FOR_FIRST_CONNECTION;
  timeLastMqttReconnectRetry = DELAY_FOR_FIRST_CONNECTION;

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
  Serial.println(F("mqtt_reconnect called"));
  if (mqttclient.connect(MQTT_CLIENT_ID)) {
    Serial.println(F("MQTT Connected"));
    // (re)subscribe to feed
    mqttclient.subscribe(DOORLOCK_OPEN_FEED_PATH);
  } else {
    Serial.println(F("mqtt_reconnect NOT CONNECTED"));
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
  if (wifiSetup) {
    String(millis(), DEC).toCharArray(logMessage, 10);
    mqttclient.publish(DOORLOCK_LOG_FEED_PATH, logMessage);
  }
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
  //Serial.println(F("luppamelo"));
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

  if (millis() > DELAY_FOR_FIRST_CONNECTION) {
    if (!wifiSetup) {
      wifiSetup = true;
      setupWiFiConnection();
    }

    theNow = millis();
    if (theNow - lastWifiConnectLoop > 300) {
      lastWifiConnectLoop = theNow;
          if (!cc3000.checkConnected()) {
      //Serial.println("CC3000 not connected!");
      unsigned long now = millis();
      if (now - timeLastWiFiConnectRetry > calculateWiFiRetryFrequency(wifiConnectConsecutiveFailures)) {
        wifiConnectConsecutiveFailures++;
        Serial.println(F("[CC300] Reconnecting to AP"));
        timeLastWiFiConnectRetry = now;
        cc3000.reboot(); // is this really necessary?
        setupWiFiConnection();
      }
    } else {
      wifiConnectConsecutiveFailures = 0;
      timeLastWiFiConnectRetry = 0;;
    }
    }

  theNow = millis();
  if (theNow - lastMqttConnectLoop > 100) {
    lastMqttConnectLoop = theNow;
    if (!mqttclient.connected()) {
      digitalWrite(LED_PIN, LOW);
      unsigned long now = millis();
      if (now - timeLastMqttReconnectRetry > calculateMqttRetryFrequency(mqttConnectConsecutiveFailures)) {
        mqttConnectConsecutiveFailures++;
        Serial.println("Attempting MQTT connection");
        timeLastMqttReconnectRetry = now;
        // try to reconnect
        if (mqtt_reconnect()) {
          Serial.println("Reconnecting to MQTT connection: SUCCESS");
          mqttConnectConsecutiveFailures = 0;
          timeLastMqttReconnectRetry = 0;
        } else {
          Serial.println("Reconnecting to MQTT connection: FAILURE");
        }
      }
    } else {
      digitalWrite(LED_PIN, HIGH);
      // mqtt client is connected
      mqttclient.loop();
    }
  }

    unsigned long now = millis();
    if (now - timeLastTemperatureMessage > TEMPERATURE_READ_EVERY) { // we send a temperature message every minute
       Serial.println("Reading temperature");
      timeLastTemperatureMessage = now;
      //snprintf(temperatureMessage, 10, "%5.2f", convertTMP36Input(analogRead(3)));
      dtostrf(convertTMP36Input(analogRead(3)), 10, 2, temperatureMessage);
      mqttclient.publish(TEMPERATURE_FEED_PATH, temperatureMessage);
    }
  }
}

bool doConnectToAP() {
  Serial.println(F("[CC3000] Connecting to AP"));
  unsigned long start = millis();
  bool connectedToAp = false;
  // (note: secure connections are not available in 'Tiny' mode)
  while (connectedToAp = cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY), (!connectedToAp && (millis() - start < AP_CONNECT_TIMEOUT))) {
    Serial.println(F("[CC300] Connection failed"));
    delay(100);
  }
  return connectedToAp;
}

bool doDHCP() {
    /* Wait for DHCP to complete */
  Serial.println(F("[CC3000] Setting DHCP"));
  unsigned long start = millis();
  bool dhcpOk = false;
  while (dhcpOk = cc3000.checkDHCP(), (!dhcpOk && (millis() - start < DHCP_TIMEOUT))) {
    Serial.println(F("[CC300] DHCP Connection failed"));
    delay(400);
  }
  return dhcpOk;
}

/*void doSetStaticIP() {
  if (!staticIpConfSet) {
    Serial.println(F("[CC3000] Setting Static IP"));
    uint32_t ipAddress = cc3000.IP2U32(192, 168, 1, 22);
    uint32_t netMask = cc3000.IP2U32(255, 255, 255, 0);
    uint32_t defaultGateway = cc3000.IP2U32(192, 168, 1, 1);
    uint32_t dns = cc3000.IP2U32(192, 168, 1, 1);
    unsigned long start = millis();
    bool staticIpConfSet = false;
    while (staticIpConfSet = cc3000.setStaticIPAddress(ipAddress, netMask, defaultGateway, dns), (!staticIpConfSet && (millis() - start < STATIC_IP_TIMEOUT))) {
      Serial.println(F("Failed to set static IP!"));
      delay(200);
    }
  }
}*/

  /* Optional: Set a static IP address instead of using DHCP.
     Note that the setStaticIPAddress function will save its state
     in the CC3000's internal non-volatile memory and the details
     will be used the next time the CC3000 connects to a network.
     This means you only need to call the function once and the
     CC3000 will remember the connection details.  To switch back
     to using DHCP, call the setDHCP() function (again only needs
     to be called once).
  */
  /*
  uint32_t ipAddress = cc3000.IP2U32(192, 168, 1, 19);
  uint32_t netMask = cc3000.IP2U32(255, 255, 255, 0);
  uint32_t defaultGateway = cc3000.IP2U32(192, 168, 1, 1);
  uint32_t dns = cc3000.IP2U32(8, 8, 4, 4);
  if (!cc3000.setStaticIPAddress(ipAddress, netMask, defaultGateway, dns)) {
    Serial.println(F("Failed to set static IP!"));
    while(1);
  }
  */
  /* Optional: Revert back from static IP addres to use DHCP.
     See note for setStaticIPAddress above, this only needs to be
     called once and will be remembered afterwards by the CC3000.
  */
  /*
  if (!cc3000.setDHCP()) {
    Serial.println(F("Failed to set DHCP!"));
    while(1);
  }
  */

void doDisplayConnectionDetails() {
  unsigned long start = millis();
  while(!displayConnectionDetails() && (millis() - start < DISPLAY_DETAILS_TIMEOUT)) {
    Serial.println(F("Waiting for connection detail"));
    delay(500);
  }
}

void setupWiFiConnection() {
  Serial.println(F("[CC3000] Init the WiFi connection"));
  if (!cc3000.begin()) {
    Serial.println(F("[CC3000] Fail init CC3000"));
    return;
  }
  //doSetStaticIP(); // alternative to DHCP

  /*if (!cc3000.setDHCP()) {
    Serial.println(F("Failed to set DHCP!"));
    while(1);
  }*/

  if (doConnectToAP()) {
    if (doDHCP()) {
      doSetTimeoutToMinimum();
      /* Display the IP address DNS, Gateway, etc. */
      doDisplayConnectionDetails();
    }
  }
}

void doSetTimeoutToMinimum() {
  // Texas Instruments wrote 20 seconds in the firmware as a minimum value, so we have to face it
  unsigned long aucDHCP = 14400;
  unsigned long aucARP = 3600;
  unsigned long aucKeepalive = 20;
  unsigned long aucInactivity = 20;
  if (netapp_timeout_values(&aucDHCP, &aucARP, &aucKeepalive, &aucInactivity) != 0) {
    Serial.println("Error setting inactivity timeout!");
  }
}

unsigned long calculateWiFiRetryFrequency(unsigned long failures) {
  if (failures < 2) {
    return 1L * 60L * 1000L;  // 1 minute
  }
  if (failures < 10) {
    return 2L * 60L * 1000L;  // 2 minutes
  }
  return 10L * 60L * 1000L;  // 10 minutes
}

unsigned long calculateMqttRetryFrequency(unsigned long failures) {
  if (failures <= 3) {
    return failures * 30L * 1000L; // 30*failures seconds
  }
  if (failures < 10) {
    return 5L * 60L * 1000L;  // 5 minutes
  }
  return 10L * 60L * 1000L;  // 10 minutes
}
