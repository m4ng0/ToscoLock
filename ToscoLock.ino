/*
  ToscoEmiliano 2.0
  Elettroserratura comandata da Arduino (EZControl.it board)
*/

// constants won't change. They're used here to set pin numbers:
const int BUTTON_PIN = 5;     // the number of the pushbutton pin
const int RELAY_PIN =  7;      // the number of the relay pin

const float ARDUINO_VOLTAGE = 3.3;  // EZControl board uses 3.3V instead of 5V

const int RELAY_CLOSED_STATE_DELAY = 500;

// variables will change:
int buttonState;         // variable for reading the pushbutton status
int previousButtonState = HIGH;
bool isPushing = false;

// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long timeButtonStateLow = 0;         // the last time the output pin was toggled
long debounce = 100;   // the debounce time, increase if the output flickers


void setup() {
  Serial.begin(9600);
  // initialize the relay pin as an output:
  pinMode(RELAY_PIN, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH);  // enable pullup
  Serial.println("Setup completato");
}

float convertTMP36Input(int rawAnalogicInput) {
  float voltage = rawAnalogicInput*ARDUINO_VOLTAGE;
  voltage /= 1024;
  return (voltage - 0.5)*100; // converting from 10 mv per degree to degrees ((voltage - 500mV) timesM
}

void loop() {
  //delay(200);
  // read the state of the pushbutton value:
  buttonState = digitalRead(BUTTON_PIN);

//  Serial.println(buttonState);
//  Serial.println(convertTMP36Input(analogRead(3)));

  // check if the pushbutton is pressed. If it is, the buttonState is LOW:
  if (isPushing || buttonState == LOW && previousButtonState == HIGH) {
    timeButtonStateLow = millis();
  }
  if (buttonState == LOW && millis() - timeButtonStateLow > debounce) {
    isPushing = true;
    Serial.println("pushed!");
    Serial.println(convertTMP36Input(analogRead(3)));
    // close relay contact:
    digitalWrite(RELAY_PIN, HIGH);
    delay(RELAY_CLOSED_STATE_DELAY);
    // reopen relay contact
    digitalWrite(RELAY_PIN, LOW);
  }
  if (buttonState == HIGH) {
    isPushing = false;
    // open relay contact:
    digitalWrite(RELAY_PIN, LOW);
  }
  previousButtonState = buttonState;
}
