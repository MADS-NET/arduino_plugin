#include <ArduinoJson.h>
#define VERSION "1.1.0"
#define BAUD_RATE 115200
// Digital inputs
#define DI1 9
#define DI2 10
#define DI3 11
#define DI4 12

// Main lood delay: this is typically a small value, used to save power
#define DELAY 40UL     // microseconds
// Sampling period: this is the time between samples, must be larger than DELAY
#define TIMESTEP 160UL // milliseconds
// Topmost field in the output JSON
#define DATA_FIELD "data"

JsonDocument Doc;
String Out;
volatile bool Changed = false, MarkerIn = false;
String Source = "";
unsigned long DebounceDelay = 500;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  Serial.print("# Starting JSON reader v" VERSION "\n");
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DI1, INPUT);
  pinMode(DI2, INPUT_PULLUP);
  pinMode(DI3, INPUT_PULLUP);
  pinMode(DI4, INPUT);
  attachInterrupt(digitalPinToInterrupt(DI1), detect_change_DI1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DI2), detect_change_debounce_DI2, RISING);
  attachInterrupt(digitalPinToInterrupt(DI3), detect_change_debounce_DI3, RISING);
}

void detect_change_DI1() {
  Changed = true;
  Source = String("logging ") + (digitalRead(DI1) ? "On" : "Off");
  MarkerIn = false;
}

void detect_change_debounce_DI2() {
  static unsigned long last = 0;

  if (millis() - last > DebounceDelay) {
    Changed = true;
    last = millis();
    MarkerIn = !MarkerIn;
    Source = String("marker ") + (MarkerIn ? "in" : "out");
  }
}

void detect_change_debounce_DI3() {
  static unsigned long last = 0;
  if (millis() - last > DebounceDelay) {
    Changed = true;
    last = millis();
    Source = "marker";
  }
}

void loop() {
  static unsigned long prev_time = 0;
  static unsigned long timestep_us = TIMESTEP * 1000;
  static unsigned long delay = DELAY;
  static unsigned long count = 0;
  static bool onoff = LOW, pause = false, raw = false;
  unsigned long now = micros();
  static unsigned long v = 0; // accumulator for serial values
  
  // Serial input
  // This allows to change parameters on the fly. Toggle parameters are read
  // as a single character, while commands that take a numeric argument 
  // (integer) follow this convention: 123X, where 123 is the value and X is the
  // command. The commands are always single characters and correspond 
  // to the following switch cases:
  char ch;
  // Read serial in
  if (Serial.available()) {
    ch = Serial.read();
    switch (ch) {
      case '0'...'9': // numeric value accumulator
        v = v * 10 + ch - '0';
        break;
      case 'p': // set timestep_us to the current accumulator value
        timestep_us = constrain(v * 1000, 1000, 1E6);
        v = 0;
        break;
      case 'd': // set delay to the current accumulator value
        delay = constrain(v, 1, timestep_us / 10.0);
        v = 0;
        break;
      case 'b':
        DebounceDelay = constrain(v, 0, 5000);
        break;
      case 'x': // toggle pause mode
        pause = !pause;
        break;
      case 'r': // toggle raw mode
        raw = !raw;
        break;
      case '?': // provide inline help
        Serial.print("Version: " VERSION "\n");
        Serial.print("Usage:\n");
        Serial.print("- 10p  set sampling period to 10 milliseconds (now ");
        Serial.print(timestep_us / 1000);
        Serial.print(" ms)\n- 30d  set loop delay to 30 microseconds (now ");
        Serial.print(delay);
        Serial.print(" us)\n- 500b  set debounce delay to 500 milliseconds (now ");
        Serial.print(DebounceDelay);
        Serial.print(" ms)\n");
        Serial.print("- x    toggle pause\n");
        Serial.print("- r    toggle raw output\n");
        break;
      default:
        v = 0;
    }
  }

  if (pause) return;

  if (now - prev_time >= timestep_us) {
    digitalWrite(LED_BUILTIN, onoff);
    onoff = !onoff;
    if (Changed) {
      Doc.clear();
      Doc["millis"] = millis();
      Doc["count"] = count;

      // Digital inputs
      Doc[DATA_FIELD]["event"] = Source;
      Doc[DATA_FIELD]["flag"] = digitalRead(DI4) == HIGH;

      serializeJson(Doc, Out);
      Serial.print(Out);
      Serial.print("\n");
      Changed = false;
      count++;
    }

    prev_time = now;
  }
  delayMicroseconds(delay);
}

