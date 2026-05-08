// Improved firmware for Chinese "14 Bit" Sim Racing Handbrake
// Based on original by Daniel Korgel
//
// Adds:
//   - Peak-hold detection to handle non-monotonic hall sensor curve
//   - EEPROM-backed calibration that persists across reboots
//   - Serial command interface for tweaking from a PC terminal
//
// Serial commands (9600 baud, newline-terminated):
//   GET           - print current calibration
//   RAW           - print one raw sensor reading
//   STREAM ON/OFF - continuously stream raw + mapped values
//   MIN <n>       - set min (rest) value
//   MAX <n>       - set max (peak) value
//   RELEASE <n>   - set release threshold for peak-hold reset
//   AUTOCAL       - capture next 5s of pulls to learn min/peak
//   SAVE          - write current values to EEPROM
//   RESET         - restore defaults (does not save until SAVE)
//
// Arduino Joystick Library by Matthew Heironimus:
// https://github.com/MHeironimus/ArduinoJoystickLibrary
//--------------------------------------------------------------------

//#define DEBUG

#define AXIS_RESOLUTION   4096
#define PIN_HALL_SENSOR   A2
#define PIN_BUTTON_0      2
#define PIN_BUTTON_1      3

#define EEPROM_MAGIC      0xA7
#define EEPROM_ADDR       0

#include <Joystick.h>
#include <EEPROM.h>

struct Calibration {
  uint8_t magic;
  int minValue;
  int maxValue;
  int releaseThreshold;  // how close to minValue before peak resets
};

// Defaults — overridden by EEPROM if a valid blob is present
Calibration cal = { EEPROM_MAGIC, 507, 620, 20 };

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
  2,                      // Button Count
  0,                      // Hat Switch Count
  false, false, true,     // Z axis only
  false, false, false,
  false, false,
  false, false, false);

// --- EEPROM helpers -------------------------------------------------

void loadCal() {
  Calibration tmp;
  EEPROM.get(EEPROM_ADDR, tmp);
  if (tmp.magic == EEPROM_MAGIC) {
    cal = tmp;
  }
}

void saveCal() {
  cal.magic = EEPROM_MAGIC;
  EEPROM.put(EEPROM_ADDR, cal);
}

// --- Serial command handling ---------------------------------------

bool streaming = false;

void printCal() {
  Serial.print(F("MIN=")); Serial.print(cal.minValue);
  Serial.print(F(" MAX=")); Serial.print(cal.maxValue);
  Serial.print(F(" RELEASE=")); Serial.println(cal.releaseThreshold);
}

void runAutocal() {
  Serial.println(F("AUTOCAL: pull handbrake fully a few times over next 5s..."));
  int learnedMin = 1023;
  int learnedMax = 0;
  unsigned long start = millis();
  while (millis() - start < 5000) {
    int v = analogRead(PIN_HALL_SENSOR);
    if (v < learnedMin) learnedMin = v;
    if (v > learnedMax) learnedMax = v;
    delay(2);
  }
  // Back the max off slightly so we stay on the rising side of the curve
  cal.minValue = learnedMin;
  cal.maxValue = learnedMax - 5;
  Serial.print(F("AUTOCAL done. "));
  printCal();
  Serial.println(F("Type SAVE to persist."));
}

void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "GET") {
    printCal();
  } else if (cmd == "RAW") {
    Serial.println(analogRead(PIN_HALL_SENSOR));
  } else if (cmd == "STREAM ON") {
    streaming = true;
    Serial.println(F("OK"));
  } else if (cmd == "STREAM OFF") {
    streaming = false;
    Serial.println(F("OK"));
  } else if (cmd.startsWith("MIN ")) {
    cal.minValue = cmd.substring(4).toInt();
    Serial.println(F("OK"));
  } else if (cmd.startsWith("MAX ")) {
    cal.maxValue = cmd.substring(4).toInt();
    Serial.println(F("OK"));
  } else if (cmd.startsWith("RELEASE ")) {
    cal.releaseThreshold = cmd.substring(8).toInt();
    Serial.println(F("OK"));
  } else if (cmd == "AUTOCAL") {
    runAutocal();
  } else if (cmd == "SAVE") {
    saveCal();
    Serial.println(F("SAVED"));
  } else if (cmd == "RESET") {
    cal = { EEPROM_MAGIC, 507, 620, 20 };
    Serial.println(F("RESET (not saved)"));
  } else if (cmd.length() > 0) {
    Serial.print(F("UNKNOWN: "));
    Serial.println(cmd);
  }
}

// --- Setup / Loop --------------------------------------------------

void setup() {
  Serial.begin(9600);

  pinMode(PIN_HALL_SENSOR, INPUT);
  pinMode(PIN_BUTTON_0, INPUT_PULLUP);
  pinMode(PIN_BUTTON_1, INPUT_PULLUP);

  loadCal();

  Joystick.begin();
  Joystick.setZAxisRange(0, AXIS_RESOLUTION);
}

void loop() {
  handleSerial();

  // Buttons pass straight through
  Joystick.setButton(0, digitalRead(PIN_BUTTON_0) == LOW);
  Joystick.setButton(1, digitalRead(PIN_BUTTON_1) == LOW);

  int hallSensorValue = analogRead(PIN_HALL_SENSOR);

  // Peak-hold: track the highest value seen during this pull.
  // Reset when the sensor returns near the rest position.
  static int peakThisPull = cal.minValue;

  if (hallSensorValue < cal.minValue + cal.releaseThreshold) {
    // Released: reset and follow the live value
    peakThisPull = hallSensorValue;
  } else if (hallSensorValue > peakThisPull) {
    // Still rising: follow it up
    peakThisPull = hallSensorValue;
  }
  // else: sensor has dropped past its peak — hold at peakThisPull

  int effective = constrain(peakThisPull, cal.minValue, cal.maxValue);
  float zAxisValue =
      (effective - cal.minValue) / (float)(cal.maxValue - cal.minValue) * AXIS_RESOLUTION;

  Joystick.setZAxis(zAxisValue);

  if (streaming) {
    Serial.print(F("raw=")); Serial.print(hallSensorValue);
    Serial.print(F(" peak=")); Serial.print(peakThisPull);
    Serial.print(F(" z=")); Serial.println((int)zAxisValue);
  }

#if DEBUG
  Serial.print(hallSensorValue);
  Serial.print(F(" -> "));
  Serial.println(zAxisValue);
#endif

  delay(10);
}
