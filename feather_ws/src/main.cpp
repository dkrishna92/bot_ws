// Remote wireless kill-switch transmitter -- see platformio.ini for board/
// upload settings and project context. Reads a physical kill-switch input
// pin and continuously reports status to a UART LoRa transceiver module on
// Serial2, which arduino_ws's Nano receives on its own paired module,
// wired directly to its Serial RX (see that project's main.cpp).
//
// Board: YD-RP2040 Feather-form-factor board (identifies over USB as
// Adafruit's own Feather RP2040, see platformio.ini). Confirmed wiring,
// 2026-09-26: GP8/GP9 -- on the arduino-pico core, GP8/GP9 are UART1's
// pins, which is Serial2 (NOT Serial1, which is UART0 on GP0/GP1). This
// board variant doesn't pin out Serial2 by default, so setTX()/setRX()
// in setup() are required. GP8 is TX (MCU output, wired to the E32's
// RXD), GP9 is RX (MCU input, wired to the E32's TXD) -- crossed, not
// straight-through.
//
// Protocol, one line every REPORT_INTERVAL_MS:
//   H\n   kill switch OK (idle)
//   X\n   kill switch asserted (e-stop)
// The Nano treats the most recent line as current status and separately
// relies on a receive timeout to detect wireless signal loss -- so this
// side reports its current status continuously rather than only on
// changes, and there is no distinct "resume" message; simply going back
// to sending H clears the Nano's e-stop.
//
// Switch wiring: normally-closed to ground, INPUT_PULLUP on the pin, so
// idle/OK reads LOW and a switch press, a cut wire, or a dead switch all
// read HIGH -- any wiring fault fails toward e-stop, not away from it.
//
// Debounce is asymmetric, favoring that same fail-safe direction: any
// single HIGH reading trips e-stop immediately (no debounce delay on the
// way INTO e-stop), but clearing back to OK requires RELEASE_DEBOUNCE_MS
// of continuous LOW first -- standard safety-relay practice (instant
// trip, debounced release).
//
// Ebyte E32 LoRa module, transparent-serial (Normal) mode: bytes written
// here are transmitted over the air as-is, no AT-command setup needed.
// M0/M1 are wired to GPIO (not tied directly to ground), so this board
// actively drives both LOW in setup() to select Normal mode.

#include <Arduino.h>

#define KILL_SWITCH_PIN 20 // confirmed wiring, 2026-09-26
#define LORA_SERIAL Serial2 // UART1, GP8 TX / GP9 RX -- see wiring note above
// Must match arduino_ws's LoraSerial baud on the Nano -- both sides
// standardized on 9600 (Ebyte E32's factory-default UART rate).
#define LORA_BAUD 9600

// Confirmed wiring, 2026-09-26. LOW/LOW selects the E32's Normal
// (transparent transmit/receive) mode; other M0/M1 combinations select
// Wake-up/Power-saving/Sleep-configuration modes we don't want here.
#define LORA_M0_PIN 10
#define LORA_M1_PIN 11

// Blinking: heartbeat ("H") being sent -- switch OK, link normal. Solid
// on: e-stop asserted ("X") being sent -- switch pressed/wire cut.
// Confirmed wiring, 2026-09-26: this board's LED_BUILTIN (GPIO13, this
// variant's genuine-Adafruit pin) has no LED on this clone -- the visible
// simple LED is on plain GPIO25 instead (this clone also has a separate
// NeoPixel, unused here).
#define STATUS_LED_PIN 25
#define LED_BLINK_INTERVAL_MS 250

#define REPORT_INTERVAL_MS 100 // well under the Nano's HEARTBEAT_TIMEOUT_MS (400ms)
#define RELEASE_DEBOUNCE_MS 50

bool estopActive = true; // fail-safe default before the first read settles
unsigned long lastOkMillis = 0;
unsigned long lastReportMillis = 0;

unsigned long lastBlinkToggleMillis = 0;
bool blinkLedState = false;

// Non-blocking, millis()-based -- no delay() here, this runs every loop()
// iteration alongside the switch debounce and report timing.
void updateStatusLed(bool active, unsigned long now) {
  if (active) {
    digitalWrite(STATUS_LED_PIN, HIGH);
  } else if (now - lastBlinkToggleMillis >= LED_BLINK_INTERVAL_MS) {
    lastBlinkToggleMillis = now;
    blinkLedState = !blinkLedState;
    digitalWrite(STATUS_LED_PIN, blinkLedState ? HIGH : LOW);
  }
}

void setup() {
  pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);

  pinMode(LORA_M0_PIN, OUTPUT);
  pinMode(LORA_M1_PIN, OUTPUT);
  digitalWrite(LORA_M0_PIN, LOW);
  digitalWrite(LORA_M1_PIN, LOW);

  // Confirmed 2026-09-26: this board's variant file (adafruit_feather/
  // pins_arduino.h) doesn't pin out Serial2 (UART1) by default -- this
  // explicit remap to GP8/GP9 is required, unlike the generic "pico"
  // board where Serial2 defaults there automatically.
  LORA_SERIAL.setTX(8);
  LORA_SERIAL.setRX(9);
  LORA_SERIAL.begin(LORA_BAUD);
}

void loop() {
  bool assertedNow = digitalRead(KILL_SWITCH_PIN); // HIGH = asserted, see wiring note above
  unsigned long now = millis();

  if (assertedNow) {
    estopActive = true;
    lastOkMillis = now; // reset the release-debounce window
  } else if (now - lastOkMillis >= RELEASE_DEBOUNCE_MS) {
    estopActive = false;
  }

  updateStatusLed(estopActive, now);

  if (now - lastReportMillis >= REPORT_INTERVAL_MS) {
    lastReportMillis = now;
    LORA_SERIAL.println(estopActive ? "X" : "H");
  }
}
