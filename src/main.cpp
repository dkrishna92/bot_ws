// Primary wireless e-stop firmware -- see platformio.ini for board/upload
// settings and project context. This Nano drives a relay that breaks the
// motor driver's power supply directly (not a logic-level enable/disable
// line), and is electrically isolated from the Pi/ROS 2 stack: its Serial
// RX comes from a UART LoRa transceiver module, not the Pi (that link
// would be bot_safety's ROS 2 watchdog, which is defense-in-depth only --
// never the primary mechanism). The transmitting end is feather_ws's
// remote kill-switch firmware, over its own paired LoRa module -- see
// that project's main.cpp for the counterpart and the shared protocol.
//
// Serial protocol, one line per message from the receiver module:
//   H\n   heartbeat / link OK, no e-stop asserted
//   X\n   e-stop explicitly asserted by the transmitter
//
// Fail-safe deadman logic: the relay is de-energized (motor driver
// disabled) whenever either condition holds:
//   - the most recently received message was X, or
//   - no message has arrived within HEARTBEAT_TIMEOUT_MS (covers wireless
//     signal loss, not just an explicit stop command)
// and it stays de-energized until a clean 'H' heartbeat arrives. Before
// the very first message (e.g. right after power-on), the relay defaults
// to de-energized too, so an unconfigured or dead board fails safe.
//
// The relay is wired energized-to-run, switching the motor driver's power
// supply itself (not a logic-level enable pin): its coil is powered from
// this board, so if the Nano itself loses power, the relay also
// de-energizes and cuts power to the driver -- the fail-safe covers Nano
// death, not just a lost wireless signal while the Nano keeps running.
// Size the relay/contacts for the driver's actual supply voltage/current
// (up to 50V per the rules' onboard voltage limit), not a logic-level load.
//
// BLE (previously used here for board identification/debugging only, no
// role in the e-stop signal path) was removed 2026-09-26 -- it was the
// prime suspect for an intermittent boot hang observed during bring-up,
// and it added risk for zero functional benefit on the safety-critical
// board.

#include <Arduino.h>

// Confirmed wiring, 2026-09-26: relay is on the board's "D5" silkscreen
// pin. Using the symbolic macro, not the raw number -- pins_arduino.h
// maps D5 to GPIO8, not GPIO7 (GPIO7 is actually D4), so the previous
// raw "7" placeholder was wired to the wrong physical pin entirely.
#define RELAY_PIN D5
#define RELAY_ENERGIZED HIGH    // flip to LOW if using an active-low relay module
#define RELAY_DEENERGIZED LOW

// Solid on: relay de-energized because of an explicit 'X' e-stop message.
// Blinking: relay de-energized because no heartbeat has arrived within
// HEARTBEAT_TIMEOUT_MS -- lets you tell "kill switch pressed" apart from
// "LoRa link dropped" at a glance, without needing a serial monitor. Off:
// relay energized, motors live.
#define STATUS_LED_PIN LED_BUILTIN
#define LED_BLINK_INTERVAL_MS 250

// On the Nano ESP32, `Serial` is the native USB-CDC console, not a GPIO
// UART -- the LoRa module is wired to real UART pins instead, via this
// dedicated HardwareSerial. Confirmed wiring, 2026-09-26: the module's
// TXD/RXD go to the board's "RX0"/"TX1" silkscreen pins -- using the
// board variant's own D0/D1 macros rather than raw GPIO numbers, since
// pins_arduino.h defines D0=GPIO44 ("also RX") and D1=GPIO43 ("also TX").
#define LORA_RX_PIN D0 // Nano's "RX0" pin, wired to the E32's TXD
#define LORA_TX_PIN D1 // Nano's "TX1" pin, wired to the E32's RXD
HardwareSerial LoraSerial(0);

// Ebyte E32 module's M0/M1 mode-select pins, wired to GPIO (not tied
// directly to ground) -- confirmed wiring, 2026-09-26. Using the Nano
// ESP32 board variant's own D3/D4 silkscreen macros rather than raw GPIO
// numbers, since those macros are guaranteed to match the physical pin
// the wiring refers to. LOW/LOW selects Normal (transparent
// transmit/receive) mode, matching feather_ws's transmitter.
#define LORA_M0_PIN D3
#define LORA_M1_PIN D4

// Well under the rules' 1s cutoff requirement, to leave margin for relay
// switching time and wireless jitter. Receiver module must send 'H' more
// often than this during normal operation -- feather_ws's transmitter
// reports every 100ms, well inside this margin.
#define HEARTBEAT_TIMEOUT_MS 400

unsigned long lastMessageMillis = 0;
bool everReceivedMessage = false;
bool estopActive = true; // mirrors the relay's current commanded state

unsigned long lastBlinkToggleMillis = 0;
bool blinkLedState = false;

void setRelay(bool active) {
  estopActive = active;
  digitalWrite(RELAY_PIN, active ? RELAY_DEENERGIZED : RELAY_ENERGIZED);
}

// Called every loop() iteration, not just on state changes, so the blink
// stays timed correctly (millis()-based, non-blocking -- no delay() here,
// this can't be allowed to stall the deadman-timing checks in loop()).
void updateStatusLed(bool linkLost, unsigned long now) {
  if (linkLost) {
    if (now - lastBlinkToggleMillis >= LED_BLINK_INTERVAL_MS) {
      lastBlinkToggleMillis = now;
      blinkLedState = !blinkLedState;
      digitalWrite(STATUS_LED_PIN, blinkLedState ? HIGH : LOW);
    }
  } else {
    digitalWrite(STATUS_LED_PIN, estopActive ? HIGH : LOW);
  }
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  setRelay(true); // safe default before any message is ever received

  pinMode(LORA_M0_PIN, OUTPUT);
  pinMode(LORA_M1_PIN, OUTPUT);
  digitalWrite(LORA_M0_PIN, LOW);
  digitalWrite(LORA_M1_PIN, LOW);

  // Baud between this Nano and its own LoRa module -- standardized on
  // 9600 (Ebyte E32's factory-default UART rate) to match feather_ws's
  // LORA_BAUD on the transmitting end.
  LoraSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
}

void loop() {
  unsigned long now = millis();

  while (LoraSerial.available()) {
    char c = LoraSerial.read();
    if (c == 'H') {
      lastMessageMillis = now;
      everReceivedMessage = true;
      setRelay(false);
    } else if (c == 'X') {
      lastMessageMillis = now;
      everReceivedMessage = true;
      setRelay(true);
    }
    // anything else (line terminators, garbage/noise) is ignored
  }

  // Covers both "never received a message" (right after power-on) and a
  // heartbeat that's gone stale -- both are wireless-link-lost cases, not
  // an explicit X, so the LED should blink rather than sit solid-on.
  bool linkLost = !everReceivedMessage || (now - lastMessageMillis > HEARTBEAT_TIMEOUT_MS);
  if (linkLost) {
    setRelay(true); // wireless link dropped (or never established) -- fail safe
  }

  updateStatusLed(linkLost, now);
}
