// Wheel encoder + ultrasonic reporting firmware -- see platformio.ini for
// the serial protocol and project context. E-stop is a separate Arduino
// Nano wired directly to the motor driver's power supply via a relay;
// none of that logic lives on this board (see arduino_ws).
//
// Ultrasonic sensing moved here from the Pi (2026-09-20) so the Pi-side
// bot_ultrasonic GPIO node (bit-banged RPi.GPIO trigger/echo timing) could
// be retired -- CLAUDE.md's Hardware section already flagged that as a
// timing-noise risk. bot_odometry's wheel_odom_node is the sole owner of
// this board's one USB serial link (Teensy exposes a single virtual COM
// port here, not a dual-serial config), so it now parses both the E and U
// lines below rather than having a second ROS node open the same port.

#include <Encoder.h>

// TEMPORARY WORKAROUND (2026-09-27): the encoder A/B voltage dividers bring
// the 5 V encoder outputs down to only ~1.2 V (left) / ~1.37 V (right) at
// the Teensy pins -- below the Teensy 4.1's ~2.3 V digital-high threshold,
// so digital reads (and the Encoder library) never see a single edge.
// Until the dividers are re-sized for ~3.0-3.3 V, decode the encoders from
// analog readings instead: a timer interrupt samples all four pins, applies
// a hysteresis threshold, and runs a quadrature state table. Set to 0 once
// the wiring is fixed to go back to the interrupt-driven Encoder library.
#define ENCODER_ANALOG_WORKAROUND 1

// Confirmed wiring, 2026-09-27. Both encoder channels need
// interrupt-capable pins; the Encoder library handles that portably
// across Teensy models. Mounted on the front-left/front-right wheels
// (one encoder per side, per the skid-steer drivetrain's per-side
// grouping -- see bot_gazebo's DiffDrive plugin config), labeled LF/RF
// accordingly, but electrically these ARE the "left"/"right" side
// encoders wheel_odom_node expects.
// Corrected 2026-09-27 against the actual wiring (checked with a meter):
// left is on 20/21 and right on 22/23, the reverse of the earlier
// assignment. Swapping A/B on a side reverses its count direction.
#define LEFT_ENC_A_PIN 20  // LF
#define LEFT_ENC_B_PIN 21  // LF
#define RIGHT_ENC_A_PIN 22 // RF
#define RIGHT_ENC_B_PIN 23 // RF

// Count direction per side: +1 or -1 so that each wheel rolling FORWARD
// counts UP (what wheel_odom_node assumes). Applied here, not in Python,
// so scripts/motor_test.py's raw E-line reading agrees with odometry.
// History: on 2026-09-28 a RIGHT_ENCODER_SIGN of -1 was added -- but to a
// stale standalone copy of this firmware (old pin map, digital Encoder
// library on ~1.2 V signals) that was then flashed by mistake and ran
// until 2026-09-29, so that sign was never valid for THIS code. Re-checked
// with scripts/motor_test.py on this firmware 2026-09-29: left counts up
// on "left forward", right counted DOWN on "right forward" -> -1. That's
// expected: the motors are mounted mirror-image, so a forward-rolling right
// wheel spins its motor shaft (and encoder) the opposite way to the left.
// (Right motor direction itself was visually confirmed 2026-09-28.)
#define LEFT_ENCODER_SIGN 1
#define RIGHT_ENCODER_SIGN -1

// Ultrasonic trigger/echo pins -- confirmed wiring, 2026-09-27. Design
// changed from three sensors (front-left/front-right/rear) to two
// (left/right) -- the rear sensor was dropped, not just unwired.
#define LEFT_TRIG_PIN 2
#define LEFT_ECHO_PIN 3
#define RIGHT_TRIG_PIN 4
#define RIGHT_ECHO_PIN 5

#define ENCODER_REPORT_INTERVAL_MS 20   // ~50Hz, matches bot_odometry's expected sample rate
#define ULTRASONIC_REPORT_INTERVAL_MS 50 // ~20Hz, matches HC-SR04's practical ping rate

// 4m max range (matches the old bot_ultrasonic default max_range_m), plus
// margin, converted to a pulseIn() timeout in microseconds:
// 2 * range / speed_of_sound.
#define ULTRASONIC_TIMEOUT_US 30000UL

#if ENCODER_ANALOG_WORKAROUND
// Analog quadrature decoding -- see ENCODER_ANALOG_WORKAROUND above.
//
// Sample rate: at the motor's 500 RPM no-load output speed each channel
// changes ~4000 times/s (979.62 counts/rev x 8.3 rev/s, split over 2
// channels), so 40 kHz gives ~5-10 samples per state. Four 8-bit reads
// with averaging off must fit well inside the 25 us period (the S status
// line reports the worst case).
#define ENC_SAMPLE_PERIOD_US 25
// Hysteresis thresholds in 8-bit ADC counts (3.3 V full scale): a channel
// goes high above ~0.8 V and back low below ~0.4 V. Measured highs are
// ~1.2-1.37 V and lows ~0.00-0.02 V, so both have a wide margin. 8 bits
// (~13 mV/step) is plenty for this and converts faster than 10/12.
#define ENC_HIGH_THRESHOLD 62  // 0.8 V
#define ENC_LOW_THRESHOLD 31   // 0.4 V

struct AnalogQuadrature {
  uint8_t pinA, pinB;
  uint8_t state;            // (A << 1) | B, after hysteresis
  volatile int32_t count;
  volatile uint32_t invalid;  // both channels changed between samples (missed state)
};
AnalogQuadrature leftEnc = {LEFT_ENC_A_PIN, LEFT_ENC_B_PIN, 0, 0, 0};
AnalogQuadrature rightEnc = {RIGHT_ENC_A_PIN, RIGHT_ENC_B_PIN, 0, 0, 0};

// Standard quadrature transition table, indexed by (old_state << 2) | new_state:
// +1/-1 for a valid single-channel step, 0 for no change or an invalid
// double step (counted separately in `invalid`).
const int8_t QUAD_TABLE[16] = {0, -1, +1, 0, +1, 0, 0, -1, -1, 0, 0, +1, 0, +1, -1, 0};

IntervalTimer encoderTimer;
volatile uint32_t encoderIsrMaxUs = 0;
// Lowest/highest raw 8-bit reading per pin since the last S line, indexed
// left A, left B, right A, right B -- shows whether the signals still
// swing across ENC_LOW/HIGH_THRESHOLD (a pin stuck mid-band never counts).
volatile uint8_t adcMin[4] = {255, 255, 255, 255};
volatile uint8_t adcMax[4] = {0, 0, 0, 0};

static inline uint8_t hysteresisBit(uint8_t pin, uint8_t idx, uint8_t previous) {
  int v = analogRead(pin);
  if (v < adcMin[idx]) adcMin[idx] = v;
  if (v > adcMax[idx]) adcMax[idx] = v;
  if (v > ENC_HIGH_THRESHOLD) return 1;
  if (v < ENC_LOW_THRESHOLD) return 0;
  return previous;
}

static inline void updateQuadrature(AnalogQuadrature &enc, uint8_t idx) {
  uint8_t a = hysteresisBit(enc.pinA, idx, (enc.state >> 1) & 1);
  uint8_t b = hysteresisBit(enc.pinB, idx + 1, enc.state & 1);
  uint8_t next = (a << 1) | b;
  if (next == enc.state) return;
  if ((next ^ enc.state) == 0b11) {
    enc.invalid++;
  } else {
    enc.count += QUAD_TABLE[(enc.state << 2) | next];
  }
  enc.state = next;
}

void encoderSampleIsr() {
  uint32_t start = micros();
  updateQuadrature(leftEnc, 0);
  updateQuadrature(rightEnc, 2);
  uint32_t took = micros() - start;
  if (took > encoderIsrMaxUs) encoderIsrMaxUs = took;
}

long readLeftTicks() { noInterrupts(); long c = leftEnc.count; interrupts(); return c; }
long readRightTicks() { noInterrupts(); long c = rightEnc.count; interrupts(); return c; }

void setupEncoders() {
  analogReadResolution(8);
  analogReadAveraging(1);  // default averaging makes each read several times slower
  for (uint8_t pin : {LEFT_ENC_A_PIN, LEFT_ENC_B_PIN, RIGHT_ENC_A_PIN, RIGHT_ENC_B_PIN}) {
    pinMode(pin, INPUT_DISABLE);  // analog only; no digital buffer or pull-up
  }
  // Start from the real channel levels, not an assumed 00, so boot doesn't
  // register a phantom count. Pins that sit mid-band read as low.
  leftEnc.state = (hysteresisBit(leftEnc.pinA, 0, 0) << 1) | hysteresisBit(leftEnc.pinB, 1, 0);
  rightEnc.state = (hysteresisBit(rightEnc.pinA, 2, 0) << 1) | hysteresisBit(rightEnc.pinB, 3, 0);
  encoderTimer.begin(encoderSampleIsr, ENC_SAMPLE_PERIOD_US);
}
#else
Encoder leftEncoder(LEFT_ENC_A_PIN, LEFT_ENC_B_PIN);
Encoder rightEncoder(RIGHT_ENC_A_PIN, RIGHT_ENC_B_PIN);
long readLeftTicks() { return leftEncoder.read(); }
long readRightTicks() { return rightEncoder.read(); }
void setupEncoders() {}
#endif

elapsedMillis sinceLastEncoderReport;
elapsedMillis sinceLastUltrasonicReport;

// Returns the raw echo pulse width in microseconds, or 0 on timeout (no
// echo / out of range). Distance conversion happens on the Pi side
// (wheel_odom_node.py), same "MCU reports raw samples, Pi applies
// application-level math" split already used for encoder ticks.
unsigned long readUltrasonicPulseUs(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH, ULTRASONIC_TIMEOUT_US);
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);

  setupEncoders();
}

void loop() {
  if (sinceLastEncoderReport >= ENCODER_REPORT_INTERVAL_MS) {
    sinceLastEncoderReport = 0;
    long leftTicks = LEFT_ENCODER_SIGN * readLeftTicks();
    long rightTicks = RIGHT_ENCODER_SIGN * readRightTicks();
    Serial.print("E,");
    Serial.print(leftTicks);
    Serial.print(",");
    Serial.print(rightTicks);
    Serial.print(",");
    Serial.println(micros());
  }

  // Sequential blocking reads (pulseIn) -- up to ~2*ULTRASONIC_TIMEOUT_US
  // worst case if both sensors time out simultaneously. That stalls
  // encoder reporting (not encoder counting itself, which is
  // interrupt-driven and keeps counting regardless of loop() timing) for
  // the duration -- acceptable jitter given this is still a bit-banged
  // sensor, same tradeoff CLAUDE.md already accepted on the Pi.
  if (sinceLastUltrasonicReport >= ULTRASONIC_REPORT_INTERVAL_MS) {
    sinceLastUltrasonicReport = 0;
    unsigned long leftUs = readUltrasonicPulseUs(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
    unsigned long rightUs = readUltrasonicPulseUs(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
    Serial.print("U,");
    Serial.print(leftUs);
    Serial.print(",");
    Serial.print(rightUs);
    Serial.print(",");
    Serial.println(micros());
  }

#if ENCODER_ANALOG_WORKAROUND
  // Once a second: S,<isr_max_us>,<left_invalid>,<right_invalid>, then
  // min/max raw 8-bit ADC per pin over that second (LA,LB,RA,RB; 1 count
  // ~13 mV). Debug only -- wheel_odom_node ignores lines that aren't E/U.
  // Nonzero invalid counts mean samples are too slow for the wheel speed.
  static elapsedMillis sinceLastStatus;
  if (sinceLastStatus >= 1000) {
    sinceLastStatus = 0;
    uint8_t mn[4], mx[4];
    noInterrupts();
    for (int i = 0; i < 4; i++) { mn[i] = adcMin[i]; mx[i] = adcMax[i]; adcMin[i] = 255; adcMax[i] = 0; }
    interrupts();
    Serial.printf("S,%lu,%lu,%lu,%u-%u,%u-%u,%u-%u,%u-%u\n", encoderIsrMaxUs, leftEnc.invalid, rightEnc.invalid,
                  mn[0], mx[0], mn[1], mx[1], mn[2], mx[2], mn[3], mx[3]);
  }
#endif
}
