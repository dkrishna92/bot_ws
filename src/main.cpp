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

// Placeholder pins -- confirm against the actual Teensy board/wiring once
// hardware is in hand. Both encoder channels need interrupt-capable pins;
// the Encoder library handles that portably across Teensy models.
#define LEFT_ENC_A_PIN 2
#define LEFT_ENC_B_PIN 3
#define RIGHT_ENC_A_PIN 4
#define RIGHT_ENC_B_PIN 5

// Ultrasonic trigger/echo pins -- placeholders, same TBD status as the
// encoder pins above and as bot_ultrasonic's old RPi.GPIO pin numbers
// (which no longer apply now that these sensors are wired to the Teensy
// instead of the Pi's GPIO header).
#define FRONT_LEFT_TRIG_PIN 6
#define FRONT_LEFT_ECHO_PIN 7
#define FRONT_RIGHT_TRIG_PIN 8
#define FRONT_RIGHT_ECHO_PIN 9
#define REAR_TRIG_PIN 10
#define REAR_ECHO_PIN 11

#define ENCODER_REPORT_INTERVAL_MS 20   // ~50Hz, matches bot_odometry's expected sample rate
#define ULTRASONIC_REPORT_INTERVAL_MS 50 // ~20Hz, matches HC-SR04's practical ping rate

// 4m max range (matches the old bot_ultrasonic default max_range_m), plus
// margin, converted to a pulseIn() timeout in microseconds:
// 2 * range / speed_of_sound.
#define ULTRASONIC_TIMEOUT_US 30000UL

Encoder leftEncoder(LEFT_ENC_A_PIN, LEFT_ENC_B_PIN);
Encoder rightEncoder(RIGHT_ENC_A_PIN, RIGHT_ENC_B_PIN);

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

  pinMode(FRONT_LEFT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_LEFT_ECHO_PIN, INPUT);
  pinMode(FRONT_RIGHT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_RIGHT_ECHO_PIN, INPUT);
  pinMode(REAR_TRIG_PIN, OUTPUT);
  pinMode(REAR_ECHO_PIN, INPUT);
}

void loop() {
  if (sinceLastEncoderReport >= ENCODER_REPORT_INTERVAL_MS) {
    sinceLastEncoderReport = 0;
    long leftTicks = leftEncoder.read();
    long rightTicks = rightEncoder.read();
    Serial.print("E,");
    Serial.print(leftTicks);
    Serial.print(",");
    Serial.print(rightTicks);
    Serial.print(",");
    Serial.println(micros());
  }

  // Sequential blocking reads (pulseIn) -- up to ~3*ULTRASONIC_TIMEOUT_US
  // worst case if every sensor times out simultaneously. That stalls
  // encoder reporting (not encoder counting itself, which is
  // interrupt-driven and keeps counting regardless of loop() timing) for
  // the duration -- acceptable jitter given this is still a bit-banged
  // sensor, same tradeoff CLAUDE.md already accepted on the Pi.
  if (sinceLastUltrasonicReport >= ULTRASONIC_REPORT_INTERVAL_MS) {
    sinceLastUltrasonicReport = 0;
    unsigned long frontLeftUs = readUltrasonicPulseUs(FRONT_LEFT_TRIG_PIN, FRONT_LEFT_ECHO_PIN);
    unsigned long frontRightUs = readUltrasonicPulseUs(FRONT_RIGHT_TRIG_PIN, FRONT_RIGHT_ECHO_PIN);
    unsigned long rearUs = readUltrasonicPulseUs(REAR_TRIG_PIN, REAR_ECHO_PIN);
    Serial.print("U,");
    Serial.print(frontLeftUs);
    Serial.print(",");
    Serial.print(frontRightUs);
    Serial.print(",");
    Serial.print(rearUs);
    Serial.print(",");
    Serial.println(micros());
  }
}
