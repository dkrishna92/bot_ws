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

// Confirmed wiring, 2026-09-27. Both encoder channels need
// interrupt-capable pins; the Encoder library handles that portably
// across Teensy models. Mounted on the front-left/front-right wheels
// (one encoder per side, per the skid-steer drivetrain's per-side
// grouping -- see bot_gazebo's DiffDrive plugin config), labeled LF/RF
// accordingly, but electrically these ARE the "left"/"right" side
// encoders wheel_odom_node expects.
#define LEFT_ENC_A_PIN 23  // LF
#define LEFT_ENC_B_PIN 22  // LF
#define RIGHT_ENC_A_PIN 21 // RF
#define RIGHT_ENC_B_PIN 20 // RF

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

  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);
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
}
