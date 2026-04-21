/*
 * ============================================================
 * RAKSHAQ — Nucleo-F030R8 Stepper Motor Controller
 * ============================================================
 *
 * Board  : STM32 Nucleo-F030R8 (Arduino IDE + STM32duino core)
 * Role   : Pan/Tilt stepper motor driver for the RAKSHAQ
 *          autonomous AI turret system.
 *
 * Communication (Raspberry Pi  →  Nucleo):
 *   Interface : USB-CDC (Virtual COM Port) — /dev/ttyACM0 on Pi
 *   Baud Rate : 115200
 *   Protocol  : ASCII lines, newline-terminated
 *
 * Command set (matches stm32.py STM32Controller.send()):
 *   X<steps>\n   — Pan  axis  (+ = right, – = left)
 *   Y<steps>\n   — Tilt axis  (+ = up,    – = down)
 *   X0\n         — Pan  heartbeat / stop
 *   Y0\n         — Tilt heartbeat / stop
 *
 * Motor math (matches STEPS_PER_DEGREE = 400/45 ≈ 8.888 in stm32.py):
 *   Driver microstepping : 1/16
 *   Steps per revolution : 200 full × 16 micro = 3200 steps/rev
 *   400 steps = 45°  →  8.888 steps per degree
 *
 * Motor driver wiring (A4988 / DRV8825 — same pinout):
 *
 *   PAN  stepper driver:
 *     STEP  → D3   (PA_4  — Timer-capable)
 *     DIR   → D4   (PB_5)
 *     EN    → D5   (PB_4, active LOW)
 *
 *   TILT stepper driver:
 *     STEP  → D6   (PB_10)
 *     DIR   → D7   (PA_8)
 *     EN    → D8   (PA_9, active LOW)
 *
 *   MS1/MS2/MS3 on both drivers → tie HIGH/HIGH/HIGH for 1/16 step
 *   SLEEP / RESET → tie HIGH (or wire together and pull HIGH)
 *   VMOT → 12V rail;  VDD → 3.3V;  GND shared with Nucleo GND
 *
 * LED feedback:
 *   LD2 (PA_5 / D13)  — blinks on every valid command received
 *
 * ============================================================
 */

// ──────────────────────────────────────────────────────────────
//  Pin definitions
// ──────────────────────────────────────────────────────────────

// PAN axis (X commands)
#define PAN_STEP_PIN   3     // D3
#define PAN_DIR_PIN    4     // D4
#define PAN_EN_PIN     5     // D5

// TILT axis (Y commands)
#define TILT_STEP_PIN  6     // D6
#define TILT_DIR_PIN   7     // D7
#define TILT_EN_PIN    8     // D8

// Status LED (on-board LD2)
#define LED_PIN        LED_BUILTIN    // PA5 on Nucleo-F030R8

// ──────────────────────────────────────────────────────────────
//  Stepper timing
// ──────────────────────────────────────────────────────────────

/*
 * Pulse width for the STEP pin (microseconds).
 * A4988 minimum is 1 µs, DRV8825 minimum is 1.9 µs.
 * 5 µs gives comfortable margin.
 */
#define STEP_PULSE_US   5

/*
 * Base delay between steps (microseconds).
 * Lower = faster.  Too low risks missed steps.
 * 800 µs  → ~1250 steps/sec → ~140°/sec at 8.89 steps/deg
 * Adjust if your motors skip steps or stall.
 */
#define STEP_DELAY_US   800

// ──────────────────────────────────────────────────────────────
//  Serial configuration
// ──────────────────────────────────────────────────────────────
#define SERIAL_BAUD     115200
#define CMD_BUF_SIZE    32      // max command string length

// ──────────────────────────────────────────────────────────────
//  State
// ──────────────────────────────────────────────────────────────
char    cmd_buf[CMD_BUF_SIZE];
uint8_t cmd_idx = 0;

// ──────────────────────────────────────────────────────────────
//  Helper: step a driver N steps
//
//  stepPin  : STEP signal pin
//  dirPin   : DIR signal pin
//  steps    : number of steps (negative = reverse direction)
// ──────────────────────────────────────────────────────────────
void stepMotor(int stepPin, int dirPin, int steps) {
  if (steps == 0) return;

  // Set direction
  digitalWrite(dirPin, steps > 0 ? HIGH : LOW);
  delayMicroseconds(1);           // DIR setup time (≥200 ns for A4988)

  int absSteps = abs(steps);
  for (int i = 0; i < absSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

// ──────────────────────────────────────────────────────────────
//  Helper: blink LED briefly to signal a received command
// ──────────────────────────────────────────────────────────────
void blinkLED() {
  digitalWrite(LED_PIN, HIGH);
  delay(30);
  digitalWrite(LED_PIN, LOW);
}

// ──────────────────────────────────────────────────────────────
//  Parse and execute a complete command string.
//
//  Expected formats:  "X<int>"  or  "Y<int>"
//  e.g.  "X200"  "X-50"  "Y0"  "Y-100"
// ──────────────────────────────────────────────────────────────
void executeCommand(const char* cmd) {
  char axis = cmd[0];

  // Must start with X or Y
  if (axis != 'X' && axis != 'Y') return;

  // Parse the integer that follows
  int steps = atoi(cmd + 1);

  blinkLED();

  if (axis == 'X') {
    // Pan axis
    stepMotor(PAN_STEP_PIN, PAN_DIR_PIN, steps);
  } else {
    // Tilt axis
    stepMotor(TILT_STEP_PIN, TILT_DIR_PIN, steps);
  }

  // Acknowledge back to Pi (optional — Pi doesn't currently read replies,
  // but useful for debugging with a serial monitor)
  Serial.print("OK ");
  Serial.println(cmd);
}

// ──────────────────────────────────────────────────────────────
//  setup()
// ──────────────────────────────────────────────────────────────
void setup() {
  // USB-CDC serial (shows up as /dev/ttyACM0 on the Pi)
  Serial.begin(SERIAL_BAUD);

  // PAN driver pins
  pinMode(PAN_STEP_PIN, OUTPUT);
  pinMode(PAN_DIR_PIN,  OUTPUT);
  pinMode(PAN_EN_PIN,   OUTPUT);
  digitalWrite(PAN_EN_PIN, LOW);     // Enable driver (active LOW)

  // TILT driver pins
  pinMode(TILT_STEP_PIN, OUTPUT);
  pinMode(TILT_DIR_PIN,  OUTPUT);
  pinMode(TILT_EN_PIN,   OUTPUT);
  digitalWrite(TILT_EN_PIN, LOW);    // Enable driver (active LOW)

  // Status LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Startup blink sequence — 3 quick flashes to confirm power-on
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW);  delay(100);
  }

  Serial.println("RAKSHAQ Nucleo ready. Awaiting X/Y commands at 115200.");
}

// ──────────────────────────────────────────────────────────────
//  loop()
//
//  Read bytes from USB-CDC serial, buffer them until '\n', then
//  forward the complete line to executeCommand().
// ──────────────────────────────────────────────────────────────
void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    // Ignore carriage returns (handles \r\n from Python's serial lib)
    if (c == '\r') continue;

    if (c == '\n') {
      // Null-terminate and execute
      cmd_buf[cmd_idx] = '\0';
      if (cmd_idx > 0) {
        executeCommand(cmd_buf);
      }
      cmd_idx = 0;   // reset buffer for next command
    } else {
      // Append character — guard against overflow
      if (cmd_idx < CMD_BUF_SIZE - 1) {
        cmd_buf[cmd_idx++] = c;
      } else {
        // Buffer overflow: discard and reset
        cmd_idx = 0;
      }
    }
  }
}
