/*
 * ============================================================
 * RAKSHAQ — Nucleo-F030R8 Stepper Motor Controller
 * ============================================================
 *
 * Board  : STM32 Nucleo-F030R8 (Arduino IDE + STM32duino core)
 * Shield : Standard Arduino CNC Shield v3.0
 * Role   : Pan/Tilt stepper motor driver for the RAKSHAQ
 *          autonomous AI turret system.
 *
 * Communication (Raspberry Pi → Nucleo):
 *   Interface : USB-CDC (Virtual COM Port) — /dev/ttyACM0 on Pi
 *   Baud Rate : 115200
 *   Protocol  : ASCII lines, newline-terminated
 *
 * Command set (matches stm32.py STM32Controller.send()):
 *   X<steps>\n   — Pan  axis  (+ = right, – = left)
 *   Y<steps>\n   — Tilt axis  (+ = up,    – = down)
 *   X0\n / Y0\n  — Heartbeat / stop (no motion)
 *
 * CNC Shield v3 pin map (these are FIXED by the shield PCB):
 * ┌──────────┬──────┬─────┬────────────────────────┐
 * │ Axis     │ STEP │ DIR │ Notes                  │
 * ├──────────┼──────┼─────┼────────────────────────┤
 * │ X (Pan)  │  D2  │  D5 │ Left/Right motor       │
 * │ Y (Tilt) │  D3  │  D6 │ Up/Down motor          │
 * │ Z        │  D4  │  D7 │ Unused                 │
 * ├──────────┼──────┼─────┼────────────────────────┤
 * │ ENABLE   │  D8  │  —  │ Shared, active LOW     │
 * │ MS1/2/3  │ D10/11/12  │ Microstepping (see note│
 * └──────────┴──────┴─────┴────────────────────────┘
 *
 * Microstepping note:
 *   MS pins are shared across all drivers on the CNC shield.
 *   With 1/16 step (user confirmed): MS1=HIGH, MS2=HIGH, MS3=HIGH
 *   These are hardwired on the shield by jumpers — no code needed.
 *
 * Motor math (matches STEPS_PER_DEGREE = 400/45 ≈ 8.888 in stm32.py):
 *   Full steps/rev : 200
 *   Microstepping  : 1/16
 *   Steps/rev      : 200 × 16 = 3200
 *   Steps/degree   : 3200 / 360 ≈ 8.888
 *   400 steps      = 45°
 *
 * ============================================================
 */

// ──────────────────────────────────────────────────────────────
//  CNC Shield v3 Pin Definitions  (DO NOT CHANGE — PCB-wired)
// ──────────────────────────────────────────────────────────────

#define X_STEP_PIN   2      // Pan axis STEP  (D2)
#define X_DIR_PIN    5      // Pan axis DIR   (D5)

#define Y_STEP_PIN   3      // Tilt axis STEP (D3)
#define Y_DIR_PIN    6      // Tilt axis DIR  (D6)

#define ENABLE_PIN   8      // Shared driver ENABLE — active LOW (D8)

// Status LED (on-board LD2 — PA5 on Nucleo-F030R8)
#define LED_PIN      LED_BUILTIN

// ──────────────────────────────────────────────────────────────
//  Stepper timing
// ──────────────────────────────────────────────────────────────

/*
 * STEP pulse width (µs).
 * A4988 min = 1 µs, DRV8825 min = 1.9 µs. 5 µs is safe for both.
 */
#define STEP_PULSE_US   5

/*
 * Delay between steps (µs) — controls motor speed.
 * 800 µs → ~1250 steps/sec → ~140°/sec at 8.89 steps/deg.
 * Lower = faster. Too low = missed steps / stall.
 */
#define STEP_DELAY_US   800

// ──────────────────────────────────────────────────────────────
//  Serial
// ──────────────────────────────────────────────────────────────
#define SERIAL_BAUD   115200
#define CMD_BUF_SIZE  32

// ──────────────────────────────────────────────────────────────
//  State
// ──────────────────────────────────────────────────────────────
char    cmd_buf[CMD_BUF_SIZE];
uint8_t cmd_idx = 0;

// ──────────────────────────────────────────────────────────────
//  Helper: drive a stepper N steps
// ──────────────────────────────────────────────────────────────
void stepMotor(uint8_t stepPin, uint8_t dirPin, int steps) {
  if (steps == 0) return;

  digitalWrite(dirPin, steps > 0 ? HIGH : LOW);
  delayMicroseconds(2);     // DIR setup time (≥200 ns for A4988/DRV8825)

  int n = abs(steps);
  for (int i = 0; i < n; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

// ──────────────────────────────────────────────────────────────
//  Helper: brief LED blink → visual confirmation of each command
// ──────────────────────────────────────────────────────────────
void blinkLED() {
  digitalWrite(LED_PIN, HIGH);
  delay(25);
  digitalWrite(LED_PIN, LOW);
}

// ──────────────────────────────────────────────────────────────
//  Parse and execute one complete command string.
//  Format: "X<int>" or "Y<int>"   e.g. "X200"  "X-50"  "Y0"
// ──────────────────────────────────────────────────────────────
void executeCommand(const char* cmd) {
  char axis = cmd[0];
  if (axis != 'X' && axis != 'Y') return;

  int steps = atoi(cmd + 1);   // handles negative numbers correctly

  blinkLED();

  if (axis == 'X') {
    stepMotor(X_STEP_PIN, X_DIR_PIN, steps);   // Pan
  } else {
    stepMotor(Y_STEP_PIN, Y_DIR_PIN, steps);   // Tilt
  }

  // Echo back to Pi (useful for debugging, harmless in production)
  Serial.print("OK ");
  Serial.println(cmd);
}

// ──────────────────────────────────────────────────────────────
//  setup()
// ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(SERIAL_BAUD);

  // CNC Shield pins
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN,  OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN,  OUTPUT);

  // Single shared ENABLE — pull LOW to energise all drivers
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  // Status LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // 3-blink startup sequence
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(120);
    digitalWrite(LED_PIN, LOW);  delay(120);
  }

  Serial.println("RAKSHAQ Nucleo ready. X=pan  Y=tilt  @115200.");
}

// ──────────────────────────────────────────────────────────────
//  loop()
//  Accumulate bytes into a buffer; execute on newline.
// ──────────────────────────────────────────────────────────────
void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;    // ignore CR in \r\n pairs from pyserial

    if (c == '\n') {
      cmd_buf[cmd_idx] = '\0';
      if (cmd_idx > 0) {
        executeCommand(cmd_buf);
      }
      cmd_idx = 0;
    } else {
      if (cmd_idx < CMD_BUF_SIZE - 1) {
        cmd_buf[cmd_idx++] = c;
      } else {
        cmd_idx = 0;   // overflow — discard and reset
      }
    }
  }
}
