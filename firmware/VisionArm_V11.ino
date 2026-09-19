/*
  ============================================================================
  VisionArm — Kinematix Team, RMK Innovate 2026
  Firmware version: V11
  ============================================================================

  WHAT THIS DOES
  ---------------
  Runs the 5-DOF (+gripper) robotic arm's servo joints directly from ESP32
  GPIO pins (no PCA9685 — driven straight off the chip using the ESP32Servo
  library), controlled manually via an Xbox controller paired over
  Bluetooth using Bluepad32.

  This is the PRIMARY control mode for the hackathon demo (manual mode).
  Camera-guided autonomous mode (laptop OpenCV + ikpy -> WebSocket) is a
  separate secondary system and is NOT part of this sketch.

  HARDWARE / PIN MAP
  -------------------
    GPIO14  -> Shoulder RIGHT   (MG996R, positional - converted from CR)
    GPIO27  -> Shoulder LEFT    (MG996R, positional - converted from CR,
                                 mirrored: left_angle = 180 - right_angle,
                                 because the two shoulder servos face each
                                 other on a shared axis)
    GPIO13  -> Upper arm        (MG996R, positional - converted from CR)
                                 rest = 160 deg. NEVER command above 160 -
                                 mechanical limit on this joint.
    GPIO33  -> Base rotation    (MG996R/MG90S, metal gears - move SLOWLY)
    GPIO26  -> Gripper twist    (MG90S)
    GPIO25  -> Gripper spin     (MG90S)
    GPIO32  -> Gripper open/close (MG90S) - see GRIPPER_ENABLED below
    GPIO12  -> DELIBERATELY UNUSED (ESP32 strapping pin - do not use for
                                 servo signal, can prevent boot)

  REST POSITIONS (the "home" pose the arm eases back to)
  ---------------------------------------------------------
    Shoulder right : 10 deg   (left mirrors to 170 deg automatically)
    Upper arm      : 160 deg
    Base           : 90 deg
    Tilt / Spin    : 90 deg (centered)
    Gripper        : 90 deg (only relevant once GRIPPER_ENABLED = 1)

  CONTROLLER MAPPING (Xbox pad via Bluepad32)
  ---------------------------------------------
    D-pad Up/Down   -> Shoulder pair (mirrored move)
    D-pad Left/Right-> Upper arm
    RB / LB         -> Tilt  (wrist twist joint on GPIO26)
    RT / LT         -> Spin  (wrist spin joint on GPIO25)
    Hold A or B     -> Ease all active joints back to rest, slowly
    Y               -> LIMP mode: detach all servos (no holding torque).
                        Pressing Y again / any move command re-engages -
                        and per the V11 fix, the arm's internal position
                        variables are snapped to REST before motion resumes,
                        so it doesn't jump back to wherever it physically
                        sagged to while limp.
    Hold X          -> Base eases to 90 deg (home)

  GRIPPER (currently INACTIVE by design)
  -----------------------------------------
    The gripper mechanism (GPIO32) is wired but not yet mechanically
    finished, so its code is included but gated off. To turn it on:

        1. Find the line:   #define GRIPPER_ENABLED 0
        2. Change the 0 to a 1
        3. Re-upload.

    That's it — no other code changes needed. While disabled, the gripper
    servo is never attached and the controller button used for it
    (Left Stick Click / L3) does nothing.

  SERIAL OUTPUT POLICY
  ------------------------
    Serial is kept to diagnostics/events ONLY (connect/disconnect, mode
    changes, errors) — NOT per-frame motion/position spam — so logs stay
    short enough to send when something goes wrong.

  LIBRARIES REQUIRED (Library Manager)
  ----------------------------------------
    - Bluepad32          (by Ricardo Quesada)
    - ESP32Servo
  Board: any ESP32 dev board (e.g. DOIT ESP32 DEVKIT V1)
  ============================================================================
*/

#include <Bluepad32.h>
#include <ESP32Servo.h>

// ============================================================================
// CONFIG — flip this to 1 once the gripper mechanism is ready to drive
// ============================================================================
#define GRIPPER_ENABLED 0   // 0 = gripper code inert, 1 = gripper active

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
const int PIN_SHOULDER_R   = 14;
const int PIN_SHOULDER_L   = 27;
const int PIN_UPPER_ARM    = 13;
const int PIN_BASE         = 33;
const int PIN_GRIPPER_TWIST = 26;  // "tilt" control
const int PIN_GRIPPER_SPIN  = 25;  // "spin" control
const int PIN_GRIPPER_GRAB  = 32;  // open/close - only used if GRIPPER_ENABLED

// GPIO12 intentionally left undefined/unused (ESP32 strapping pin).

// ============================================================================
// REST / HOME POSITIONS (degrees)
// ============================================================================
const int REST_SHOULDER_R = 10;    // shoulder left mirrors: 180 - this
const int REST_UPPER_ARM  = 160;   // hard mechanical ceiling for this joint
const int REST_BASE       = 90;
const int REST_TILT       = 90;
const int REST_SPIN       = 90;
const int REST_GRIPPER    = 90;

// Mechanical safety limit — upper arm must NEVER exceed this
const int UPPER_ARM_MAX = 160;

// ============================================================================
// MOTION TUNING
// ============================================================================
const int STEP_FAST   = 2;   // degrees per update tick for normal manual moves
const int STEP_EASE   = 1;   // degrees per update tick when easing to rest
const int STEP_BASE   = 1;   // base moves slowly - metal gears, protect them
const unsigned long UPDATE_INTERVAL_MS = 20; // ~50 updates/sec

// ============================================================================
// SERVO OBJECTS
// ============================================================================
Servo servoShoulderR;
Servo servoShoulderL;
Servo servoUpperArm;
Servo servoBase;
Servo servoTilt;
Servo servoSpin;
#if GRIPPER_ENABLED
Servo servoGripper;
#endif

// ============================================================================
// LIVE STATE — current commanded angle for each joint
// ============================================================================
int posShoulderR = REST_SHOULDER_R;
int posUpperArm  = REST_UPPER_ARM;
int posBase      = REST_BASE;
int posTilt      = REST_TILT;
int posSpin      = REST_SPIN;
#if GRIPPER_ENABLED
int posGripper   = REST_GRIPPER;
#endif

// Arm mode flags
bool isLimp        = false;  // true while Y-mode (no holding torque) is active
bool easingToRest  = false;  // true while A/B held, moving back to rest
bool baseHoming    = false;  // true while X held, base moving to rest

// Controller connection tracking (diagnostics only)
ControllerPtr controllerPtr = nullptr;

unsigned long lastUpdateTime = 0;

// ============================================================================
// BLUEPAD32 CALLBACKS
// ============================================================================

// Called once when a controller connects
void onConnectedController(ControllerPtr ctl) {
  if (controllerPtr == nullptr) {
    Serial.println("[EVENT] Controller connected.");
    controllerPtr = ctl;
  } else {
    Serial.println("[EVENT] Extra controller connected but only one is used - ignoring.");
  }
}

// Called once when a controller disconnects
void onDisconnectedController(ControllerPtr ctl) {
  if (controllerPtr == ctl) {
    Serial.println("[EVENT] Controller disconnected.");
    controllerPtr = nullptr;
  }
}

// ============================================================================
// HELPER: attach a servo only if it isn't already attached
// ============================================================================
void attachIfNeeded(Servo &s, int pin) {
  if (!s.attached()) {
    s.attach(pin);
  }
}

// ============================================================================
// HELPER: detach a servo (used for LIMP mode - removes holding torque)
// ============================================================================
void detachIfAttached(Servo &s) {
  if (s.attached()) {
    s.detach();
  }
}

// ============================================================================
// Move one joint's stored position by 'step' toward 'target', clamped.
// Returns the new position. Does not write to hardware - call writeAllServos()
// afterward.
// ============================================================================
int stepToward(int current, int target, int step) {
  if (current < target) {
    current += step;
    if (current > target) current = target;
  } else if (current > target) {
    current -= step;
    if (current < target) current = target;
  }
  return current;
}

// ============================================================================
// Push current pos* variables out to the physical servos.
// Shoulder LEFT is derived (mirrored) from shoulder RIGHT here, so callers
// only ever need to update posShoulderR.
// ============================================================================
void writeAllServos() {
  servoShoulderR.write(posShoulderR);
  servoShoulderL.write(180 - posShoulderR);  // mirrored pair, shared axis
  servoUpperArm.write(posUpperArm);
  servoBase.write(posBase);
  servoTilt.write(posTilt);
  servoSpin.write(posSpin);
#if GRIPPER_ENABLED
  servoGripper.write(posGripper);
#endif
}

// ============================================================================
// Re-engage from LIMP mode.
// V11 FIX: snap all position variables to REST before reattaching, so the
// arm doesn't lurch back toward whatever pose it sagged into while limp -
// it always resumes motion FROM the rest pose, not from the physical sag.
// ============================================================================
void exitLimpMode() {
  if (!isLimp) return;

  Serial.println("[EVENT] Exiting LIMP mode - resetting to REST before resuming control.");

  posShoulderR = REST_SHOULDER_R;
  posUpperArm  = REST_UPPER_ARM;
  posBase      = REST_BASE;
  posTilt      = REST_TILT;
  posSpin      = REST_SPIN;
#if GRIPPER_ENABLED
  posGripper   = REST_GRIPPER;
#endif

  attachIfNeeded(servoShoulderR, PIN_SHOULDER_R);
  attachIfNeeded(servoShoulderL, PIN_SHOULDER_L);
  attachIfNeeded(servoUpperArm,  PIN_UPPER_ARM);
  attachIfNeeded(servoBase,      PIN_BASE);
  attachIfNeeded(servoTilt,      PIN_GRIPPER_TWIST);
  attachIfNeeded(servoSpin,      PIN_GRIPPER_SPIN);
#if GRIPPER_ENABLED
  attachIfNeeded(servoGripper,   PIN_GRIPPER_GRAB);
#endif

  writeAllServos();
  isLimp = false;
}

// ============================================================================
// Enter LIMP mode - detach every servo so the arm can be moved by hand
// with no motor resistance / no holding torque.
// ============================================================================
void enterLimpMode() {
  if (isLimp) return;

  Serial.println("[EVENT] Entering LIMP mode - all servos detached.");

  detachIfAttached(servoShoulderR);
  detachIfAttached(servoShoulderL);
  detachIfAttached(servoUpperArm);
  detachIfAttached(servoBase);
  detachIfAttached(servoTilt);
  detachIfAttached(servoSpin);
#if GRIPPER_ENABLED
  detachIfAttached(servoGripper);
#endif

  isLimp = true;
}

// ============================================================================
// Read the controller and update joint targets/state.
// Kept simple and readable rather than clever - easy to tweak per playtest.
// ============================================================================
void handleControllerInput(ControllerPtr ctl) {

  // --- Y button: toggle LIMP mode -----------------------------------------
  if (ctl->y()) {
    if (!isLimp) {
      enterLimpMode();
    }
    // While Y is held down we just stay limp; nothing else to do this tick.
    return;
  } else if (isLimp) {
    // Y released after being limp -> any further input re-engages control.
    // We exit limp mode the moment Y is no longer pressed.
    exitLimpMode();
  }

  // --- Hold X: base eases home to REST_BASE -------------------------------
  if (ctl->x()) {
    baseHoming = true;
  } else {
    baseHoming = false;
  }

  // --- Hold A or B: ease EVERYTHING back to rest --------------------------
  if (ctl->a() || ctl->b()) {
    easingToRest = true;
  } else {
    easingToRest = false;
  }

  // If easing to rest, that overrides manual joint moves this tick.
  if (easingToRest) {
    posShoulderR = stepToward(posShoulderR, REST_SHOULDER_R, STEP_EASE);
    posUpperArm  = stepToward(posUpperArm,  REST_UPPER_ARM,  STEP_EASE);
    posBase      = stepToward(posBase,      REST_BASE,       STEP_EASE);
    posTilt      = stepToward(posTilt,      REST_TILT,       STEP_EASE);
    posSpin      = stepToward(posSpin,      REST_SPIN,       STEP_EASE);
#if GRIPPER_ENABLED
    posGripper   = stepToward(posGripper,   REST_GRIPPER,    STEP_EASE);
#endif
    return; // don't also process D-pad/triggers this tick
  }

  // --- Base homing (X held) - slow, independent of other joints ----------
  if (baseHoming) {
    posBase = stepToward(posBase, REST_BASE, STEP_BASE);
  }

  // --- D-pad Up/Down: shoulder pair (mirrored) ----------------------------
  if (ctl->dpad() & DPAD_UP) {
    posShoulderR += STEP_FAST;
  } else if (ctl->dpad() & DPAD_DOWN) {
    posShoulderR -= STEP_FAST;
  }
  posShoulderR = constrain(posShoulderR, 0, 180);

  // --- D-pad Left/Right: upper arm (hard-capped at UPPER_ARM_MAX) --------
  if (ctl->dpad() & DPAD_LEFT) {
    posUpperArm -= STEP_FAST;
  } else if (ctl->dpad() & DPAD_RIGHT) {
    posUpperArm += STEP_FAST;
  }
  posUpperArm = constrain(posUpperArm, 0, UPPER_ARM_MAX);

  // --- RB / LB: tilt (wrist twist, GPIO26) --------------------------------
  if (ctl->r1()) {
    posTilt += STEP_FAST;
  } else if (ctl->l1()) {
    posTilt -= STEP_FAST;
  }
  posTilt = constrain(posTilt, 0, 180);

  // --- RT / LT: spin (wrist spin, GPIO25) ---------------------------------
  // Bluepad32 reports triggers as an axis 0-1023, not a simple button.
  if (ctl->throttle() > 100) {        // RT pressed
    posSpin += STEP_FAST;
  } else if (ctl->brake() > 100) {    // LT pressed
    posSpin -= STEP_FAST;
  }
  posSpin = constrain(posSpin, 0, 180);

  // --- Base free rotation isn't manually mapped beyond X-hold-home yet;
  //     left as a clear extension point if a free-rotate control is added.

#if GRIPPER_ENABLED
  // --- L3 (left stick click): toggle gripper open/close ------------------
  // Only compiled in once GRIPPER_ENABLED is set to 1.
  static bool lastL3State = false;
  bool l3Now = ctl->thumbL();
  if (l3Now && !lastL3State) {
    posGripper = (posGripper > 90) ? 0 : 180; // simple open/close toggle
  }
  lastL3State = l3Now;
#endif
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("[EVENT] VisionArm V11 booting...");

  // Attach all active servos at startup and move to rest pose.
  servoShoulderR.attach(PIN_SHOULDER_R);
  servoShoulderL.attach(PIN_SHOULDER_L);
  servoUpperArm.attach(PIN_UPPER_ARM);
  servoBase.attach(PIN_BASE);
  servoTilt.attach(PIN_GRIPPER_TWIST);
  servoSpin.attach(PIN_GRIPPER_SPIN);
#if GRIPPER_ENABLED
  servoGripper.attach(PIN_GRIPPER_GRAB);
  Serial.println("[EVENT] Gripper ENABLED - GPIO32 active.");
#else
  Serial.println("[EVENT] Gripper INACTIVE (set GRIPPER_ENABLED to 1 to turn on).");
#endif

  writeAllServos();
  Serial.println("[EVENT] Servos attached, arm at REST pose.");

  // --- Bluepad32 init ------------------------------------------------------
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys(); // avoids stale pairing issues between sessions
  Serial.println("[EVENT] Bluepad32 ready - waiting for Xbox controller...");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Bluepad32 needs to be polled every loop to process controller data.
  bool dataUpdated = BP32.update();
  (void)dataUpdated; // not currently used, kept for clarity/future use

  // Rate-limit motion updates so servo steps are smooth and consistent
  // regardless of how fast loop() itself is spinning.
  unsigned long now = millis();
  if (now - lastUpdateTime < UPDATE_INTERVAL_MS) {
    return;
  }
  lastUpdateTime = now;

  if (controllerPtr != nullptr && controllerPtr->isConnected()) {
    handleControllerInput(controllerPtr);
    if (!isLimp) {
      writeAllServos();
    }
    // While limp, we deliberately skip writeAllServos() - servos are
    // detached and should not receive pulses.
  }
  // If no controller connected, arm simply holds its last commanded pose.
}
