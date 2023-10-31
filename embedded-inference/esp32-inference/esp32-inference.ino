/** 
 * @file esp32-inference.ino
 * @brief Swing up pendulum using trained agent and discrete actions map
 * @author Shawn Hymel
 * @date 2023-10-08
 * 
 * @details
 * Deploy trained reinforcement learning agent to the ESP32 using this
 * sketch. Inference is performed using the observations taken and used
 * to determine which action to perform (which way to move the stepper
 * motor). The episode ends when the pendulum swings up softly ("lands"),
 * moves too fast in the area around the up position ("crashes"), or the
 * stepper motor moves +/-180 deg away from start ("out of bounds"). The
 * stepper motor will automatically move back to the home position, wait
 * for a few seconds, and a new episode will begin.
 *
 * @copyright
 * Zero-Clause BSD
 * 
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS” AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE
 * FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN
 * AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "RotaryEncoder.h"
#include "AccelStepper.h"

// Update this to be the name of your Edge Impulse Arduino library
#include <pendulum-rl-conversion_inferencing.h>

/******************************************************************************
 * Constants and globals
 */

// Note: Serial calls without a connection causes delays
// See: https://github.com/espressif/arduino-esp32/issues/6983
#define DEBUG 0

// Actions map
static const float STP_ACTIONS_MAP[] = {-10, 0, 10};

// Pin definitions
static const int LED_PIN = LED_BUILTIN;
static const uint16_t STP_EN_PIN = D0;
static const uint16_t STP_DIR_PIN = D1;
static const uint16_t STP_STEP_PIN = D2;
static const uint16_t ENC_A_PIN = D9;     // Green wire
static const uint16_t ENC_B_PIN = D10;     // White wire

// Communication and demo constants
static const unsigned int BAUD_RATE = 115200;
static const unsigned int TERMINATE_DELAY = 2000;
static const unsigned int RESET_DELAY = 20000;      // 20 sec to let the pendulum fully settle
static const unsigned long EP_TIMEOUT = 30000;      // ms

// Stepper constants
static const uint32_t STP_MAX_SPEED = 1000;
static const uint32_t STP_ACCELERATION = 5000;
static const uint16_t STP_DIVS_PER_STEP = 2;
static const uint16_t STP_STEPS_PER_ROTATION = 200;
static const uint8_t STP_DRIVER_TYPE = 1;
static const uint32_t STP_TICK_PERIOD = 100;   // microseconds
static const float STP_HARD_LIMIT = 360.0;     // Stepper can't go past 1 rotation
static const uint32_t STP_HOME_SPEED = 500;
static const uint32_t STP_HOME_ACCELERATION = 300;

// Angle constants
static const int ENC_STEPS_PER_ROTATION = 1200;
static const float ENC_OFFSET = 180.0;         // 0 deg should be pendulum up
static const float DEGREES_PER_REVOLUTION = 360.0;
static const float ENC_ANGLE_NORM = 180.0;
static const float ENC_GOAL_ANGLE = 5;         // Reward agent and end episode if pendulum is within the goal (+/-5 deg)
static const float ENC_GOAL_VELOCITY = 540;    // ...and velocity is <= this amount (deg/sec)
static const float ENC_CRASH_ANGLE = 45;       // Penalize agent and end episode if pendulum is within the crash zone (+/- 45 deg)
static const float ENC_CRASH_VELOCITY = 540;   // ...and veolicy is > this amount (deg/sec)
static const float STP_ANGLE_MIN = -180.0;     // Episode ends if stepper goes beyond this angle
static const float STP_ANGLE_MAX = 180.0;      // Episode ends if stepper goes beyond this angle
static const float STP_ANGLE_NORM = 180.0;

// Globals
hw_timer_t *stp_timer = NULL;
AccelStepper stepper = AccelStepper(STP_DRIVER_TYPE, STP_STEP_PIN, STP_DIR_PIN);
RotaryEncoder *encoder = nullptr;
static float features[4];

/******************************************************************************
 * Interrupt service routines (ISRs)
 */

// Encoder interrupt service routine (pin change): check state
void encoderISR() {
  encoder->tick();
}

/******************************************************************************
 * Functions
 */

// Get the angle of the encoder in degrees (0 is starting position)
float get_encoder_angle() {
  
  int pos, dir;
  float deg = 0.0;

  // Get position and direction
  pos = encoder->getPosition();
  dir = (int)encoder->getDirection();

  // Convert to degrees
  pos = pos % ENC_STEPS_PER_ROTATION;
  pos = pos >= 0 ? pos : pos + ENC_STEPS_PER_ROTATION;
  deg = (float)pos * (360.0 / ENC_STEPS_PER_ROTATION);

  return deg;
}

// Get the position of the stepper motor (in degrees)
float get_stepper_angle() {
  
  int pos;
  float deg = 0.0;
  
  // Get stepper position (in number of steps)
  pos = stepper.currentPosition();

  // Convert to degrees
  deg = (float)pos * (360.0 / (STP_STEPS_PER_ROTATION * STP_DIVS_PER_STEP));

  return deg;
}

// Tell the stepper to set the current position as "home" (0 deg)
void set_stepper_home() {
  stepper.setCurrentPosition(0);
}

// Move stepper motor by a given amount (in degrees)
void move_stepper_by(float deg) {
  
  long int move_by_divs;

  // Don't go beyond limit
  if ((STP_HARD_LIMIT != 0) && 
    (abs(get_stepper_angle() + deg) > STP_HARD_LIMIT)) {
    return;
  }

  // Calculate number of divisions (round to nearest division)
  move_by_divs = (long int)((((float)(STP_DIVS_PER_STEP * 
    STP_STEPS_PER_ROTATION) / 360.0) * deg) + 0.5);

  // Move stepper
  stepper.move(move_by_divs);
}

// Move stepper motor to a given location (in degress)
void move_stepper_to(float deg) {

  long int move_by_divs;

  // Don't go beyond limit
  if ((STP_HARD_LIMIT != 0) && (abs(deg) > STP_HARD_LIMIT)) {
    return;
  }

  // Calculate number of divisions (round to nearest division)
  move_by_divs = (long int)((((float)(STP_DIVS_PER_STEP * 
    STP_STEPS_PER_ROTATION) / 360.0) * deg) + 0.5);

  // Move stepper
  stepper.moveTo(move_by_divs);
}

// Calculate angular velocity
float calc_angular_velocity(float angle, float angle_prev, float dtime) {

  float da;

  da = angle - angle_prev;
  if (da > (DEGREES_PER_REVOLUTION / 2)) {
    da -= DEGREES_PER_REVOLUTION;
  } else if (da < -1 * (DEGREES_PER_REVOLUTION / 2)) {
    da += DEGREES_PER_REVOLUTION;
  }

  return da / dtime;
}

// Copy raw features from global buffer to inference buffer
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

/******************************************************************************
 * Main
 */

void setup() {
  
  // Configure pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(STP_DIR_PIN, OUTPUT);
  pinMode(STP_STEP_PIN, OUTPUT);
  pinMode(STP_EN_PIN, OUTPUT);
  digitalWrite(STP_EN_PIN, LOW);

  // Initialize our communication interface
#if DEBUG
  Serial.begin(BAUD_RATE);
#endif

  // Configure encoder
  encoder = new RotaryEncoder(
    ENC_A_PIN, 
    ENC_B_PIN, 
    RotaryEncoder::LatchMode::TWO03
  );
  encoder->setPosition(0);

  // Configure encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderISR, CHANGE);

  // Stepper settings
  stepper.setMaxSpeed(STP_MAX_SPEED);
  stepper.setAcceleration(STP_ACCELERATION);
}

void loop() {

  bool ep_running = true;
  float enc_angle;
  float stp_angle;
  float enc_angle_prev = 180.0; // Assume pendulum is pointing down
  float stp_angle_prev = 0.0;
  unsigned long timestamp;
  unsigned long timestamp_prev;
  unsigned long timestamp_ep_start;
  unsigned long timestamp_inference_start;
  unsigned long timestamp_inference_end;
  float itime;
  float dtime;
  float dtheta;
  float dphi;
  unsigned int step_counter = 0;
  ei_impulse_result_t result = {0};
  signal_t features_signal;
  EI_IMPULSE_ERROR resp;
  float max_action_logit;
  uint16_t action_idx = 0;
  static bool first_ep = true;
  
  // Move home
  stepper.setMaxSpeed(STP_HOME_SPEED);
  stepper.setAcceleration(STP_HOME_ACCELERATION);
  move_stepper_to(0.0);
  while (stepper.isRunning()) {
    stepper.run();
  }
  stepper.setMaxSpeed(STP_MAX_SPEED);
  stepper.setAcceleration(STP_ACCELERATION);

  // Wait for pendulum to settle
  if (first_ep) {
    delay(1.0);
    first_ep = false;
  } else {
    delay(RESET_DELAY);
  }

  // There also seems to be a weird bug where the encoder will lose
  // its absolute position after a few episodes, so we will let the
  // pendulum fully settle and then reset the encoder "home"
  encoder->setPosition(0);

  // Get initial observation
  enc_angle_prev = get_encoder_angle();
  stp_angle_prev = get_stepper_angle();
  timestamp = millis();

  // Prevent divide by zero
  delay(10);

  // Perform episode
#if DEBUG
  Serial.println();
  Serial.println("--- Start episode ---");
#endif
  timestamp_ep_start = millis();
  while (ep_running) {
  
    // Keep track of number of steps
    step_counter++;

    // Get observation
    enc_angle = get_encoder_angle();
    stp_angle = get_stepper_angle();
    timestamp = millis();
    dtime = (float)(timestamp - timestamp_prev) / 1000.0;
    timestamp_prev = timestamp;

    // Offset encoder angle so that 0 deg is up
    enc_angle -= ENC_OFFSET;

    // Calculate velocities
    dtheta = calc_angular_velocity(enc_angle, enc_angle_prev, dtime);
    dphi = calc_angular_velocity(stp_angle, stp_angle_prev, dtime);

    // Construct normalized observation as our raw feature set
    features[0] = enc_angle / ENC_ANGLE_NORM;
    features[1] = dtheta / ENC_ANGLE_NORM;
    features[2] = stp_angle / STP_ANGLE_NORM;
    features[3] = dphi / STP_ANGLE_NORM;

    // Save angles for next step
    enc_angle_prev = enc_angle;
    stp_angle_prev = stp_angle;

    // TEST: print raw features
#if DEBUG
    Serial.print("Observation: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(features[i]);
      Serial.print(", ");
    }
    Serial.print("dtime: ");
    Serial.print(dtime, 3);
#endif

    // Do inference (provide observation, get action logits)
    timestamp_inference_start = millis();
    features_signal.total_length = sizeof(features) / sizeof(features[0]);
    features_signal.get_data = &raw_feature_get_data;
    resp = run_classifier(&features_signal, &result, false);
    timestamp_inference_end = millis();
    if (resp != EI_IMPULSE_OK) {
        ei_printf("ERROR: Failed to run classifier (%d)\n", resp);
        return;
    }

    // Find highest logit index (that is our best action to take)
    max_action_logit = -3.4e38; // Close as possible to -inf
    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
      if (result.classification[i].value > max_action_logit) {
        max_action_logit = result.classification[i].value;
        action_idx = i;
      }
    }
    
    // Move stepper motor (blocking)
    move_stepper_by(STP_ACTIONS_MAP[action_idx]);
    while (stepper.isRunning()) {
      stepper.run();
    }

#if DEBUG
    Serial.print(", infer time: ");
    itime = (float)(timestamp_inference_end - timestamp_inference_start) / 1000.0;
    Serial.print(itime, 3);
    Serial.print(", action: ");
    Serial.print(STP_ACTIONS_MAP[action_idx]);
    Serial.println();
#endif

    // Make sure stepper is in bounds
    if ((stp_angle >= STP_ANGLE_MIN) && 
      (stp_angle <= STP_ANGLE_MAX)) {

      // Fail. We "crashed" by swinging too hard
      if ((abs(enc_angle) <= ENC_CRASH_ANGLE) && 
        (abs(dtheta) > ENC_CRASH_VELOCITY)) {
#if DEBUG
        Serial.println("Crashed");
#endif
        ep_running = false;
      
      // Success! We landed softly at the top
      } else if ((abs(enc_angle) <= ENC_GOAL_ANGLE) &&
        (abs(dtheta) <= ENC_GOAL_VELOCITY)) {
#if DEBUG
          Serial.println("Success!");
#endif
          ep_running = false;
      }

    // Stepper motor moved out of bounds
    } else {
#if DEBUG
      Serial.println("Stepper out of bounds");
#endif
      ep_running = false;
    }
    
    // Check for timeout
    if (millis() - timestamp_ep_start > EP_TIMEOUT) {
#if DEBUG
      Serial.println("Episode timed out");
#endif
      ep_running = false;
    }
  }

  // Print metrics
#if DEBUG
  Serial.print("Num steps: ");
  Serial.println(step_counter);
#endif
}
