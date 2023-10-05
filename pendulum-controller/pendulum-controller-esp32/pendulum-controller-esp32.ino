/** 
 * @file pendulum-controller-esp32.ino
 * @brief Inverted pendulum controller for ESP32
 * @author Shawn Hymel
 * @date 2023-09-30
 * 
 * @details
 * Use JSON strings to control the stepper motor and read from the encoder
 * on the ESP32. Used for designing controllers and reinforcement learning AI
 * agents in Python (or other high-level languages).
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

#include "control-comms.hpp"

/******************************************************************************
 * Constants and globals
 */

// Pin definitions
const int LED_PIN = LED_BUILTIN;
const uint16_t STP_EN_PIN = D0;
const uint16_t STP_DIR_PIN = D1;
const uint16_t STP_STEP_PIN = D2;
const uint16_t ENC_A_PIN = D9;     // Green wire
const uint16_t ENC_B_PIN = D10;     // White wire

// Communication constants
static const unsigned int BAUD_RATE = 1000000;
static const ControlComms::DebugLevel CTRL_DEBUG = ControlComms::DEBUG_ERROR;
static constexpr size_t NUM_ACTIONS = 1;
static constexpr size_t NUM_OBS = 2;
static const unsigned int STATUS_OK = 0;
static const unsigned int STATUS_STP_MOVING = 1;
static const unsigned int CMD_SET_HOME = 0;
static const unsigned int CMD_MOVE_TO = 1;
static const unsigned int CMD_MOVE_BY = 2;
static const unsigned int CMD_SET_STEP_MODE = 3;
static const unsigned int CMD_SET_BLOCK_MODE = 4;
static const unsigned int CMD_NOP = 5;
static const unsigned int CMD_MOVE_HOME = 6;

// Stepper constants
const uint32_t STP_MAX_SPEED = 1000;
const uint32_t STP_ACCELERATION = 10000;
const uint16_t STP_DIVS_PER_STEP = 2;
const uint16_t STP_STEPS_PER_ROTATION = 200;
const uint8_t STP_DRIVER_TYPE = 1;
const uint32_t STP_TICK_PERIOD = 100;   // microseconds
const float STP_HARD_LIMIT = 360.0;     // Stepper can't go past 1 rotation
const uint32_t STP_HOME_SPEED = 500;
const uint32_t STP_HOME_ACCELERATION = 300;

// Encoder constants
const int ENC_STEPS_PER_ROTATION = 1200;

// Globals
hw_timer_t *stp_timer = NULL;
AccelStepper stepper = AccelStepper(STP_DRIVER_TYPE, STP_STEP_PIN, STP_DIR_PIN);
RotaryEncoder *encoder = nullptr;
ControlComms ctrl;

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
  Serial.begin(BAUD_RATE);
  ctrl.init(Serial, CTRL_DEBUG);

  // Configure encoder
  encoder = new RotaryEncoder(
    ENC_A_PIN, 
    ENC_B_PIN, 
    RotaryEncoder::LatchMode::TWO03
  );

  // Configure encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderISR, CHANGE);

  // Stepper settings
  stepper.setMaxSpeed(STP_MAX_SPEED);
  stepper.setAcceleration(STP_ACCELERATION);
}

void loop() {

  int command;
  float action[NUM_ACTIONS];
  int status;
  float observation[NUM_OBS];
  ControlComms::StatusCode rx_code;
  static bool blocking = false;

  // Handle stepper
  stepper.run();

  // Receive
  rx_code = ctrl.receive_action<NUM_ACTIONS>(&command, action);
  if (rx_code == ControlComms::OK) {

    // Move the stepper as requested
    switch (command) {
      case CMD_SET_HOME:
        set_stepper_home();
        break;
      case CMD_MOVE_TO:
        move_stepper_to(action[0]);
        break;
      case CMD_MOVE_BY:
        move_stepper_by(action[0]);
        break;
      case CMD_SET_STEP_MODE:
         break;
      case CMD_SET_BLOCK_MODE:
        if (action[0] == 0) {
          blocking = false;
        } else {
          blocking = true;
        }
        break;
      case CMD_NOP:
        break;
      case CMD_MOVE_HOME:
        stepper.setMaxSpeed(STP_HOME_SPEED);
        stepper.setAcceleration(STP_HOME_ACCELERATION);
        move_stepper_to(0.0);
        while (stepper.isRunning()) {
          stepper.run();
        }
        stepper.setMaxSpeed(STP_MAX_SPEED);
        stepper.setAcceleration(STP_ACCELERATION);
      default:
        break;
    }

    // If blocking is set, wait for stepper to finish moving
    if (blocking) {
      while (stepper.isRunning()) {
        stepper.run();
      }
    }

    // Read encoder and stepper angles (in degrees)
    observation[0] = get_encoder_angle();
    observation[1] = get_stepper_angle();

    // Determine motor status
    if (stepper.isRunning()) {
      status = STATUS_STP_MOVING;
    } else {
      status = STATUS_OK;
    }

    // Send back observation
    ctrl.send_observation(status, millis(), false, observation, NUM_OBS);
  
  // Handle receiver error (ignore "RX_EMPTY" case)
  } else if (rx_code == ControlComms::ERROR) {
    Serial.println("Error receiving actions");
  }
}