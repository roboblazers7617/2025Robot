#include <Control_Surface.h> // Include the Control Surface library
#include "CCAbsoluteEncoderLooped.hpp"

// Hardware info
const uint8_t JOYSTICK_VRx = A1;
const uint8_t JOYSTICK_VRy = A0;
const uint8_t JOYSTICK_SW = D22;
const uint8_t ENCODER_CLK = D21;
const uint8_t ENCODER_DT = D20;
const uint8_t ENCODER_SW = D19;

// Instantiate a MIDI over USB interface.
USBMIDI_Interface midi;

CCAbsoluteEncoderLooped wheelEncoder {
  {ENCODER_CLK, ENCODER_DT},
  0, // MIDI CC
  1, // Multiplier
  2, // Pulses per step
};

CCPotentiometer joystick[] {
  {JOYSTICK_VRx, 1},
  {JOYSTICK_VRy, 2},
};

NoteButton joystickButton {
  JOYSTICK_SW,
  0,
};

void setup() {
  Control_Surface.begin(); // Initialize Control Surface
}

void loop() {
  Control_Surface.loop(); // Update the Control Surface
}