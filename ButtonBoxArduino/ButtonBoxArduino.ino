#include <FastLED.h>
#include <Control_Surface.h> // Include the Control Surface library
#include "CCAbsoluteEncoderLooped.hpp"

// Hardware info
const uint8_t JOYSTICK_VRx = A1;
const uint8_t JOYSTICK_VRy = A0;
const uint8_t JOYSTICK_SW = D22;
const uint8_t ENCODER_CLK = D21;
const uint8_t ENCODER_DT = D20;
const uint8_t ENCODER_SW = D19;
const uint8_t NEOPIXEL_PIN = D24;
const uint8_t NEOPIXEL_COUNT = 1;

// Define the array of leds.
Array<CRGB, NEOPIXEL_COUNT> leds {};
bool ledsDirty = false;

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

// Custom MIDI callback that prints incoming SysEx messages.
struct MyMIDI_Callbacks : MIDI_Callbacks {
  // This callback function is called when a SysEx message is received.
  void onSysExMessage(MIDI_Interface &, SysExMessage sysex) override {
    // Print the message
    if (sysex.isCompleteMessage() && sysex.length == 13) {
      leds[0] = CRGB(sysex.data[9], sysex.data[10], sysex.data[11]);
    }
    ledsDirty = true;
  }
} callback {};

void setup() {
  FastLED.addLeds<NEOPIXEL, NEOPIXEL_PIN>(leds.data, leds.length);
  FastLED.setCorrection(TypicalPixelString);

  Control_Surface.begin(); // Initialize Control Surface
  midi.setCallbacks(callback);
}

void loop() {
  Control_Surface.loop(); // Update the Control Surface

  if (ledsDirty) { // If the colors changed
    FastLED.show(); // Update the LEDs with the new colors
    ledsDirty = false; // Clear the dirty flag
  }
}