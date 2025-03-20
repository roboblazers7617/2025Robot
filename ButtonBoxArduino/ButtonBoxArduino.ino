#include <FastLED.h>
#include <Control_Surface.h> // Include the Control Surface library

// Hardware info
const uint8_t NEOPIXEL_PIN = D2;
const uint8_t NEOPIXEL_COUNT = 15;

// Define the array of leds.
Array<CRGB, NEOPIXEL_COUNT> leds{};
bool ledsDirty = false;

// Instantiate a MIDI over USB interface.
USBMIDI_Interface midi;

// Custom MIDI callback that prints incoming SysEx messages.
struct MyMIDI_Callbacks : MIDI_Callbacks {
	// This callback function is called when a SysEx message is received.
	void onSysExMessage(MIDI_Interface&, SysExMessage sysex) override {
		// Print the message
		if (sysex.isCompleteMessage() && sysex.length == 13) {
			for (int i = 0; i < leds.length; i++) {
				leds[i] = CRGB(sysex.data[9], sysex.data[10], sysex.data[11]);
			}
		}
		ledsDirty = true;
	}
} callback{};

void setup() {
	FastLED.addLeds<NEOPIXEL, NEOPIXEL_PIN>(leds.data, leds.length);
	FastLED.setCorrection(TypicalPixelString);

	leds[0] = CRGB::Purple;
	FastLED.show();

	Control_Surface.begin(); // Initialize Control Surface
	midi.setCallbacks(callback);
}

void loop() {
	Control_Surface.loop(); // Update the Control Surface

	// If the colors changed
	if (ledsDirty) {
		FastLED.show();	   // Update the LEDs with the new colors
		ledsDirty = false; // Clear the dirty flag
	}
}
