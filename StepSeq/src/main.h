#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>


// pin configuration
const int ENC_PIN_A = 6;
const int ENC_PIN_B = 7;
const int MCP_PIN_INT_A = 26;
const int MCP_PIN_INT_B = 28;
const int SSD_PIN_RESET = 13;
const int BUTTON_PLAY_PIN = 2;
const int BUTTON_PLAY_LED = 38;
const int BUTTON_TRACK_PIN = 3;
const int BUTTON_TRACK_LED = 39;
const int RESET_PIN = 26;
// constants
const byte MIDI_START = 250;
const byte MIDI_CONTINUE = 251;
const byte MIDI_STOP = 252;
const byte MIDI_CLOCK = 248;

// buttons -> shift buttons
enum ShiftButtonState { off, switchTrackActive, switchPatternActive };
enum PlayerState { stopped, playing };

void playback();
void pauseButtonPressed();
void updateTempo(float bpm);

// MCP
uint8_t mcpRead (byte mcpAdress, byte registerAdress);
void mcpWrite (byte mcpAdress, byte registerAdress, byte registerValues);
// MENU
void markMenu(int test);
// SSD1306
void draw(void);
// Player
void beatClock(byte realtimebyte);
void sendMidiNotes(byte spur, byte schritt);
// buttons
void writeStepLed(byte stepNummer, bool anOderAus);
void buttonPressedMcpA();
void buttonPressedMcpB();
void getPressedButtonId(byte woGedrueckt);
void trackInterrupt();
// leds
void seqTrackToLED(byte trackNr);
void seqLauflicht (byte schrittNr);
// Pattern
void updateSequencer(int buttonId);
void togglePlaybackState();
void loadPreset (int whichPreset);
// encoder
void encoderSwitch();

int getPressedButtonId(int identifier);

#endif
