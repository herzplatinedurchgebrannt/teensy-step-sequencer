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
const uint8_t MIDI_START = 250;
const uint8_t MIDI_CONTINUE = 251;
const uint8_t MIDI_STOP = 252;
const uint8_t MIDI_CLOCK = 248;

// buttons -> shift buttons
enum ShiftButtonState { off, switchTrackActive, switchPatternActive };
enum PlayerState { stopped, playing };

void playback();
void pauseButtonPressed();
void updateTempo(float bpm);

// MCP
uint8_t mcpRead (uint8_t mcpAdress, uint8_t registerAdress);
void mcpWrite (uint8_t mcpAdress, uint8_t registerAdress, uint8_t registerValues);
// MENU
void markMenu(int test);
// SSD1306
void draw(void);
// Player
void beatClock(uint8_t realtimeuint8_t);
void sendMidiNotes(uint8_t spur, uint8_t schritt);
// buttons
void writeStepLed(uint8_t stepNummer, bool anOderAus);
void buttonPressedMcpA();
void buttonPressedMcpB();
void getPressedButtonId(uint8_t woGedrueckt);
void trackInterrupt();
// leds
void seqTrackToLED(uint8_t trackNr);
void seqLauflicht (uint8_t schrittNr);
// Pattern
void updateSequencer(int buttonId);
void togglePlaybackState();
void loadPreset (int whichPreset);
// encoder
void encoderSwitch();

int getPressedButtonId(int identifier);


void buttonsAbfragen(byte woGedrueckt);
void seqNoteSchreiben(byte noteInBits, int mcpNummer);

#endif
