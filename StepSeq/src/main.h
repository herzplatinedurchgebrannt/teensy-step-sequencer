#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

// pin configuration
const int ENC_PIN_A = 6;
const int ENC_PIN_B = 7;
const int MCP_PIN_INT_A = 29;    // before: 26
const int MCP_PIN_INT_B = 28;
const int SSD_PIN_RESET = 13;
const int BTN_PLAY_PIN = 2;
const int BTN_PLAY_LED = 33;     // before: 38
const int BTN_TRACK_PIN = 3;
const int BTN_TRACK_LED = 34;    // before: 39
const int BTN_PATTERN_PIN = 4;
const int BTN_PATTERN_LED = 35;  // before: 40
const int RST_PIN = 26;
const int ST_SCLK = 13;  // SCLK can also use pin 14
const int ST_MOSI = 11;  // MOSI can also use pin 7
const int ST_CS   = 10;  // CS & DC can use pins 2, 6, 9, 10, 15, 20, 21, 22, 23
const int ST_DC   = 9;   //  but certain pairs must NOT be used: 2+10, 6+9, 20+23, 21+22
const int ST_RST  = 8;   // RST can use any pin
const int ST_SDCS = 4;   // CS for SD card, can use any pin

// constants
const int N_TRACKS = 8;
const int N_STEPS = 16;
const char ST_STR_PLAYER[9] = "STATE";    
const char ST_STR_TRACK[9] = "TR";     
const char ST_STR_PATTERN[9] = "PA"; 
const char ST_STR_BPM[9] = "BPM";         
const int ST_X0_OFFSET = 0;   
const int ST_Y0_OFFSET = 30;  
const int ST_MARGIN_1PX = 1; 
const int ST_STEP_WIDTH = 9;  
const int ST_STEP_HEIGHT = 9; 

// state enums
enum PlayerState { PLAYER_STOPPED, PLAYER_PLAYING };
enum ButtonState { BTN_OFF, BTN_CLICKED, BTN_PRESSED, BTN_ON, BTN_RELEASED };

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
void writeLed(uint8_t stepNummer, bool anOderAus);
void mcpAButtonPressedInterrupt();
void mcpBButtonPressedInterrupt();
void getPressedButtonId(uint8_t woGedrueckt);
void selectInterrupt();
// leds
void seqTrackToLED(uint8_t trackNr);
void runLedEffect (byte schrittNr);

// Pattern
void updatePattern(int buttonId);
void togglePlaybackState();
void loadPreset (int whichPreset);
// encoder
void encoderSwitch();

int getPressedButtonId(int identifier);


int identifyStepButton(byte woGedrueckt);
void seqNoteSchreiben(byte noteInBits, int mcpNummer);

void sendMidiNotes();

void drawPlayerState();
void drawTempo();
void drawTrackNum();
void drawPatternNum();
void drawShiftFunction();
void drawSequencerGrid(int delayInMs);
void drawActiveTrack();
void drawActiveSteps();
void drawSequencerStep(int track, int step, bool fill);
#endif
