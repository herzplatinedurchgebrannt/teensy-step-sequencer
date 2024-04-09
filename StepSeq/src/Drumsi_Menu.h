#ifndef DISPLAYMENU_H
#define DISPLAYMENU_H

#include <iostream>

class DisplayMenu {

  public:
    static DisplayMenu& getInstance();
    void someMethod();

    enum MenuSelection {SPUR, TEMPO, SAVE, PATTERN, MIDI_NOTE, MIDI_VELOCITY};
    
  private:
    DisplayMenu(); // Private constructor to prevent instantiation
    DisplayMenu(const DisplayMenu&) = delete; // Delete copy constructor
    void operator=(const DisplayMenu&) = delete; // Delete assignment operator
};

#endif // DISPLAYMENU_H




