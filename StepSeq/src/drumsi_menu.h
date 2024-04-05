#ifndef SINGLETON_H
#define SINGLETON_H

#include <iostream>

class DisplayMenu {
private:
    int name, loves;
    static DisplayMenu* instancePtr;

    DisplayMenu() {}

public:
    DisplayMenu(const DisplayMenu&) = delete;

    static DisplayMenu* getInstance();

    void setValues(int name, int loves);

    void print();
};

#endif // SINGLETON_H
