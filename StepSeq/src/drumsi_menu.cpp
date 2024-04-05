
#include <bits/stdc++.h>

class DisplayMenu{
  private:
   
  // member variables
  int name, loves; 
     
  static DisplayMenu* instancePtr; 
   
  // Default constructor
  DisplayMenu() 
  {
  }
   
  public:
   
  // deleting copy constructor
  DisplayMenu(const DisplayMenu& obj)
    = delete; 
 
  /*
    getInstance() is a static method that returns an
    instance when it is invoked. It returns the same
    instance if it is invoked more than once as an instance
    of Singleton class is already created. It is static
    because we have to invoke this method without any object
    of Singleton class and static method can be invoked
    without object of class
 
    As constructor is private so we cannot create object of
    Singleton class without a static method as they can be
    called without objects. We have to create an instance of
    this Singleton class by using getInstance() method.
  */
  static DisplayMenu* getInstance()
  {
    // If there is no instance of class
    // then we can create an instance.
    if (instancePtr == NULL) 
    {
      // We can access private members 
      // within the class.
      instancePtr = new DisplayMenu(); 
       
      // returning the instance pointer
      return instancePtr; 
    }
    else
    {
      // if instancePtr != NULL that means 
      // the class already have an instance. 
      // So, we are returning that instance 
      // and not creating new one.
      return instancePtr;
    }
  }
 
  // sets values of member variables.
  void setValues(int name, 
                 int loves) 
  {
    this->name = name;
    this->loves = loves;
  }
   
  // prints values of member variables
  void print() 
  {
    std::cout << name << " Loves " << 
            loves << "." << std::endl;
  }
};