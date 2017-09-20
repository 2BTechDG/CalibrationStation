#ifndef MFC_h
#define MFC_h

#include "Arduino.h"
#include <string.h>
class MFC {
  public:
    int tankC; //Tank Concentration
    int gasC; //Desired Gas Concentration
    String gas; //gas used in system
    double offset; //voltage offset for mfc
    double gain; //gain of mfc
    double desiredFlow; //flow input by user
    int voltIn; // 65535 signal sent to mfc
    int voltOut; // 65535 signal mfc is reading
    double stdFlow; //standard flow using 65535 signal
    bool onOff; //whether mfc is on or off
};

#endif
