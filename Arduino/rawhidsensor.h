#pragma once

#include "Arduino.h"

class RawHIDSensor
{
public:
    virtual bool dataAvailable();
    virtual void readData(byte* buffer);
    virtual bool begin();
    virtual void setMode(int mode);
};
