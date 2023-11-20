#pragma once

#include "Arduino.h"
#include "rawhidsensor.h"

class AC58Input : public RawHIDSensor
{
public:
    AC58Input();

    bool dataAvailable() override;
    void readData(byte *buffer) override;
    bool begin() override;
    void setMode(int mode) override;
};
