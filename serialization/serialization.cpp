#include <iostream>
#include <iomanip>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <cstdint>

void serialize_can_message(uint8_t* data, int speed_val, bool forward, bool backward, bool motor, bool brake) {
    // Create control byte
    uint8_t control_byte = 0;
    if (motor) control_byte |= 0x08;
    if (brake) control_byte |= 0x04;
    if (backward) control_byte |= 0x02;
    if (forward) control_byte |= 0x01;

    // Convert speed value to high and low bytes
    uint8_t high_byte = (speed_val >> 8) & 0xFF;
    uint8_t low_byte = speed_val & 0xFF;

    // Set message data
    data[0] = control_byte;
    data[1] = high_byte;
    data[2] = low_byte;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
}