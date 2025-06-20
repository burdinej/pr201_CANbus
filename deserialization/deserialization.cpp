#include <iostream>
#include <iomanip>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <cstdint>

struct MonitoringData {
    bool forward;
    bool backward;
    bool brake;
    bool enable;
    int16_t encoder_speed; //RPM
    int motor_temp;
    int controller_temp;
    uint8_t fault_code;

    float battery_voltage;
    float busbar_current;
};

void deserialize_can_message(uint32_t can_id, uint8_t* data, MonitoringData& dat) {
    if ((can_id >> 4) == 0x18) {   //// Motor Status Data
        dat.forward = (data[0] & 0x01) != 0;
        dat.backward = (data[0] & 0x02) != 0;
        dat.brake = (data[0] & 0x04) != 0;
        dat.enable = (data[0] & 0x08) != 0;
        dat.encoder_speed = (static_cast<int16_t>(data[3]) << 8) | data[4];
        dat.motor_temp = static_cast<int>(data[5]) - 40;  // It is 1 deg C with offset 40
        dat.controller_temp = static_cast<int>(data[6]) - 40;  // It is 1 deg C with offset 40
        dat.fault_code = data[7];
    }
    else if ((can_id >> 4) == 0x28) { //// Current/Voltage Data
        float batt_decivolt = (data[0] << 8) | data[1]; // Combine high and low bytes to get the voltage value in V
        float busbar_deciamp = (data[2] << 8) | data[3]; // Combine high and low bytes to get the current value in A
        dat.battery_voltage = batt_decivolt * 0.1;
        dat.busbar_current = busbar_deciamp * 0.1;
    }
    else{
        std::cout << "Unknown CAN ID: " << std::hex << can_id << std::endl;
    }
}