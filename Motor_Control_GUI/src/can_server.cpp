#include <iostream>
#include <iomanip>  // for std::setw and std::setfill
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <arpa/inet.h>
#include <thread>
#include "../cppzmq/zmq.hpp"
#include <atomic>

uint8_t last_sent_can_id_suffix = 0;

struct MonitoringData { 
    bool forward;
    bool backward;
    bool brake;
    int16_t encoder_speed; //RPM
    int motor_temp;
    int controller_temp;
    uint8_t fault_code;

    float battery_voltage;
    float busbar_current;
};

// Serialize control values into CAN message bytes for sending to the CAN bus
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
// Deserialize driver status CAN messages
void deserialize_can_message(uint32_t can_id, uint8_t* data, MonitoringData& dat) {
    if ((can_id >> 4) == 0x18) {   //// Motor Status Data
        dat.forward = data[0] != 0;
        dat.backward = data[1] != 0;
        dat.brake = data[2] != 0;
        dat.encoder_speed = (static_cast<int16_t>(data[3]) << 8) | data[4];
        dat.motor_temp = static_cast<int>(data[5]) - 40;  // It is 1 deg C with offset 40
        dat.controller_temp = static_cast<int>(data[6]) - 40;  // It is 1 deg C with offset 40
        dat.fault_code = data[7];
    }
    else if ((can_id >> 4) == 0x28) { //// Current/Voltage Data
        float batt_decivolt = (data[0] << 8) | data[1]; // Combine high and low bytes to get the voltage value in V
        //std::cout << "batt_decivolt: " << batt_decivolt << std::endl;
        //std::cout << "data[0]: " << static_cast<int>(data[0]) << std::endl;
        //std::cout << "data[1]: " << static_cast<int>(data[1]) << std::endl;
        float busbar_deciamp = (data[2] << 8) | data[3]; // Combine high and low bytes to get the current value in A
        //std::cout << "busbar_deciamp: " << busbar_deciamp << std::endl;
        //std::cout << "data[2]: " << static_cast<int>(data[2]) << std::endl;
        //std::cout << "data[3]: " << static_cast<int>(data[3]) << std::endl;
        dat.battery_voltage = batt_decivolt * 0.1;
        dat.busbar_current = busbar_deciamp * 0.1;
    }
    else{
        std::cout << "Unknown CAN ID: " << std::hex << can_id << std::endl;
    }
}

std::atomic<bool> running(true);

// Thread: Read CAN and publish monitoring data to Python via ZMQ
void monitoring_data_publish_thread(zmq::socket_t& monitoring_pub_socket, int can_sock) {
    while (running) {
        struct can_frame frame;
        int nbytes = read(can_sock, &frame, sizeof(struct can_frame));
        if (nbytes > 0 && (frame.can_id & CAN_EFF_MASK & 0x0F) == last_sent_can_id_suffix) {
            // Print CAN frame in candump style
            /*std::cout << "can0  " << std::hex << std::uppercase << frame.can_id << "   [" << std::dec << (int)frame.can_dlc << "]  ";
            for (int i = 0; i < frame.can_dlc; ++i) {
                std::cout << std::setw(2) << std::setfill('0') << std::hex << std::uppercase << (int)frame.data[i] << " ";
            }
            std::cout << std::dec << std::nouppercase << std::endl;*/
            MonitoringData dat;
            deserialize_can_message(frame.can_id & CAN_EFF_MASK, frame.data, dat);  // Mask out extended frame flag
            // Print MonitoringData in a human-readable way, showing raw boolean values
            std::cout << "[MonitoringData] CAN_ID: 0x" << std::hex << frame.can_id << std::dec
                      << " | Forward: " << dat.forward
                      << " | Backward: " << dat.backward
                      << " | Brake: " << dat.brake
                      << " | Encoder Speed: " << dat.encoder_speed << " RPM"
                      << " | Motor Temp: " << dat.motor_temp << " C"
                      << " | Controller Temp: " << dat.controller_temp << " C"
                      << " | Fault Code: " << static_cast<int>(dat.fault_code)
                      << " | Battery Voltage: " << dat.battery_voltage << " V"
                      << " | Busbar Current: " << dat.busbar_current << " A"
                      << std::endl;
            // Publish parsed monitoring data fields to Python
            char msg[256];
            snprintf(msg, sizeof(msg), "%03X %d %d %d %d %d %d %d %.2f %.2f",
                frame.can_id,
                dat.forward, dat.backward, dat.brake, dat.encoder_speed,
                dat.motor_temp, dat.controller_temp, dat.fault_code,
                dat.battery_voltage, dat.busbar_current);
            zmq::message_t zmq_msg(msg, strlen(msg));   
            monitoring_pub_socket.send(zmq_msg, zmq::send_flags::none);
        }
    }
}

// Thread: Receive control commands from Python via ZMQ and send to CAN bus
void control_command_receive_thread(zmq::socket_t& control_cmd_socket, int can_sock) {
    while (running) {
        zmq::message_t msg;
        if (control_cmd_socket.recv(msg, zmq::recv_flags::none)) {
            std::string cmd(static_cast<char*>(msg.data()), msg.size());
            int can_id, speed_val, forward, backward, motor, brake;
            if (sscanf(cmd.c_str(), "%x %d %d %d %d %d", &can_id, &speed_val, &forward, &backward, &motor, &brake) == 6) {
                struct can_frame frame;
                frame.can_id = can_id | CAN_EFF_FLAG;  // Set extended frame flag
                last_sent_can_id_suffix = can_id & 0x0F;  // Store suffix from original ID
                frame.can_dlc = 8;
                serialize_can_message(frame.data, speed_val, forward, backward, motor, brake);
                write(can_sock, &frame, sizeof(struct can_frame));
                std::cout << "Sent CAN control command: ID=0x" << std::hex << frame.can_id 
                          << " Data: ";
                for (int i = 0; i < frame.can_dlc; i++) {
                    std::cout << std::setw(2) << std::setfill('0') 
                              << static_cast<int>(frame.data[i]) << " ";
                }
                std::cout << std::dec << std::endl;
            }
        }
    }
}

int main() {
    // ZMQ setup
    zmq::context_t context(1);
    zmq::socket_t control_cmd_socket(context, zmq::socket_type::pull); // Receives control commands from Python
    zmq::socket_t monitoring_pub_socket(context, zmq::socket_type::push); // Publishes monitoring data to Python
    control_cmd_socket.bind("tcp://*:5555"); // Python PUSHes control commands here
    monitoring_pub_socket.bind("tcp://*:5556"); // Python PULLs monitoring data here

    // CAN setup
    int can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    struct ifreq ifr;
    struct sockaddr_can addr;
    strcpy(ifr.ifr_name, "can0");
    ioctl(can_sock, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(can_sock, (struct sockaddr *)&addr, sizeof(addr));

    // Enable extended frame support (CAN 2.0B)
    int enable_canfd = 1;
    setsockopt(can_sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd));

    // Start threads
    std::thread monitoring_thread(monitoring_data_publish_thread, std::ref(monitoring_pub_socket), can_sock);
    std::thread control_thread(control_command_receive_thread, std::ref(control_cmd_socket), can_sock);

    monitoring_thread.join();
    control_thread.join();

    close(can_sock);
    return 0;
}
