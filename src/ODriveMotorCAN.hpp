#pragma once

#include <Arduino.h>
#include <ODriveHardwareCAN.hpp>
#include <Arduino_CAN.h>

#define CAN_BAUDRATE 500000


struct ODriveUserData
{
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

//Command to disable CAN from the controller. Closed loop must be disabled before running 
struct Disable_Can_msg_t final {
    constexpr Disable_Can_msg_t() = default;
    
    void encode_buf(uint8_t* buf) const {
        can_set_signal_raw<uint8_t>(buf, Identify, 0, 8, true);
    }

    void decode_buf(const uint8_t* buf) {
        Identify = can_get_signal_raw<uint8_t>(buf, 0, 8, true);
    }

    static const uint8_t cmd_id = 0x01E;
    static const uint8_t msg_length = 1;
    
    uint8_t Identify = 0;
};

class ODriveMotorCAN
{
    public:
    ODriveCAN *odrv;
    ODriveUserData odrv_user_data;
    String name;
    HardwareCAN *can_intf;
    float pos_offset = 0;
    bool enabled = false; 

    // Run through the setup process of odrive motor

    ODriveMotorCAN(HardwareCAN &can_intf,u_int8_t node_id, String name); 
    void setup();
    bool disable_can_bus();

    Get_Bus_Voltage_Current_msg_t get_bus_voltage_current(); 
    Get_Encoder_Estimates_msg_t get_encoder_estimates(); 
    Get_Powers_msg_t get_powers(); 
    Get_Temperature_msg_t get_temps(); 

    void print_volt_current_bus(String delim = "\n"); 
    void print_temps(String delim = "\n");
    void print_powers(String delim = "\n"); 
    void print_pos_vel(String delim = "\n"); 
};
