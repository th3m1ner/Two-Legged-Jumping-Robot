#include "ODriveMotorCAN.hpp"

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t &msg, void *user_data)
{
  ODriveUserData *odrv_user_data = static_cast<ODriveUserData *>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t &msg, void *user_data)
{
  ODriveUserData *odrv_user_data = static_cast<ODriveUserData *>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

ODriveMotorCAN::ODriveMotorCAN(HardwareCAN &can_intf, u_int8_t node_id, String name)
{
  this->name = name;
  this->can_intf = &can_intf;
  odrv = new ODriveCAN(wrap_can_intf(can_intf), node_id);
}

void ODriveMotorCAN::setup()
{
  // Register callbacks for the heartbeat and encoder feedback messages
  odrv->onFeedback(onFeedback, &odrv_user_data);
  odrv->onStatus(onHeartbeat, &odrv_user_data);

  Serial.println("Waiting for ODrive...");
  while (!odrv_user_data.received_heartbeat)
  {
    pumpEvents(*can_intf);
    delay(100);
  }

  Serial.println("found ODrive");

  // disable_can_bus(odrv);

  // request bus voltage and current (1sec timeout)
  Serial.println("attempting to read bus voltage and current");
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!odrv->request(vbus, 1))
  {
    Serial.println("vbus request failed!");
    while (true)
      ; // spin indefinitely
  }

  Serial.print("DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  Serial.println("Enabling closed loop control...");
  while (odrv_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
  {
    odrv->clearErrors();
    delay(1);
    odrv->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    // Pump events for 150ms. This delay is needed for two reasons;
    // 1. If there is an error condition, such as missing DC power, the ODrive might
    //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
    //    on the first heartbeat response, so we want to receive at least two
    //    heartbeats (100ms default interval).
    // 2. If the bus is congested, the setState command won't get through
    //    immediately but can be delayed.
    for (int i = 0; i < 15; ++i)
    {
      delay(10);
      pumpEvents(*can_intf);
    }
  }

  Serial.println("ODrive | " + name + " | running!");
}

bool ODriveMotorCAN::disable_can_bus()
{
  Disable_Can_msg_t msg;
  bool res = odrv->send(msg);
  Serial.println("CAN bus has been disabled on: " + name);
  return res;
}


Get_Bus_Voltage_Current_msg_t ODriveMotorCAN::get_bus_voltage_current()
{
  Get_Bus_Voltage_Current_msg_t vbus;

  if (!odrv->getBusVI(vbus, 1))
    Serial.println("vbus request failed!");

  return vbus;
}

Get_Encoder_Estimates_msg_t ODriveMotorCAN::get_encoder_estimates()
{
  Get_Encoder_Estimates_msg_t feedback; 

  if (odrv_user_data.received_feedback)
  {
    feedback = odrv_user_data.last_feedback;
    odrv_user_data.received_feedback = false;
    return feedback; 
  }

  return feedback; 
}

Get_Powers_msg_t ODriveMotorCAN::get_powers()
{
  Get_Powers_msg_t pwr;

  if (!odrv->getPower(pwr, 1))
    Serial.println("power request failed!");

  return pwr;
}

Get_Temperature_msg_t ODriveMotorCAN::get_temps()
{
   Get_Temperature_msg_t tmp;
   
  if (!odrv->getTemperature(tmp, 1))
    Serial.println("temp request failed!");

  return tmp;
}

void ODriveMotorCAN::print_volt_current_bus(String delim)
{
  Get_Bus_Voltage_Current_msg_t vbus = get_bus_voltage_current();

  Serial.print(" | [V]: ");
  Serial.print(vbus.Bus_Voltage);
  Serial.print(" | [A]: ");
  Serial.print(vbus.Bus_Current);
  Serial.print(" | [W]: ");
  Serial.print(vbus.Bus_Voltage * vbus.Bus_Current);
  Serial.print(delim);
}

void ODriveMotorCAN::print_temps(String delim)
{
  Get_Temperature_msg_t tmp = get_temps();

  Serial.print(" | FET Temp [C]: ");
  Serial.print(tmp.FET_Temperature);
  Serial.print(" | Motor Temp [C]: ");
  Serial.println(tmp.Motor_Temperature);
  Serial.print(delim);
}

void ODriveMotorCAN::print_powers(String delim)
{
  Get_Powers_msg_t pwr = get_powers(); 

  Serial.print(" | Electrical Power [W]: ");
  Serial.print(pwr.Electrical_Power);
  Serial.print(" | Mechcanical Power [N]: ");
  Serial.print(pwr.Mechanical_Power);
  Serial.print(delim);
}

void ODriveMotorCAN::print_pos_vel(String delim)
{
  Get_Encoder_Estimates_msg_t feedback = get_encoder_estimates(); 

  Serial.print(" | pos:");
  Serial.print(feedback.Pos_Estimate);
  Serial.print(" | vel:");
  Serial.print(feedback.Vel_Estimate);
  Serial.print(delim);
}
