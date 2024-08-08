#include <Servo.h>
#include <PS4BT.h>
#include <usbhub.h>
#include <SPI.h>
#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>

USB Usb;
// USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the PS4BT class in two ways */
// This will start an inquiry and then pair with the PS4 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS4 controller will then start to blink rapidly indicating that it is in pairing mode
// PS4BT PS4(&Btd, PAIR);
PS4BT PS4(&Btd);

#define TWIST_SERVO_PIN 6
Servo twist_servo;

// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 500000

// Define the motor ID's for drive and lrgs
#define MOTOR_DRIVE 0
#define MOTOR_FRONT_LEG 1
#define MOTOR_REAR_LEG 2

struct ODriveUserData
{
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Ardunio Can setup
HardwareCAN &can_intf = CAN;

bool setupCan()
{
  return can_intf.begin((CanBitRate)CAN_BAUDRATE);
}

// Instantiate ODrive objects
ODriveCAN odrv_drive_motor(wrap_can_intf(can_intf), MOTOR_DRIVE); // Standard CAN message ID
ODriveCAN odrv_front_jump_motor(wrap_can_intf(can_intf), MOTOR_FRONT_LEG); // Standard CAN message ID
ODriveCAN odrv_rear_jump_motor(wrap_can_intf(can_intf), MOTOR_REAR_LEG); // Standard CAN message ID

// Make sure all ODriveCAN instances are accounted for here
ODriveCAN *odrives[] = {&odrv_drive_motor, &odrv_front_jump_motor, &odrv_rear_jump_motor};


// Keep some application-specific user data for every ODrive.
ODriveUserData odrv_user_data_drive_motor;
ODriveUserData odrv_user_data_front_jump_motor;
ODriveUserData odrv_user_data_rear_jump_motor;

//Run through the setup process of odrive motor
void setupOdrive(ODriveCAN *odrv0, ODriveUserData odrv0_user_data, String name)
{
  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0->onFeedback(onFeedback, &odrv0_user_data);
  odrv0->onStatus(onHeartbeat, &odrv0_user_data);

  Serial.println("Waiting for ODrive...");
  while (!odrv0_user_data.received_heartbeat)
  {
    pumpEvents(can_intf);
    delay(100);
  }

  Serial.println("found ODrive");

  // request bus voltage and current (1sec timeout)
  Serial.println("attempting to read bus voltage and current");
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!odrv0->request(vbus, 1))
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
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
  {
    odrv0->clearErrors();
    delay(1);
    odrv0->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
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
      pumpEvents(can_intf);
    }
  }
  

  Serial.println("ODrive | " + name + " | running!");
}

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

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg &msg)
{
  for (auto odrive : odrives)
  {
    onReceive(msg, *odrive);
  }
}

void setupPS4Controller()
{

    if (Usb.Init() == -1) 
    {
      Serial.print(F("\r\nOSC did not start"));
      while (1); // Halt
    }

    Serial.print(F("\r\nPS4 Bluetooth Library Started"));
}

void setup_twist_servo()
{
  twist_servo.attach(TWIST_SERVO_PIN);

  twist_servo.write(90); // default to middle

  Serial.println("Twist Servo Started");

}

void setup()
{

  Serial.begin(115200);

  while (!Serial)
    ; // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection

  Serial.println("Serial Started");

    // your hardware and the CAN stack that you're using.
  if (!setupCan())
  {
    Serial.println("CAN failed to initialize: reset required");
    while (true)
      ; // spin indefinitely
  }


  setupOdrive(&odrv_drive_motor, odrv_user_data_drive_motor,"drive motor");
  // setupOdrive(&odrv_front_jump_motor, &odrv_user_data_front_jump_motor,"front jump motor");
  // setupOdrive(&odrv_rear_jump_motor, &odrv_user_data_rear_jump_motor,"rear jump motor");

  odrv_drive_motor.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);

}

#define UPPDER_DEAD_ZONE 132
#define LOWER_DEAD_ZONE 122

void odrive_sine_loop(ODriveCAN odrv0, ODriveUserData odrv0_user_data)
{
  pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages

  float SINE_PERIOD = 2.0f; // Period of the position command sine wave in seconds

  float t = 0.001 * millis();

  float phase = t * (TWO_PI / SINE_PERIOD);

  odrv0.setPosition(
      sin(phase),                         // position
      cos(phase) * (TWO_PI / SINE_PERIOD) // velocity feedforward (optional)
  );

  // print position and velocity for Serial Plotter
  if (odrv0_user_data.received_feedback)
  {
    Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
    odrv0_user_data.received_feedback = false;
    Serial.print("odrv0-pos:");
    Serial.print(feedback.Pos_Estimate);
    Serial.print(",");
    Serial.print("odrv0-vel:");
    Serial.println(feedback.Vel_Estimate);
  }
}

void odrive_drive_motor(int speed)
{
  pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages

  odrv_drive_motor.setVelocity(speed);

  if (odrv_user_data_drive_motor.received_feedback)
  {
    Get_Encoder_Estimates_msg_t feedback = odrv_user_data_drive_motor.last_feedback;
    odrv_user_data_drive_motor.received_feedback = false;

    Serial.print("odrv0-vel:");
    Serial.println(feedback.Vel_Estimate);
  }
}

void test_odrive()
{
  pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages

  if (Serial.available() > 0)
  {
    Serial.print("Waiting for input: ");
    int i = Serial.parseInt();

    if (i == 0)
      return;

    Serial.print("Set at: ");
    Serial.print(i);
    odrv_drive_motor.setVelocity(i, 0.5);

    //   // request bus voltage and current (1sec timeout)
    // Serial.println("attempting to read bus voltage and current");
    // Get_Bus_Voltage_Current_msg_t vbus;
    // if (!odrv0.request(vbus, 1)) {
    //   Serial.println("vbus request failed!");
    //   //while (true); // spin indefinitely
    // }

    // Serial.print("DC voltage [V]: ");
    // Serial.print(vbus.Bus_Voltage);
    // Serial.print("DC current [A]: ");
    // Serial.print(vbus.Bus_Current);

    // Get_Bus_Voltage_Current_msg_t vbus;
    // odrv0.request(vbus,1);
    // Serial.print( "electrical power [W]");
    // Serial.print(vbus.Bus_Current);

    if (odrv_user_data_drive_motor.received_feedback)
    {
      Get_Encoder_Estimates_msg_t feedback = odrv_user_data_drive_motor.last_feedback;
      odrv_user_data_drive_motor.received_feedback = false;
      Serial.print(" odrv0-vel:");
      Serial.println(feedback.Vel_Estimate);
    }
  }
}

void ps4_eval_hats()
{
  if (PS4.connected())
  {
    uint8_t left_hat_x = PS4.getAnalogHat(LeftHatX);
    uint8_t left_hat_y = PS4.getAnalogHat(LeftHatY);
    uint8_t right_hat_x = PS4.getAnalogHat(RightHatX);
    uint8_t right_hat_y = PS4.getAnalogHat(RightHatY);

    if (left_hat_x > UPPDER_DEAD_ZONE || left_hat_x < LOWER_DEAD_ZONE ||
        left_hat_y > UPPDER_DEAD_ZONE || left_hat_y < LOWER_DEAD_ZONE ||
        right_hat_x > UPPDER_DEAD_ZONE || right_hat_x < LOWER_DEAD_ZONE ||
        right_hat_y > UPPDER_DEAD_ZONE || right_hat_y < LOWER_DEAD_ZONE)
    {
      Serial.print(F("\r\nLeftHatX: "));
      Serial.print(PS4.getAnalogHat(LeftHatX));
      Serial.print(F("\tLeftHatY: "));
      Serial.print(PS4.getAnalogHat(LeftHatY));
      Serial.print(F("\tRightHatX: "));
      Serial.print(PS4.getAnalogHat(RightHatX));
      Serial.print(F("\tRightHatY: "));
      Serial.print(PS4.getAnalogHat(RightHatY));

      uint8_t scaled_left_hat_y = map(left_hat_y, 0, 255, 0, 180);
      twist_servo.write(scaled_left_hat_y);
    }
  }
}

void test_driving_robot()
{

  if (PS4.getButtonClick(RIGHT))
  {
    Serial.print(F("\r\nRight"));
    odrive_drive_motor(1);
  }
  else if (PS4.getButtonClick(LEFT))
  {
    Serial.print(F("\r\nLeft"));
    odrive_drive_motor(-1);
  }
  else
  {
    odrive_drive_motor(0);
  }
}

void loop()
{
  test_odrive();
  // Usb.Task();

  // ps4_eval_hats();

}
