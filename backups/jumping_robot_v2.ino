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

class ODriveMotor
{
public:
  ODriveCAN *odrv;
  ODriveUserData odrv_user_data;
  String name;
  // Run through the setup process of odrive motor

  ODriveMotor(int id, String name)
  {
    this->name = name;
    odrv = new ODriveCAN(wrap_can_intf(can_intf), id);
  }

  void setup()
  {
    // Register callbacks for the heartbeat and encoder feedback messages
    odrv->onFeedback(onFeedback, &odrv_user_data);
    odrv->onStatus(onHeartbeat, &odrv_user_data);

    Serial.println("Waiting for ODrive...");
    while (!odrv_user_data.received_heartbeat)
    {
      pumpEvents(can_intf);
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
        pumpEvents(can_intf);
      }
    }

    Serial.println("ODrive | " + name + " | running!");
  }
};

ODriveMotor drive_motor(MOTOR_DRIVE, "drive motor");
ODriveMotor front_leg_motor(MOTOR_FRONT_LEG, "front leg motor");
ODriveMotor rear_leg_motor(MOTOR_REAR_LEG, "rear leg motor");

// Make sure all ODriveCAN instances are accounted for here
ODriveCAN *odrives[] = {drive_motor.odrv, rear_leg_motor.odrv};


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

bool disable_can_bus(ODriveCAN *odrv) {
    Disable_Can_msg_t msg;
    bool res = odrv->send(msg);
    Serial.println("CAN bus has been disabled");
    return res;
}


// Called for every message that arrives on the CAN bus. Function is from the Arduino side.
void onCanMessage(const CanMsg &msg)
{
  for (auto odrive : odrives)
  {
    onReceive(msg, *odrive); 
  }
}

void setup_PS4_controller()
{

  if (Usb.Init() == -1)
  {
    Serial.print(F("\r\nOSC did not start"));
    while (1)
      ; // Halt
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

  drive_motor.setup();
  front_leg_motor.setup();
  rear_leg_motor.setup();

  drive_motor.odrv->setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  drive_motor.odrv->setVelocity(0, 0);
  drive_motor.odrv->setLimits(10, 10);

  Serial.println("Drive Motor controller set to velocity control");

  rear_leg_motor.odrv->setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  rear_leg_motor.odrv->setLimits(10, 10);
  Serial.println("Rear Leg controller set to Position control");

  front_leg_motor.odrv->setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  front_leg_motor.odrv->setLimits(10, 10);
  Serial.println("Front Leg controller set to Position control");

  // setup_PS4_controller();

  setup_twist_servo();
}

#define UPPDER_DEAD_ZONE 132
#define LOWER_DEAD_ZONE 122

void test_odrive()
{

  if (Serial.available() > 0)
  {
    float i = Serial.parseFloat();

    if (i == 0)
      return;

    Serial.println(i);
    pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages
    rear_leg_motor.odrv->setPosition(i, 0.5, 0.5);
  }

  // print position and velocity for Serial Plotter
  if (rear_leg_motor.odrv_user_data.received_feedback)
  {
    Get_Encoder_Estimates_msg_t feedback = rear_leg_motor.odrv_user_data.last_feedback;
    rear_leg_motor.odrv_user_data.received_feedback = false;
    Serial.print("pos:");
    Serial.print(feedback.Pos_Estimate);
    Serial.print(",");
    Serial.print("vel:");
    Serial.print(feedback.Vel_Estimate);
  }

  Get_Bus_Voltage_Current_msg_t vbus;
  if (!rear_leg_motor.odrv->getBusVI(vbus, 1))
  {
    Serial.println("vbus request failed!");
  }

  Serial.print(" | [V]: ");
  Serial.print(vbus.Bus_Voltage);
  Serial.print(" | [A]: ");
  Serial.print(vbus.Bus_Current);
  Serial.print(" | [W]: ");
  Serial.println(vbus.Bus_Voltage * vbus.Bus_Current);

  // Get_Powers_msg_t pwr;
  // if (!rear_leg_motor.odrv->getPower(pwr, 1))
  // {
  //   Serial.println("power request failed!");
  // }

  // Serial.print(" | Electrical Power [W]: ");
  // Serial.print(pwr.Electrical_Power);
  // Serial.print(" | Mechcanical Power [N]: ");
  // Serial.print(pwr.Mechanical_Power);

  // Get_Temperature_msg_t tmp;
  // if (!rear_leg_motor.odrv->getTemperature(tmp, 1))
  // {
  //   Serial.println("temp request failed!");
  // }

  // Serial.print(" | FET Temp [C]: ");
  // Serial.print(tmp.FET_Temperature);
  // Serial.print(" | Motor Temp [C]: ");
  // Serial.println(tmp.Motor_Temperature);

  delay(100);
}

long mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void test_driving_robot()
{
  Usb.Task();

  if (PS4.connected())
  {
    int left_hat_x = PS4.getAnalogHat(LeftHatX);
    int left_hat_y = PS4.getAnalogHat(LeftHatY);
    int right_hat_x = PS4.getAnalogHat(RightHatX);
    int right_hat_y = PS4.getAnalogHat(RightHatY);

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

      float scaled_left_hat_y = mapf(left_hat_y, 0, 255, 0, 20) - 10;
      Serial.print(F("\tdrive value: "));
      Serial.print(scaled_left_hat_y);
      pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages

      drive_motor.odrv->setVelocity(scaled_left_hat_y);

      uint8_t scaled_left_hat_x = map(left_hat_x, 0, 255, 0, 180);
      twist_servo.write(scaled_left_hat_x);
      Serial.print(F("\twist value: "));
      Serial.print(scaled_left_hat_y);
    }
  }
}

void loop()
{
  // test_odrive();
  // test_driving_robot();
}
