#include <Servo.h>
#include <PS4BT.h>
#include <usbhub.h>
#include <SPI.h>
#include "ODriveMotorCAN.hpp"
#include <neotimer.h>

USB Usb;
// USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the PS4BT class in two ways */
// This will start an inquiry and then pair with the PS4 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS4 controller will then start to blink rapidly indicating that it is in pairing mode
// PS4BT PS4(&Btd, PAIR);
PS4BT PS4(&Btd);

#define UPPDER_DEAD_ZONE 137
#define LOWER_DEAD_ZONE 117

#define TWIST_SERVO_PIN 6
Servo twist_servo;

// Ardunio Can setup
HardwareCAN &can_intf = CAN;
Neotimer jump_timer = Neotimer(200); // timer for between jump actions 
Neotimer jump_timer_pullup = Neotimer(170); //timer for the pullup action

bool setupCan()
{
  return can_intf.begin((CanBitRate)CAN_BAUDRATE);
}

// Define the motor ID's for drive and lrgs
#define MOTOR_DRIVE 0
#define MOTOR_FRONT_LEG 1
#define MOTOR_REAR_LEG 2

ODriveMotorCAN drive_motor(can_intf,MOTOR_DRIVE, "drive motor");
ODriveMotorCAN front_leg_motor(can_intf, MOTOR_FRONT_LEG, "front leg motor");
ODriveMotorCAN rear_leg_motor(can_intf, MOTOR_REAR_LEG, "rear leg motor");

// Make sure all ODriveCAN instances are accounted for here
ODriveCAN *odrives[] = {drive_motor.odrv, rear_leg_motor.odrv, front_leg_motor.odrv};


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

//float version of the map fuction 
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup()
{

  Serial.begin(115200);

  //must be commented out when running headless or it will wait for the serial to give a response
  // while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection

  Serial.println("Serial Started");

  // your hardware and the CAN stack that you're using.
  if (!setupCan())
  {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }

  drive_motor.setup();
  front_leg_motor.setup();
  rear_leg_motor.setup();

  drive_motor.odrv->setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  drive_motor.odrv->setVelocity(0, 0);
  drive_motor.odrv->setLimits(50, 50);
  Serial.println("Drive Motor controller set to velocity control");

  rear_leg_motor.odrv->setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  rear_leg_motor.odrv->setLimits(50, 50);
  rear_leg_motor.pos_offset = 0.3;
  // rear_leg_motor.odrv->setAbsolutePosition(0);

  Serial.println("Rear Leg controller set to Position control");

  front_leg_motor.odrv->setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  front_leg_motor.odrv->setLimits(50, 50);
  // front_leg_motor.odrv->setAbsolutePosition(0);
  Serial.println("Front Leg controller set to Position control");

  setup_PS4_controller();

  setup_twist_servo();

  digitalWrite(LED_BUILTIN, HIGH);
}

void run_jump_motors(float pos, float vel = 0, float torque = 0)
{
  if(pos > 1.8 || pos < 0)
    return;
  front_leg_motor.odrv->setPosition(pos + front_leg_motor.pos_offset, vel, torque);
  delay(1);//need slight delay between the 2 can messages
  rear_leg_motor.odrv->setPosition(pos + rear_leg_motor.pos_offset, vel, torque);
  delay(1);
}

//jump properties
float low_point = 0.3;
float neutral_point = 1; 
float jump_point = 1.8;
float delay_time = 200;
float delay_time_pullup = 170;


void debug()
{
  // auto vbus1 = front_leg_motor.get_bus_voltage_current(); 
  // auto vbus2 = rear_leg_motor.get_bus_voltage_current(); 
  // Serial.print(vbus1.Bus_Voltage); 
  // Serial.print(" "); 
  // Serial.print(vbus1.Bus_Current);
  // Serial.print(" "); 
  // Serial.println(vbus2.Bus_Current);

  // delay(100);

  if (Serial.available() == 0)
    return; 

  String s = Serial.readString();
  Serial.println(s); 

  if (s.length() < 1)
    return;

  if(s[0] == 'f')
  {
    float val = s.substring(1).toFloat();
    Serial.print("Setting front (1) motor offset to: ");
    front_leg_motor.pos_offset = val;
    Serial.println(val);

  }

  if(s[0] == 'r')
  {
    float val = s.substring(1).toFloat();
    Serial.print("Setting rear (2) motor offset to: ");
    rear_leg_motor.pos_offset = val;
    Serial.println(val);
  }

  if(s[0] == 'j')
  {
    Serial.println("Jump!");
    // max to up point then hold for a few ms then lower back to holding 
    run_jump_motors(1); //start at nutral position
    delay(200);
    run_jump_motors(0.1);// lower to lowest point
    delay(200);
    run_jump_motors(1.8); // run to end point
    delay(200); // time it takes to fully get to end point
    run_jump_motors(1); // reset back to base to land and spring and not overload motors
  }

  if (s[0] == 'l')
  {
    float val = s.substring(1).toFloat();
    Serial.print("Setting Limits to: ");
    Serial.println(val);
    front_leg_motor.odrv->setLimits(val,val);
    rear_leg_motor.odrv->setLimits(val,val);
  }
  

  /*  rear offset 0.3
  low 0 
  high 1.8

  will want to use feedback loop to determine the true time between actions to also get the power feedback

  low of 0.3 seems to prevent it from overshoot to the floor, or i could tune the pid loops 
  c0.3,1,1.8,200s
  */

  if(s[0] == 't') // set jump times
  {
    int delim_1 = s.indexOf(','); //"t100,200"
    int delim_2 = s.indexOf(',', delim_1+1);
    delay_time = s.substring(1,delim_1).toFloat();
    delay_time_pullup = s.substring(delim_1+1,delim_2).toFloat(); 
    jump_timer.set(delay_time);
    jump_timer_pullup.set(delay_time_pullup);

    Serial.print("delay: ");
    Serial.print(delay_time);
    Serial.print(" delay pullup: ");
    Serial.println(delay_time_pullup);
  }

  if(s[0] == 'c')
  {
    //c0.5,1,1.8,200,170
    int delim_1 = s.indexOf(','); //"c0.5,1,2.2,100,100"
    int delim_2 = s.indexOf(',', delim_1+1);
    int delim_3 = s.indexOf(',', delim_2+1);
    int delim_4 = s.indexOf(',', delim_3+1);

    low_point = s.substring(1,delim_1).toFloat();
    neutral_point = s.substring(delim_1+1,delim_2).toFloat(); 
    jump_point = s.substring(delim_2+1, delim_3).toFloat();
    delay_time = s.substring(delim_3+1, delim_4).toFloat();
    delay_time_pullup = s.substring(delim_4+1).toFloat();

    jump_timer.set(delay_time);
    jump_timer_pullup.set(delay_time_pullup);

    Serial.print("Low Point: ");
    Serial.print(low_point);
    Serial.print(" neutral_point: ");
    Serial.print(neutral_point);
    Serial.print(" jump_point: ");
    Serial.print(jump_point);
    Serial.print(" delay: ");
    Serial.print(delay_time);
    Serial.print(" delay pullup: ");
    Serial.println(delay_time_pullup);


    //-_^_-

    // test the jump out
    run_jump_motors(neutral_point); //start at nutral position
    delay(delay_time);
    run_jump_motors(low_point);// lower to lowest point
    delay(delay_time);
    run_jump_motors(jump_point); // run to end point
    delay(delay_time_pullup); // quickly bring up legs
    run_jump_motors(low_point); // reset back to baseto land and spring and not overload motors
    delay(delay_time); // time it takes to fully get to end point
    run_jump_motors(neutral_point); // reset back to baseto land and spring and not overload motors

  }

  if(s[0] == '<')
  {
    float m = s.substring(1).toFloat();
    if(m > 1.8)
      return;
    // Serial.println(m1);
    rear_leg_motor.odrv->setPosition(m, 0.5, 0.5);
  }
  
  if(s[0] == '>')
  {
    float m = s.substring(1).toFloat();
    if(m > 1.8)
      return;
    // Serial.println(m2);
    front_leg_motor.odrv->setPosition(m, 0.5, 0.5);
  }

  if(s[0] == '=')
  {
    float m = s.substring(1).toFloat();
    // Serial.println(m2);
    if(m > 1.8)
      return;
    front_leg_motor.odrv->setPosition(m + front_leg_motor.pos_offset, 0.5, 0.5);
    delay(1);//need slight delay between the 2 can messages
    rear_leg_motor.odrv->setPosition(m + rear_leg_motor.pos_offset, 0.5, 0.5);
  }

  if(s[0]== 's')
  {
    float val = s.substring(1).toFloat();
    Serial.print("Setting twist servo to: ");
    Serial.println(val);
    twist_servo.write(val);
  }

}

int servo_center = 8;
float servo_pos = servo_center;
bool reset_servo = false; 
int drive_speed = 25; 
int servo_limit = 30; 
bool jump_in_progress = false; 
int jump_state = 0;

void drive_robot()
{
  Usb.Task();

  if (!PS4.connected())
    return; 
  
  int left_hat_x = PS4.getAnalogHat(LeftHatX);
  int left_hat_y = PS4.getAnalogHat(LeftHatY);
  int right_hat_y = PS4.getAnalogHat(RightHatY);


  if(left_hat_x > UPPDER_DEAD_ZONE + 20|| left_hat_x < LOWER_DEAD_ZONE - 20)
  {
    left_hat_x = left_hat_x - 127;
    float step_size = 0.01;

    if(left_hat_x > 0 && servo_pos < servo_center + servo_limit)
      servo_pos += step_size;
    
    if(left_hat_x < 0 && servo_pos > servo_center - servo_limit)
      servo_pos -= step_size;

    twist_servo.write(servo_pos);
    // Serial.print("twist value: ");
    // Serial.println(servo_pos);
    reset_servo = true;
  }
  else 
  {
    if(reset_servo)
    {
      twist_servo.write(servo_center);
      servo_pos = servo_center;
      reset_servo = false; 
    }
  }

  if(left_hat_y > UPPDER_DEAD_ZONE + 20|| left_hat_y < LOWER_DEAD_ZONE - 20)
  {
    drive_motor.enabled = true; 
    left_hat_y = left_hat_y - 127; 
    float scaled_left_hat_y = mapf(left_hat_y, -127, 127, -drive_speed,drive_speed);
    // Serial.print("drive value: ");
    // Serial.println(scaled_left_hat_y);
    drive_motor.odrv->setVelocity(scaled_left_hat_y);
    delay(1);

  }
  else
  {
    if(drive_motor.enabled)// so that it only updates once 
    {
      drive_motor.odrv->setVelocity(0);//turn off otherwise
      drive_motor.enabled = false; 
      delay(1);
      // Serial.println("disabled");
    }
  }

  if(right_hat_y > UPPDER_DEAD_ZONE || right_hat_y < LOWER_DEAD_ZONE)
  { 
    float scaled_right_hat_y = mapf(right_hat_y, 0, 255, 0.3, 1.8);
    run_jump_motors(scaled_right_hat_y);
    // Serial.println(scaled_right_hat_y);
  }

  if (PS4.getButtonClick(CROSS))
  {
    if(!jump_in_progress)
    {
      jump_in_progress = true;
    }
  }

  if(jump_in_progress)
  {    
    if(jump_state == 0)
    {
      run_jump_motors(neutral_point); //start at neutral position
      jump_timer.start();
      jump_state = 1;
      Serial.print("Jumping State: 1 -> ");
    }
 
    if(jump_timer.done() && jump_state == 1)
    {
      run_jump_motors(low_point);// lower to lowest point
      jump_state = 2;
      jump_timer.reset();
      jump_timer.start();
      Serial.print("2 -> ");

    }

    if(jump_timer.done() && jump_state == 2)
    {
      run_jump_motors(jump_point); // run to end point
      jump_state = 3;
      jump_timer.reset();
      jump_timer_pullup.start();
      Serial.print("3 -> ");
    }

    if(jump_timer_pullup.done() && jump_state == 3)
    {
      run_jump_motors(low_point); // run back to lowest point using alt timer
      jump_state = 4;
      jump_timer_pullup.reset();
      jump_timer.start();
      Serial.print("4 -> ");
    }

    if(jump_timer.done() && jump_state == 4)
    {
      run_jump_motors(neutral_point); // reset back to base to land, spring and not overload motors
      jump_timer.reset();
      jump_in_progress = false;
      jump_state = 0;
      Serial.println("5 END");
    }
  }

}

void loop()
{
  debug();  
  drive_robot();
}
