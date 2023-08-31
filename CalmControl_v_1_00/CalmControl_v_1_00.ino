/*
 * Copyright (C) 2021 Italian Institute of Technology
 *
 * @code Alperen Acemoglu (21-01-2020)
 *
 * 
 * This code was written for TEENSY 3.6 based on Standalone_lmm.cpp
 *
 *
 * All variables functions and main function was written intentionally
 * in a SINGLE '.ino' file, except motion sensor firmware. 
 *
 *
 * THIS CODE IS FOR PARALEL TELEOPERATION AND LOCAL CONTROL
 * 
 * Dedicated motion sensor and Wacom tablet with teleoperantion PC 
 * can be used to control simultanaeously the CALM system.
 * 
 * For enabling teleoperation, teleoperation PC must be connected to 
 * Teensy 3.5 main usb port. 
 * On teleoperation PC, following ROS nodes must be run in different 
 * terminals.
 * 
 * ***********
 * IMPORTANT: Do not forget to add udev rules for Teensy 3.6.
 * For more information: https://www.pjrc.com/teensy/loader_linux.html
 * 
 * 1) ROS communication (main serial port) through ttyACM0 main usb port teensy
 * 
 * DO NOT USE A USB HUB FOR SERIAL CONNECTION WITH TEENSY
 * 
 * >> roscore
 * >> rosrun tablet_bamboo relativeBamboo 
 * >> rosrun teensy_serial teensy_serial 
 * >> rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200
 * 
 * 2) RJ22 connector must be connected to the Lumenis Scanner. 
 * Following ROS node handles scanner control
 * 
 * 3) Binder male connector must be connected to the DeKa HiScan Laser Scanner
 * 
 */

/*
 * roscore
 * rosrun tablet_bamboo relativeBamboo 
 * rosrun teensy_serial teensy_serial 
 * rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200
 * 
 * TURN ON THE CALM
 * 
 */
 
// *********** //
// *** ROS *** //
// *********** //
#include "Arduino.h"
#include <ros.h>   // ros teleoperation
#include <ralp_msgs/teensy_input.h>

ros::NodeHandle  nh;

// ********************* //// ********************* //// ********************* //

// ********************* //
// *** MOTION SENSOR *** //
// ********************* //

// Simple test of USB Host Mouse/Keyboard
// Derived from the Mouse.ino example from the USBHost_t36 library
// Works on teensy 3.6. Should work on teensy 4.0

// This example is in the public domain
#include "USBHost_t36.h"

USBHost myusb;
USBHIDParser hid1(myusb);

MouseController mouse1(myusb);

// Lets look at HID Input devices
USBHIDInput *hiddriver = &mouse1;
#define CNT_HIDDEVICES 1
const char * hid_driver_name = "Mouse";
bool hid_driver_active = false;
// ********************* //// ********************* //// ********************* //

// *************** //
// *** DEFINES *** //
// *************** //
#define CLOCK_PIN             2       // clock            // Output Pin used to implement Synchronous Serial Interface (SSI) to communicate with the rotary encoders of the motor.
#define ENC_X_PIN             4       // encoder x        // Input Pin used to read in position data from X axis rotary encoder
#define ENC_Y_PIN             3       // encoder y        // Input Pin used to read in position data from Y axis rotary encoder
#define BUZZER_PIN            35      // buzzer pin            // digital pin used to create error sound in case of unexpected event.


#define PAT9130_SCALE_X       0.00004 // dont modify
#define PAT9130_SCALE_Y       0.00002 // dont modify


// Tablet Device
#define TABLET_SCALE_X        0.00020   //0.00035 //0.0007 //0.0008  // Defines scale factor of the user position input for x-axis
#define TABLET_SCALE_Y        0.00010   //0.0001  //0.00025 // Defines scale factor of the user position input for y-axis


#define LPF_CUTOFF_INP        40.0
#define LPF_CUTOFF_ENCX       120.0   // Defines the frequency of commands and positions that are removed by the low pass filter, so that the changes in target and motor positions remain smooth for encoder X. radians/sec
#define LPF_CUTOFF_ENCY       120.0   // Defines the frequency of commands and positions that are removed by the low pass filter, so that the changes in target and motor positions remain smooth for encoder Y. radians/sec

#define SAMPLING_TIME               0.001   // Defines the single loop time. In seconds. 0.001 sec corresponds to 1000 Hz

#define GEAR_RATIO_X                56.25   // Defines the gear ratio of the mechanism attached to the X axis motor.
#define GEAR_RATIO_Y                30.00   // Defines the gear ratio of the mechanism attached to the Y axis motor.

// Limit INPUT POSITION commands according to physical mechanism limits
// These values should be less than MOVE_RANGE values
#define POSITION_RANGE_XP           0.3                     // Defines the maximum user position inputs for x-axis // radians 
#define POSITION_RANGE_XN           0.05                     // Defines the minimum user position inputs for x-axis // radians 
#define POSITION_RANGE_YP           1.0                     // Defines the maximum user position inputs for y-axis // radians
#define POSITION_RANGE_YN           0.75                     // Defines the minimum user position inputs for y-axis // radians

// Do not change MOVE_RANGE values
#define MOVE_RANGE_XP               (POSITION_RANGE_XP+0.01)  // Defines the maximum mechanical limit for x-axis.  
#define MOVE_RANGE_XN               (POSITION_RANGE_XN-0.01)  // Defines the minimum mechanical limit for x-axis.  
#define MOVE_RANGE_YP               (POSITION_RANGE_YP+0.01)  // Defines the maximum mechanical limit for y-axis. 
#define MOVE_RANGE_YN               (POSITION_RANGE_YN-0.01)  // Defines the minimim mechanical limit for y-axis.
#define MOVE_ERROR                  0.08                      // Defines the maximum change in position between a previous point and the current target.

#define VOLTAGE_OUT_BOUNDS          5.0    // Defines the maximum absolute voltage 
// (where the sign determines the direction but the absolute voltage remains the same) whereby any calculations resulting in |voltage| > 10 will be set as 0 (whilst maintaining the original sign in a separate variable).
// limiting the output voltage (anti-windup) (V)

#define MOTOR_POW             14      // motor power on/off 
#define MOTOR_SET             39      // SET INPUT 
#define MOTOR_SET2            37      // SET2 OUTPUT
#define PWM_X_PIN             20      // motor x pwm pin  // Output Pin used to send a specific voltage to the X axis motor driver using pulse width modulation.
#define PWM_Y_PIN             22      // motor y pwm pin  // Outpin ut Pin used to send a specific voltage to the Y axis motor driver using pulse width modulation.¬
#define DIR_X_PIN             21      // motor x direction pin // Output Pin used to send a Boolean direction signal to the X axis motor driver.
#define DIR_Y_PIN             23      // motor y direction p// Output Pin used to send a Boolean direction signal to the Y axis motor driver.

#define ANALOG_WRITE_RES      12        //  //12         // PWM Resolution (# of bits)Teensy 3.5 12Bit DAC
#define ANALOG_WRITE_FREQ     14648.437 //  //14648.437  // https://www.pjrc.com/teensy/td_pulse.htm // analog write frequencies // Defines the analog write frequency for PWM and direction pins. 
#define DAC_RESOLUTION        4095      //  //4095       // PWM Value --> 0 - 4095

#define WATCH_DOG             30      // watch dog             // Output Pin used to pulse the watchdog circuit every loop allowing automated response to incorrect activity (e.g. unresponsive code; voltage drop; etc.) via a forced system reset.

#define ROTATE_RIGHT          27      // PINS THAT CONTROL THE SCANNING LINE ROTATION (LUMENIS OR DEKA)
#define ROTATE_LEFT           28      // PINS THAT CONTROL THE SCANNING LINE ROTATION (LUMENIS OR DEKA)
 

typedef struct
{
  volatile int iX;         // An integer values used to read the x position of the optical motion sensor based on Delta_X_L and Delta_X_H
  volatile int iY;         // An integer values used to read the y position of the optical motion sensor based on Delta_Y_L and Delta_Y_H
  volatile int idX;        // second sensor PAT9130 An integer values used to read the x position of the optical motion sensor based on Delta_X_L and Delta_X_H
  volatile int idY;        // second sensor PAT9130 An integer values used to read the y position of the optical motion sensor based on Delta_Y_L and Delta_Y_H
  volatile int idB;        // second sensor buttons PAT9130
} struct_sensor_positions;

typedef struct
{
  int     iTouched;     // An integer value used to store state of the position input device (touched:1 --- not touched:0)
  float   dX;           // A double point used to store relative user position input x-coordinate.
  float   dY;           // A double point used to store relative user position input y-coordinate.
  int     iFlag;        // An integer value used to store state of the button on the position input device (tablet pen) (button is pressed : 1  --- buttoned is not pressed : 0)
  float   dX_lpf;
  float   dY_lpf;
  float   dX_lpf_prev;
  float   dY_lpf_prev;
  int     i_counter;
  int     i_counter2;
} struct_global_input; // __attribute__((__packed__)) struct_global_input;

typedef struct
{
  float fPos;            // A floating point used to store the current position. // raw encoder value
  float fPos_gear;       // A floating point used to store the current position divided by the gear ratio (radians) //raw encoder value with gear ratio
  float fPos_lpf;        // A floating point used to store the low pass filtered values of fPos_gear. // filtered encoder value with gear ratio
  float fPos_lpf_prev;   // A floating point used to store the previous filtered values of fPos_lpf (radians) // previous filtered encoder value with gear ratio
  float fVolt_out;       // A floating point used to store the voltage value that will be sent to the motor driver, where the sign of the variable will determine the direction that the motor should travel.
  float fTarget_temp;    // A floating point used to store the target position in terms of distance (radians) of the motor encoder. This value is set depending on where the command is derived.
  float fSpeed;
  float fSpeed_prev;
  float fAccel;
} struct_motor_data;

typedef struct
{
  float err;                // A floating point used to store the difference between the current motor position and the target position.
  float err_prev;           // A floating point used to store the previous err value so that an error differential can be produced for the time delay control algorithm.
  float err_dot_prev;       // A floating point used to store the previous error differential that was used in the time delay control algorithm. To be used again in the time delay control algorithm.
  float u_old;              // A floating point used to store the previous voltage that was sent to this instance of the motor struct.
  float pid_intg;
  float speed_ref;
  float position_ref_der;
  float speed_err;
  float speed_err_prev;
  float speed_err_integ;
  float torque_ref;
  float accel_err;
  float k_phi;
} struct_motor_control;

typedef struct
{
  int    iTouched;          // An integer value used to store state of the position input device (touched:1 --- not touched:0)
  int    iFlag;             // An integer value used to store state of the button on the position input device (stylus) (button is pressed : 1  --- buttoned is not pressed : 0)
  double dXdelta;           // A double point used to store relative user position input x-coordinate.
  double dYdelta;           // A double point used to store relative user position input y-coordinate.
} struct_tablet_positions;


struct_sensor_positions *stSensor;
struct_global_input     *stInput;
struct_motor_data       *stData[2];
struct_motor_control    *stControl[2];
struct_tablet_positions *stTablet;

// *************************** //
// *** ROS TELEOP CALLBACK *** //
// *************************** //
void callback_tablet (const ralp_msgs::teensy_input& msg){
  int value         = msg.buttons;  
  stTablet->dXdelta = msg.deltax;  
  stTablet->dYdelta = msg.deltay; 

  if (value == 0){
    stTablet->iTouched = 0;
    stTablet->iFlag    = 0;
    }
  else if (value == 1){
    stTablet->iTouched = 1;
    stTablet->iFlag    = 0;    
    }
  else if (value == 65){    // Wacom Tablet Pen Lower Button
    stTablet->iTouched = 1;
    stTablet->iFlag    = 1;  // for scanner rotation right
    }
  else if (value == 129){   // Wacom Tablet Pen Upper Button
    stTablet->iTouched = 1;
    stTablet->iFlag    = 2;  // for scanner rotation left
    }
  else{
    stTablet->iTouched = 0;
    stTablet->iFlag    = 0;
    }
}

// ********************** //
// *** ROS SUBSCRIBER *** //
// ********************** //
ros::Subscriber<ralp_msgs::teensy_input> sub("/ralp_msgs/teensy_input", callback_tablet);



//bool right=false;
//bool left=false;

void setup()
{
  // ROS subscriber
  nh.initNode();
  nh.subscribe(sub);
  
  // Communication with Serial1 Monitor
  Serial1.begin(115200); // dev/ttyUSB0

  //Serial1.println("\n\nUSB Host Testing");
  //Serial1.println(sizeof(USBHub), DEC);
  myusb.begin();


  pinMode(ENC_X_PIN, INPUT);
  pinMode(ENC_Y_PIN, INPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  // PIN MODE BUZZER_PIN : no need to do this.

  stSensor = (struct_sensor_positions*) malloc(sizeof(struct_sensor_positions));
  stSensor->iX      = 0;
  stSensor->iY      = 0;
  stSensor->idX     = 0;
  stSensor->idY     = 0;
  stSensor->idB     = 0;

  stInput = (struct_global_input*) malloc(sizeof(struct_global_input));
  stInput->iTouched     = 0;
  stInput->dX           = 0.0;
  stInput->dY           = 0.0;
  stInput->iFlag        = 0;
  stInput->dX_lpf       = 0.0;
  stInput->dY_lpf       = 0.0;
  stInput->dX_lpf_prev  = 0.0;
  stInput->dY_lpf_prev  = 0.0;
  stInput->i_counter    = 0;
  stInput->i_counter2   = 0;

  stData[0] = (struct_motor_data*) malloc(sizeof(struct_motor_data));
  stData[1] = (struct_motor_data*) malloc(sizeof(struct_motor_data));
  for (int i = 0; i < 2; i++)
  {
    stData[i]->fPos            = 0.0;
    stData[i]->fPos_gear       = 0.0;
    stData[i]->fPos_lpf        = 0.0;
    stData[i]->fPos_lpf_prev   = 0.0;
    stData[i]->fVolt_out       = 0.0;
    stData[i]->fTarget_temp    = 0.0;
    stData[i]->fSpeed          = 0.0;
    stData[i]->fSpeed_prev     = 0.0;
    stData[i]->fAccel          = 0.0;
  }

  // initial encoder values;
  for (int i = 0; i < 20; i++)
  {
    // Read Absolute Encoder
    stData[0]->fPos         = readEncoder(ENC_X_PIN);
    stData[1]->fPos         = readEncoder(ENC_Y_PIN);

    // Encoder values with gear ratio
    stData[0]->fPos_gear    = stData[0]->fPos / GEAR_RATIO_X;
    stData[1]->fPos_gear    = stData[1]->fPos / GEAR_RATIO_Y;

    // filtered encoder values
    stData[0]->fPos_lpf      = stData[0]->fPos_gear;
    stData[1]->fPos_lpf      = stData[1]->fPos_gear;
    stData[0]->fPos_lpf_prev = stData[0]->fPos_gear;
    stData[1]->fPos_lpf_prev = stData[1]->fPos_gear;

    // Clone encoder values to the input variables
    stInput->dX           = stData[0]->fPos_gear;
    stInput->dY           = stData[1]->fPos_gear;
    stInput->dX_lpf       = stData[0]->fPos_gear;
    stInput->dY_lpf       = stData[1]->fPos_gear;
    stInput->dX_lpf_prev  = stData[0]->fPos_gear;
    stInput->dY_lpf_prev  = stData[1]->fPos_gear;
  }

  //Serial1.print(stInput->dX_lpf,6);
  //Serial1.print(" ");
  //Serial1.print(stInput->dY_lpf,6);
  //Serial1.print(" ");
  //Serial1.print(stData[0]->fPos_gear,6);
  //Serial1.print(" ");
  //Serial1.print(stData[1]->fPos_gear,6);
  //Serial1.print(" ");
  //Serial1.print(stData[0]->fPos,6);
  //Serial1.print(" ");
  //Serial1.println(stData[1]->fPos,6);

  stControl[0] = (struct_motor_control*) malloc(sizeof(struct_motor_control));
  stControl[1] = (struct_motor_control*) malloc(sizeof(struct_motor_control));
  for (int i = 0; i < 2; i++)
  {
    stControl[i]->err              = 0.0;
    stControl[i]->err_prev         = 0.0;
    stControl[i]->err_dot_prev     = 0.0;
    stControl[i]->u_old            = 0.0;
    stControl[i]->pid_intg         = 0.0;
    stControl[i]->speed_ref        = 0.0;
    stControl[i]->position_ref_der = 0.0;
    stControl[i]->speed_err        = 0.0;
    stControl[i]->speed_err_prev   = 0.0;
    stControl[i]->speed_err_integ  = 0.0;
    stControl[i]->torque_ref       = 0.0;
    stControl[i]->accel_err        = 0.0;
    stControl[i]->k_phi            = 0.0;
  }

  stTablet = (struct_tablet_positions*) malloc(sizeof(struct_tablet_positions));
  stTablet->iTouched = 0;          // An integer value used to store state of the position input device (touched:1 --- not touched:0)
  stTablet->iFlag    = 0;          // An integer value used to store state of the button on the position input device (stylus) (button is pressed : 1  --- buttoned is not pressed : 0)
  stTablet->dXdelta  = 0.0;        // A double point used to store relative user position input x-coordinate.
  stTablet->dYdelta  = 0.0;  



  // ******************************* //
  // *** DC motor Initialization *** //
  // ******************************* //
  pinMode(MOTOR_SET, INPUT);
  pinMode(MOTOR_SET2, OUTPUT);

  //int setval = 0;
  //setval = digitalRead(MOTOR_SET);
  //Serial1.print("SETVAL = ");
  //Serial1.println(setval);
  digitalWrite(MOTOR_SET2, LOW);
  delayMicroseconds(500000); //delay 500 ms
  digitalWrite(MOTOR_SET2, HIGH); 

  //Motors do NOT have power at the beginning
  pinMode(MOTOR_POW, OUTPUT);
  digitalWrite(MOTOR_POW, LOW);

  // Commercial Scanner rotation pin init
  pinMode(ROTATE_RIGHT, OUTPUT);
  pinMode(ROTATE_LEFT, OUTPUT);

  // INITIALIZE outputs
  digitalWrite(ROTATE_RIGHT, LOW);
  digitalWrite(ROTATE_LEFT, LOW);  

  delay(1000);

  // Motor power on
  digitalWrite(MOTOR_POW,  HIGH);
  //Serial1.println(F("Motor Power ON!"));

  pinMode(PWM_X_PIN, OUTPUT);
  pinMode(PWM_Y_PIN, OUTPUT);

  pinMode(DIR_X_PIN, OUTPUT);
  pinMode(DIR_Y_PIN, OUTPUT);

  // Set analog write frequencies
  analogWriteFrequency(PWM_X_PIN, ANALOG_WRITE_FREQ);
  analogWriteFrequency(PWM_Y_PIN, ANALOG_WRITE_FREQ);

  analogWriteFrequency(DIR_X_PIN, ANALOG_WRITE_FREQ);
  analogWriteFrequency(DIR_Y_PIN, ANALOG_WRITE_FREQ);

  // PWM Resolution (# of bits)
  analogWriteResolution(ANALOG_WRITE_RES);

  // Initial values
  analogWrite(PWM_X_PIN, 0);
  analogWrite(PWM_Y_PIN, 0);

  digitalWrite(DIR_X_PIN, LOW);
  digitalWrite(DIR_Y_PIN, LOW);

  // Buzzer signal: Ready for operation
  unsigned long CONF_DURA = 125; // 125-250 ms

  tone(BUZZER_PIN, 800, CONF_DURA);
  delay(CONF_DURA);
  tone(BUZZER_PIN, 1000, CONF_DURA);
  // ******************************* //

}


void loop()
{
  // time variables
  unsigned long start, stop, us;

  // start timing
  start = micros();

  // --------------------- //
  // --- USER COMMAND  --- //
  // --------------------- //
  
  if (stInput->i_counter == 10)
  {
    // ros
    readTabletRos();
    
    //motion sensor
    myusb.Task();
    if (*hiddriver != hid_driver_active) {
      if (hid_driver_active) {
        Serial1.printf("*** HID Device %s - disconnected ***\n", hid_driver_name);
        hid_driver_active = false;
      } else {
        //Serial1.printf("*** HID Device %s %x:%x - connected ***\n", hid_driver_name, hiddriver->idVendor(), hiddriver->idProduct());
        hid_driver_active = true;

        //const uint8_t *psz = hiddriver->manufacturer();
        //if (psz && *psz) Serial1.printf("  manufacturer: %s\n", psz);
        //psz = hiddriver->product();
        //if (psz && *psz) Serial1.printf("  product: %s\n", psz);
        //psz = hiddriver->serialNumber();
        //if (psz && *psz) Serial1.printf("  Serial: %s\n", psz);
      }
    }

    if (mouse1.available()) {
      stSensor->idX     = mouse1.getMouseX();
      stSensor->idY     = mouse1.getMouseY();
      stSensor->idB     = mouse1.getButtons();     
      mouse1.mouseDataClear();
    } else {
      stSensor->idX     = 0;
      stSensor->idY     = 0;
      stSensor->idB     = 0;
    }

    // Limit Maximum Delta
    int limitPAT = 12;
    if (stSensor->idX > limitPAT) {
      stSensor->idX = limitPAT;
    }
    else if (stSensor->idX < -limitPAT) {
      stSensor->idX = -limitPAT;
    }

    if (stSensor->idY > limitPAT) {
      stSensor->idY = limitPAT;
    }
    else if (stSensor->idY < -limitPAT) {
      stSensor->idY = -limitPAT;
    }

    // X position
    stInput->dX -= stSensor->idX * PAT9130_SCALE_X;

    // Y position
    stInput->dY += stSensor->idY * PAT9130_SCALE_Y;

    // BUTTON Flags
    ////stInput->iFlag    = stSensor->idB;

    // ------------------------ //
    // --- SCANNER ROTATION --- //
    // ------------------------ //
    int rotate_value = 0;

    if (stSensor->idB == 1 || stInput->iFlag == 1)
    {
      rotate_value = 1;
      }

    if (stSensor->idB == 2 || stInput->iFlag == 2)
    {
      rotate_value = 2;
    }

    if (stSensor->idB == 4)
    {
      rotate_value = 4;
    }
    

    if (rotate_value == 0)
    {
      //Serial1.println(" - flag N");
      digitalWrite(ROTATE_LEFT, LOW); 
      digitalWrite(ROTATE_RIGHT, LOW);
    }
    else if (rotate_value == 1)
    {
      //Serial1.println(" - flag R");    // rotate RIGHT
      digitalWrite(ROTATE_LEFT, HIGH); 
      digitalWrite(ROTATE_RIGHT, LOW); 
    }
    else if (rotate_value == 2)
    {
      //Serial1.println(" - flag L");    // rotate LEFT
      digitalWrite(ROTATE_RIGHT, HIGH);
      digitalWrite(ROTATE_LEFT, LOW);
    }
    else if (rotate_value == 4)
    {
      //Serial1.println(" - flag SCALE");    // rotate SCALE
      digitalWrite(ROTATE_RIGHT, HIGH);
      digitalWrite(ROTATE_LEFT, HIGH);
    }
    else
    {
      // DONT DO ANYTHING 
    }
    // ------------------------ // rotation end

    // reset the input counter
    stInput->i_counter = 1;
  }
 
  // Low pass filter for input values
  stInput->dX_lpf = lowPassFilter(stInput->dX, stInput->dX_lpf_prev, LPF_CUTOFF_INP);
  stInput->dY_lpf = lowPassFilter(stInput->dY, stInput->dY_lpf_prev, LPF_CUTOFF_INP);

  // Target reference speed.
  stControl[0]->position_ref_der = (stInput->dX_lpf - stInput->dX_lpf_prev) / SAMPLING_TIME;
  stControl[1]->position_ref_der = (stInput->dY_lpf - stInput->dY_lpf_prev) / SAMPLING_TIME;

  // --- LIMIT WORKSPACE --- //
  // The buzzer warns the user when workspace limits are reached.
  limitWorkspace();

  // Temporary target to control the motors
  stData[0]->fTarget_temp  = stInput->dX_lpf_prev;
  stData[1]->fTarget_temp  = stInput->dY_lpf_prev;

  // ---------------------- // // ------------------------ //
  // --- MOTOR POSITION --- // // --- ENCODER POSITION --- //
  // ---------------------- // // ------------------------ //
  // Read Absolute Encoder
  stData[0]->fPos         = readEncoder(ENC_X_PIN);
  stData[1]->fPos         = readEncoder(ENC_Y_PIN);

  // Encoder values with gear ratio
  stData[0]->fPos_gear    = stData[0]->fPos / GEAR_RATIO_X;
  stData[1]->fPos_gear    = stData[1]->fPos / GEAR_RATIO_Y;

  // Filtered Encoder Values with Gear Ratio
  stData[0]->fPos_lpf     = lowPassFilter(stData[0]->fPos_gear, stData[0]->fPos_lpf_prev, LPF_CUTOFF_ENCX);
  stData[1]->fPos_lpf     = lowPassFilter(stData[1]->fPos_gear, stData[1]->fPos_lpf_prev, LPF_CUTOFF_ENCY);

  // Measured speed
  stData[0]->fSpeed       = (stData[0]->fPos_lpf - stData[0]->fPos_lpf_prev) / SAMPLING_TIME;
  stData[1]->fSpeed       = (stData[1]->fPos_lpf - stData[1]->fPos_lpf_prev) / SAMPLING_TIME;

  // Measured Acceleration
  stData[0]->fAccel       = (stData[0]->fSpeed - stData[0]->fSpeed_prev) / SAMPLING_TIME;
  stData[1]->fAccel       = (stData[1]->fSpeed - stData[1]->fSpeed_prev) / SAMPLING_TIME;

  // ------------------ //
  // --- CONTROLLER --- //
  // ------------------ //

  // Square wave Reference speed
  // if (stInput->i_counter2 == 2000)
  // {
  //  stControl[1]->speed_ref = -stControl[1]->speed_ref;
  //  stInput->i_counter2 = 1;
  // }
  // float freqy = 0.5;
  // stControl[1]->speed_ref = 0.6 * sin(2*M_PI*freqy*millis()*1E-3);

  run_position_controller(0);
  run_position_controller(1);

  run_speed_controller(0);
  run_speed_controller(1);

  run_torque_controller(0);
  run_torque_controller(1);


  // -------------------- //
  // --- COMPANSATION --- //
  // -------------------- //
  // Check the direction of the motion sign(stControl[index]->fError)
  bool compansation_on = true;
  if (compansation_on)
  {
    float Vcompy; // mV
    float cy = 750;

    Vcompy = cy *  fabs(stControl[1]->speed_ref);

    if (stControl[1]->speed_ref < 0)
    {
      stData[1]->fVolt_out -= Vcompy;
    }
    else if (stControl[1]->speed_ref > 0)
    {
      stData[1]->fVolt_out += Vcompy;
    }
    else
    {
      stData[1]->fVolt_out = 0.0;
    }
  } // compansation_on


  // -------------- //
  // --- SAFETY --- //
  // -------------- //
  for (int i = 0; i < 2; i++)
  {
    if (! isSafe(i) )
    {
      // Serial.println(F("UNSAFE Motion!"));
      stData[i]->fVolt_out = 0.0;
      digitalWrite(MOTOR_POW, LOW);
      return;
    }
  }


  // ----------------- //
  // --- MOTOR RUN --- //
  // ----------------- //
  motorDriver(PWM_X_PIN, DIR_X_PIN, (-stData[0]->fVolt_out), 24000);
  motorDriver(PWM_Y_PIN, DIR_Y_PIN, (-stData[1]->fVolt_out), 24000);


  

  
  // --------------------- //
  // --- ROS SPIN ONCE --- //
  // --------------------- //
  nh.spinOnce();

  // ------------- //
  // --- PRINT --- //
  // ------------- //
  // 1 : X and Y  ||  3 : only X roll || 4: only Y pitch
  int iPrintVar = 1;
  switch (iPrintVar) {
    case 0:
      //Serial1.print(stSensor->idB);
      //Serial1.print(" ");
      //Serial1.print(stInput->iFlag);
      //Serial1.print(" ");
      //Serial1.println(" ");
      //int bounds = 1500;
      //Serial1.print(-bounds);
      //Serial1.print(" ");
      //Serial1.print(bounds);
      //Serial1.print(" ");
      //Serial1.print(1E5 * stInput->dX_lpf);     // Reference position
      //Serial1.print(" ");
      //Serial1.println(1E5 * stData[0]->fPos_lpf); // Motor Position
      //Serial1.print(" ");
      //Serial1.print(1E5 * stInput->dY_lpf);     // Reference position
      //Serial1.print(" ");
      //Serial1.println(stInput->iFlag);
      //Serial1.println(1E5 * stData[1]->fPos_lpf); // Motor Position
      break;

    case 1:
      Serial1.print(stInput->iFlag);
      Serial1.print(" ");
      Serial1.print(stInput->dX_lpf);     // Reference position
      Serial1.print(" ");
      Serial1.print(stData[0]->fPos_lpf); // Motor Position
      Serial1.print(" ");
      Serial1.print(stInput->dY_lpf);     // Reference position
      Serial1.print(" ");
      Serial1.println(stData[1]->fPos_lpf); // Motor Position
      break;

    case 2:
      Serial1.print(100 * stControl[0]->speed_ref);  // Reference Speed
      Serial1.print(" ");
      Serial1.println(100 * stData[0]->fSpeed);      // Motor speed
      //Serial1.print(" ");
      //Serial1.println(stData[0]->fVolt_out);      // Calculated voltage
      break;

    case 3:
      //Serial1.print(stSensor->idX);
      //Serial1.print(" ");
      Serial1.print(1E5 * stInput->dX_lpf);       // Reference position
      Serial1.print(" ");
      Serial1.print(1E5 * stData[0]->fPos_lpf); // Motor Position
      Serial1.print(" ");
      Serial1.print(stControl[0]->speed_ref, 6);  // Reference Speed
      Serial1.print(" ");
      Serial1.print(stData[0]->fSpeed, 6);        // Motor speed
      Serial1.print(" ");
      Serial1.println(stData[0]->fVolt_out);      // Calculated voltage
      break;

    case 4:
      Serial1.print(stSensor->idY);
      Serial1.print(" ");
      Serial1.print(1E5 * stInput->dY_lpf);       // Reference position
      Serial1.print(" ");
      Serial1.print(1E5 * stData[1]->fPos_lpf); // Motor Position
      Serial1.print(" ");
      Serial1.print(stControl[1]->speed_ref, 6);  // Reference Speed
      Serial1.print(" ");
      Serial1.print(stData[1]->fSpeed, 6);        // Motor speed
      Serial1.print(" ");
      Serial1.println(stData[1]->fVolt_out);      // Calculated voltage
      break;
  }

  // ----------------------- //
  // --- PREVIOUS VALUES --- //
  // ----------------------- //
  // Previous Filtered Input Values
  stInput->dX_lpf_prev   = stInput->dX_lpf;
  stInput->dY_lpf_prev   = stInput->dY_lpf;

  // Previous Filtered Encoder Values
  stData[0]->fPos_lpf_prev = stData[0]->fPos_lpf;
  stData[1]->fPos_lpf_prev = stData[1]->fPos_lpf;

  // Previous Speed values
  stData[0]->fSpeed_prev   = stData[0]->fSpeed;
  stData[1]->fSpeed_prev   = stData[1]->fSpeed;

  // ---------------------- //
  // --- TIME INCREMENT --- //
  // ---------------------- //
  stInput->i_counter  += 1;
  stInput->i_counter2 += 1;

  // ----------------- //
  // --- WATCH DOG --- //
  // ----------------- //
  watchDogCheck();

  // stop timing
  stop = micros();

  // the time spent for calculation
  us = stop - start;

  // print the time SPENT
  // this value must be smaller than the SAMPLING_TIME
  // Serial1.print(F(" - time [us]: "));
  // Serial1.println(us);

  //Serial1.print(F(" - time [ms]: "));
  //Serial1.println(millis());

  //1 ms delay | or | 1 KHz loop
  delayMicroseconds((SAMPLING_TIME * 1E6 - us));

  // stop timing
  stop = micros();

  // loop time microseconds
  us = stop - start;

  // print the loop time
  //Serial1.print(F(" - c_loop_us: "));
  //Serial1.println(us);

  // loop time seconds
  // float fT = (float) us * (1E-6);

  //Serial1.print(F(" - c_loop_s: "));
  //Serial1.println(stCommand->fT,6);

}


/** A function that is used to filter motor positions and stylus
   positions. This function is an implementation of the low pass
   filter algorithm that passes signals with a frequency lower
   than a selected cutoff frequency and attenuates signals with
   frequencies higher than the cutoff frequency.
   @param [in] q           current data
   @param [in] q_prev      previous data
   @param [in] cutoff_freq cut off frequency

   return fRet filtered value
*/
float lowPassFilter(float q, float q_prev, float cutoff_freq)
{
  float fRet   = q;
  //  q_prev   = q;

  float fNumer = ((cutoff_freq * SAMPLING_TIME * q) +  q_prev);
  float fDenom = (1 + (SAMPLING_TIME * cutoff_freq));
  float q_f    = fNumer / fDenom;
  //q_prev       = q_f;
  fRet         = q_f;

  return fRet;
}


void limitWorkspace()
{
  unsigned int  WARNING_FREQ = 350;          // frequency: the frequency of the tone in hertz. Allowed data types: unsigned int.
  unsigned long WARNING_DURA = 50;           // duration : the duration of the tone in milliseconds (optional). Allowed data types: unsigned long.

  // X position
  if (stInput->dX < POSITION_RANGE_XN)
  {
    tone(BUZZER_PIN, WARNING_FREQ, WARNING_DURA);
    stInput->dX = POSITION_RANGE_XN;
  }
  else if (stInput->dX > POSITION_RANGE_XP)
  {
    tone(BUZZER_PIN, WARNING_FREQ, WARNING_DURA);
    stInput->dX = POSITION_RANGE_XP;
  }

  // Y position
  if (stInput->dY < POSITION_RANGE_YN)
  {
    tone(BUZZER_PIN, WARNING_FREQ, WARNING_DURA);
    stInput->dY = POSITION_RANGE_YN;
  }
  else if (stInput->dY > POSITION_RANGE_YP)
  {
    tone(BUZZER_PIN, WARNING_FREQ, WARNING_DURA);
    stInput->dY = POSITION_RANGE_YP;
  }
}

/** A function that returns a floating point containing the motor position
   in terms of radians from the (0,0) point. readEncoder() uses SSI to
   communicate with the motor encoder on the desired GPIO, the returning
   value is most often used to calculate the error between the target position
   and the actual position.
   @param [in] encPin Encoder pin

   return radOut Encoder position
*/
float readEncoder(int encPin)
{
  noInterrupts(); // very critical! do not remove this line
  delayMicroseconds(25); //VITAL, DO NOT REMOVE
  //Encoder Variables
  const int dataLength = 65;
  float resolution     = 131072;  // 17 bits

  unsigned int buf1    = 0;
  unsigned int buf2    = 0;
  unsigned int buf3    = 0;
  unsigned int bigByte = 0;
  unsigned int data    = 0;

  for (int i = 0; i < dataLength; i++)
  {
    data <<= 1; //shift data left ready for new LSB
    digitalWrite(CLOCK_PIN, LOW);
    delayMicroseconds(1); //Keep clock low for 2uS
    digitalWrite(CLOCK_PIN, HIGH);
    delayMicroseconds(1); //Pull clock high to clock in new bit

    int newBit      = digitalRead(encPin);
    int newBitMatch = digitalRead(encPin);

    bitWrite(data, 0, newBit);

    if (i == 39)
    {
      buf3 = data >> 16;
      buf1 = (data) << 16; //select first 16 bits single turn data
      data = 0;
    }

    if (i == 55)
    {
      buf2 = (data);  //select latter 16 bits single turn data
      data = 0;
    }

    if (newBit != newBitMatch) //Check for discrepancy
    {
      //Serial1.print(F("ERROR! Encoder MisRead! Motor: "));
      if (encPin == ENC_X_PIN)
      {
        //Serial1.println(F("X"));
      }
      else if (encPin == ENC_Y_PIN)
      {
        //Serial1.println(F("Y"));
      }
      else
      {
        //Serial1.println(F("Err"));
      }
    }
  }
  interrupts(); // very critical! do not remove this line

  bigByte       = buf1 | buf2;
  int multiTurn = (buf3 << 17) | (bigByte >> 15);
  float byteOut = (float)multiTurn;

  float radOut  = (byteOut / resolution) * (2 * M_PI);

  return radOut;
}


void controllerPID(int index, float kp)
{
  if (! isSafe(index) )
  {
    // Serial1.println(F("UNSAFE Motion!"));
    stData[index]->fVolt_out = 0.0;
    return;
  }

  // Error
  stControl[index]->err = stData[index]->fTarget_temp - stData[index]->fPos_lpf;

  // Calculate err_dot and err_dbldot from Numerical Diff (below)
  // AND/OR use from 3rd order coeffs
  float err        = stControl[index]->err;
  float err_dot    = (err - stControl[index]->err_prev) / SAMPLING_TIME;
  //float err_dbldot = (err_dot - stControl[index]->err_dot_prev) / SAMPLING_TIME;

  // Integral
  stControl[index]->pid_intg += stControl[index]->err * SAMPLING_TIME;

  //float kp = ?????; // Function input
  float ki = 0;
  float kd = 0;

  float u = kp * err + ki * stControl[index]->pid_intg + kd * err_dot;

  // Safety
  if (u > VOLTAGE_OUT_BOUNDS )
  {
    u = 0;
  }
  if ( u < (-VOLTAGE_OUT_BOUNDS) )
  {
    u = 0;
  }

  stControl[index]->u_old  = u;

  // Set output voltage to motor
  stData[index]->fVolt_out = u * 1000;

  // Previous Values
  stControl[index]->err_prev     = err;
  //stControl[index]->err_dot_prev = err_dot;
}

/** Check if position and position error are within safe zone!
   uses internal structs
   @param   [in]    index   index of motor
   @return                  true, if safe
                            false, otherwise
*/
bool isSafe(int index)
{
  if ( stData[0]->fPos_lpf > MOVE_RANGE_XP )
  {
    Serial1.print(F("X MOTOR NOT +RANGE SAFE: index: "));
    Serial1.print(index);
    Serial1.print(F("  -- X_POS: "));
    Serial1.print(stData[0]->fPos_lpf, 3);
    Serial1.print(F("  -- Y_POS: "));
    Serial1.println(stData[1]->fPos_lpf, 3);
    return false;
  }
  else if ( stData[0]->fPos_lpf < (MOVE_RANGE_XN) )
  {
    Serial1.print(F("X MOTOR NOT -RANGE SAFE: index: "));
    Serial1.print(index);
    Serial1.print(F("  -- X_POS: "));
    Serial1.print(stData[0]->fPos_lpf, 3);
    Serial1.print(F("  -- Y_POS: "));
    Serial1.println(stData[1]->fPos_lpf, 3);
    return false;
  }
  else if ( stData[1]->fPos_lpf > MOVE_RANGE_YP )
  {
    Serial1.print(F("Y MOTOR NOT +RANGE SAFE: index: "));
    Serial1.print(index);
    Serial1.print(F("  -- X_POS: "));
    Serial1.print(stData[0]->fPos_lpf, 3);
    Serial1.print(F("  -- Y_POS: "));
    Serial1.println(stData[1]->fPos_lpf, 3);
    return false;
  }
  else if ( stData[1]->fPos_lpf < (MOVE_RANGE_YN) )
  {
    Serial1.print(F("Y MOTOR NOT -RANGE SAFE: index: "));
    Serial1.print(index);
    Serial1.print(F("  -- X_POS: "));
    Serial1.print(stData[0]->fPos_lpf, 3);
    Serial1.print(F("  -- Y_POS: "));
    Serial1.println(stData[1]->fPos_lpf, 3);
    return false;
  }

  else if ( stControl[index]->err >   MOVE_ERROR )
  {
    Serial1.print(F("NOT +ERROR SAFE: index: "));
    Serial1.print(index);
    Serial1.print(F(" error:  "));
    Serial1.print(stControl[index]->err);
    Serial1.print(F(" "));
    Serial1.print(1000 * stData[0]->fTarget_temp, 3); // input value x
    Serial1.print(F(" "));
    Serial1.print(1000 * stData[0]->fPos_lpf, 3);  // motor position x
    Serial1.print(F("   "));
    Serial1.print(1000 * stData[0]->fPos, 3);  // motor position x without filter
    Serial1.print(F("   "));
    Serial1.print(1000 * stData[1]->fTarget_temp, 3); // input value y
    Serial1.print(F(" "));
    Serial1.print(1000 * stData[1]->fPos_lpf, 3); // motor position y
    Serial1.print(F("   "));
    Serial1.println(1000 * stData[1]->fPos, 3);  // motor position y without filter

    return false;
  }
  else if ( stControl[index]->err < (-MOVE_ERROR) )
  {
    Serial1.print(F("NOT -ERROR SAFE: index: "));
    Serial1.print(index);
    Serial1.print(F(" error:  "));
    Serial1.print(stControl[index]->err);
    Serial1.print(F(" "));
    Serial1.print(1000 * stData[0]->fTarget_temp, 3); // input value x
    Serial1.print(F(" "));
    Serial1.print(1000 * stData[0]->fPos_lpf, 3);  // motor position x
    Serial1.print(F("   "));
    Serial1.print(1000 * stData[0]->fPos, 3);  // motor position x without filter
    Serial1.print(F("   "));
    Serial1.print(1000 * stData[1]->fTarget_temp, 3); // input value y
    Serial1.print(F(" "));
    Serial1.print(1000 * stData[1]->fPos_lpf, 3); // motor position y
    Serial1.print(F("   "));
    Serial1.println(1000 * stData[1]->fPos, 3);  // motor position y without filter
    return false;
  }
  else
    return true;
}


/** A function that calculates the values required to communicate with
    the Motor Driver board and direct the motors to their desired positions.
    fVolt_out is converted to the correct range using the DACResolution
    and the maximum voltage out. The direction of the motor is then
    determined by the sign of the resulting duty cycle.

*/
void motorDriver(int PWM_pin, int DIR_pin, float fVoltOut, float scale_max)   // ALPALP
{
  float ratio      = fVoltOut / scale_max;
  float PWM_Duty   = ratio * DAC_RESOLUTION;


  if (PWM_Duty > 0)
  {
    digitalWrite(DIR_pin, HIGH);
    analogWrite(PWM_pin, fabs(PWM_Duty));
  }
  else if (PWM_Duty < 0)
  {
    digitalWrite(DIR_pin, LOW);
    analogWrite(PWM_pin, fabs(PWM_Duty));
  }
  else
  {
    digitalWrite(DIR_pin, HIGH);
    analogWrite(PWM_pin, 0);
  }
}

// ***************** //
// *** WATCH DOG *** //
// ***************** //
/** A function that sends HIGH and LOW signals to the watch
   in each loop. If watch dog does not receive these signals
   longer than 200 us shuts down the all system.
*/
void watchDogCheck()
{
  digitalWrite(WATCH_DOG, HIGH);
  delayMicroseconds(10);
  digitalWrite(WATCH_DOG, LOW);
  delayMicroseconds(10);
}

// ****************** //
// *** SATURATION *** //
// ****************** //
float saturation(float boundary, float value)
{
  if (value < boundary)
  {
    if (value > -boundary)
      return value;
    else
      return -boundary;
  }
  else
    return boundary;
}


// *************** //
// *** CONTROL *** //
// *************** //

// *************** //// *************** //// *************** //
// control variables
const int K_p_pos[2] = { 150, 1000};
const int speed_saturation[2] = { 1, 1};
const int tot_speed_saturation[2] = { 2, 2};

void run_position_controller(int index)
{
  // Position Error
  stControl[index]->err = stData[index]->fTarget_temp - stData[index]->fPos_lpf;

  stControl[index]->speed_ref = K_p_pos[index] * stControl[index]->err;

  stControl[index]->speed_ref = saturation(tot_speed_saturation[index], saturation(speed_saturation[index], stControl[index]->speed_ref) +  stControl[index]->position_ref_der);
}

// *************** //// *************** //// *************** //
// *************** //// *************** //// *************** //
// *************** //// *************** //// *************** //

const float K_p_speed[2] = { 200, 100};
const float K_i_speed[2] = { 0, 0};
const float K_d_speed[2] = { 0, 0};

const float integration_error_limit[2] = { 1E8, 1E8 }; //
const float integration_sum_limit[2] = { 1E10, 1E10 }; //

void run_speed_controller(int index)
{
  /* Speed errors (P)*/
  stControl[index]->speed_err = stControl[index]->speed_ref - stData[index]->fSpeed;

  /* Accel_error (D) */
  stControl[index]->accel_err      = (stControl[index]->speed_err - stControl[index]->speed_err_prev) / SAMPLING_TIME;

  /* Previous Speed error value */
  stControl[index]->speed_err_prev = stControl[index]->speed_err;

  /* Integrations (I)*/
  if (fabsf(stControl[index]->speed_err) < integration_error_limit[index])
  {
    stControl[index]->speed_err_integ += stControl[index]->speed_err * SAMPLING_TIME;
  }


  stControl[index]->speed_err_integ = saturation(integration_sum_limit[index], stControl[index]->speed_err_integ);

  /* Computation of the Torque reference */
  stControl[index]->torque_ref = K_p_speed[index] * stControl[index]->speed_err + K_i_speed[index] * stControl[index]->speed_err_integ + K_d_speed[index] * stControl[index]->accel_err;
}


// *************** //// *************** //// *************** //
// *************** //// *************** //// *************** //
// *************** //// *************** //// *************** //
const float k_phi_data[2] = { 34.8, 28.9}; // mNm/A
const float voltage_sat[2] = { 2, 3 };
const float terminal_resistance[2] = { 2.84, 1.03 }; // Ohms

void run_torque_controller(int index)
{
  float current;

  /* Compensations */
  stControl[index]->k_phi = k_phi_data[index];

  current = stControl[index]->torque_ref / stControl[index]->k_phi;

  /* Voltage calculations and saturations */
  stData[index]->fVolt_out  = current * terminal_resistance[index];

  stData[index]->fVolt_out = saturation(voltage_sat[index], stData[index]->fVolt_out);

  stData[index]->fVolt_out = 1E3 * stData[index]->fVolt_out; // mV
}



/** A function used to read tablet device x y delta positions 
 * through teleoperation pc with ROS interface.
 * Then, these positions are assigned to a global input variable. 
 * The function also limits the global input variables.
 * 
 */
void readTabletRos()
{  
  // X position
  stInput->dX -= stTablet->dXdelta * TABLET_SCALE_X;
  
  // Y position
  stInput->dY += stTablet->dYdelta * TABLET_SCALE_Y;
    
  stInput->iFlag     = stTablet->iFlag;    // button pressed
  stInput->iTouched  = stTablet->iTouched; // touched
  
  // Print
  bool bPrint = false;
  
  if (bPrint)
  {
    Serial1.print(stTablet->dXdelta);
    Serial1.print(" ");
    Serial1.println(stTablet->dYdelta);
    
    //Serial1.print("x = ");
    //Serial1.print(stInput->dX,6);
    //Serial1.print(" - ");
    //Serial1.print("y = ");
    //Serial1.print(stInput->dY,6);
    //Serial1.print(" - ");
    //Serial1.print("button = ");
    //Serial1.println(stInput->iFlag);
  }
}  
