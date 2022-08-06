/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Sat Aug 06 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

vex::controller Controller;

// The drive used for testing had 8 motors
vex::motor motor_lf1(  PORT1, vex::gearSetting::ratio36_1, true );
vex::motor motor_lf2(  PORT3, vex::gearSetting::ratio36_1);

vex::motor motor_lb1(  PORT2, vex::gearSetting::ratio36_1, true );
vex::motor motor_lb2( PORT11, vex::gearSetting::ratio36_1 );

vex::motor motor_rf1( PORT10, vex::gearSetting::ratio36_1 );
vex::motor motor_rf2(  PORT8, vex::gearSetting::ratio36_1, true );

vex::motor motor_rb1(  PORT9, vex::gearSetting::ratio36_1 );
vex::motor motor_rb2( PORT20, vex::gearSetting::ratio36_1, true );

// Adjust as necessary for a 4 motor drive
vex::motor_group  motor_lb( motor_lb1, motor_lb2 );
vex::motor_group  motor_lf( motor_lf1, motor_lf2 );
vex::motor_group  motor_rf( motor_rf1, motor_rf2 );
vex::motor_group  motor_rb( motor_rb1, motor_rb2 );

// An instance of an IMU in port 12
vex::inertial     imu1( PORT12 );

//
// A thread for motor debugging on my 8 motor drive
//
int
motorDebugThread() {
  Brain.Screen.setFont(mono20);

  // show actual velocity of motors
  while(1) {
    Brain.Screen.printAt( 20, 90, "LF1: %4d", (int)motor_lf1.velocity( rpm ) );
    Brain.Screen.printAt( 20,110, "LF2: %4d", (int)motor_lf2.velocity( rpm ) );
    Brain.Screen.printAt( 20,150, "LB1: %4d", (int)motor_lb1.velocity( rpm ) );
    Brain.Screen.printAt( 20,170, "LB2: %4d", (int)motor_lb2.velocity( rpm ) );

    Brain.Screen.printAt( 140, 90, "%-4d :RF1", (int)motor_rf1.velocity( rpm ) );
    Brain.Screen.printAt( 140,110, "%-4d :RF2", (int)motor_rf2.velocity( rpm ) );
    Brain.Screen.printAt( 140,150, "%-4d :RB1", (int)motor_rb1.velocity( rpm ) );
    Brain.Screen.printAt( 140,170, "%-4d :RB2", (int)motor_rb2.velocity( rpm ) );
    this_thread::sleep_for(20);
  }
}
//
// A thread to handle the drive base
//
int
fieldCentrixMecanumDrive(void) {
  static double deg_to_rad = (2 * M_PI) / 360.0;
  
  // wait for imu cal to be complete
  while( imu1.isCalibrating() ) {
    this_thread::sleep_for(20);
  }

  while(1) {
    // get the control inputs
    int ctrl_fwd   = Controller.Axis3.value();
    int ctrl_turn  = Controller.Axis4.value();
    int ctrl_right = Controller.Axis1.value();

    // clip values close to 0
    const int deadband = 15;
    if( abs(ctrl_fwd   ) < deadband ) ctrl_fwd   = 0;
    if( abs(ctrl_turn  ) < deadband ) ctrl_turn  = 0;
    if( abs(ctrl_right ) < deadband ) ctrl_right = 0;

    // Get IMU angle in radians
    double theta = imu1.heading() * deg_to_rad;
      
    // rotate coordinate system - IMU positive angle is CW
    int temp   = ctrl_right * sin(theta) + ctrl_fwd * cos(theta);
    ctrl_right = ctrl_right * cos(theta) - ctrl_fwd * sin(theta);
    ctrl_fwd = temp;
    
    // Set drive motor values
    int drive_l_front = ctrl_fwd + ctrl_turn + ctrl_right;
    int drive_l_back  = ctrl_fwd + ctrl_turn - ctrl_right;

    int drive_r_front = ctrl_fwd - ctrl_turn - ctrl_right;
    int drive_r_back  = ctrl_fwd - ctrl_turn + ctrl_right;

    // optional normalization ---------------------------------------
    // normalize drive so max is max_rpm if any drive is over max_rpm
    int max_rpm = 100;
    int max = abs(drive_l_front);
    if (abs(drive_l_back)  > max)
        max = abs(drive_l_back);
    if (abs(drive_r_back)  > max)
        max = abs(drive_r_back);
    if (abs(drive_r_front) > max)
        max = abs(drive_r_front);

    if (max>max_rpm) {
        drive_l_front = max_rpm * drive_l_front / max;
        drive_l_back  = max_rpm * drive_l_back  / max;
        drive_r_back  = max_rpm * drive_r_back  / max;
        drive_r_front = max_rpm * drive_r_front / max;
    }
    // --------------------------------------------------------------
  
    // send final drive values to motors
    motor_lf.spin( forward, drive_l_front, rpm );
    motor_lb.spin( forward, drive_l_back , rpm );
    motor_rf.spin( forward, drive_r_front, rpm );
    motor_rb.spin( forward, drive_r_back , rpm );

    // no need to run quickly
    this_thread::sleep_for(20);
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // run the drivetrain in a separate thread
  thread t1 = thread( fieldCentrixMecanumDrive );
  // start thread to show motor velocity
  thread t2 = thread( motorDebugThread );

  Brain.Screen.setFont(mono40);
  while(1) {
    Brain.Screen.printAt(20, 50, "IMU Heading %7.2f", imu1.heading());
    this_thread::sleep_for(100);
  }
}
