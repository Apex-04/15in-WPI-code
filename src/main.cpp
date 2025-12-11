/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Program:      BBR1 15 WPI                                               */
/*    Module:       main.cpp                                                  */
/*    Author:       Andrew Bobay                                              */
/*    Team:         BBR1                                                      */
/*    Created:      Sep. 30th 2025, 2:30 PM                                   */
/*    Modified:     Oct. 23rd 2025, 07:00 PM                                  */
/*    Description:  Janky thrown together code for                            */
/*                   our first skills run at WPI                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;
brain Brain;
controller Controller = controller(primary);

// A global instance of competition
competition Competition;
motor drive_fr = motor(PORT5, ratio6_1, false);
motor drive_br = motor(PORT3, ratio6_1, false);
motor drive_tr = motor(PORT2, ratio6_1, true);
motor drive_fl = motor(PORT6, ratio6_1, true);
motor drive_bl = motor(PORT4, ratio6_1, true);
motor drive_tl = motor(PORT1, ratio6_1, false);


motor_group drive_right = motor_group(drive_fr, drive_br, drive_tr);
motor_group drive_left = motor_group(drive_fl, drive_bl, drive_tl);

inertial IMU = inertial(PORT7);
// Included below are example values, CHANGE THEM FOR YOUR ROBOT
double wheel_travel = 260; // Given by vex
double track_width = 298.45; // Distance between the 2 drive sides
double wheel_base = 177.8; // Distance between the front and back axels
double wheel_c = 10.21; // Circumfrence of wheels
double gear_ratio = (36/48); // (Motorin/MotorOut)
smartdrive Drivetrain = smartdrive(drive_left, drive_right, IMU, wheel_travel, track_width, wheel_base, mm, gear_ratio);


motor intake_left = motor(PORT18, ratio6_1, false);
motor intake_right = motor(PORT11, ratio6_1, true);
motor outtake_lower = motor(PORT19, ratio18_1, true);
motor outtake_upper = motor(PORT20, ratio6_1, true); 
motor_group intake = motor_group(intake_left, intake_right, outtake_lower, outtake_upper);


pneumatics liftR = pneumatics(Brain.ThreeWirePort.G);
pneumatics liftL = pneumatics(Brain.ThreeWirePort.F);

pneumatics door = pneumatics(Brain.ThreeWirePort.H);


// define control booleans for driver control here
bool drive_right_bool = true;
bool drive_left_bool = true;
bool lift_toggle_bool = true;
bool door_toggle_bool = false;
bool intake_control_bool = true;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

// Basic Intertial Calibration 
Brain.Screen.print("Device Init...");
Brain.Screen.setCursor(2,1);
wait(200, msec);
IMU.calibrate(0);
Brain.Screen.print("Calibrating inertial");
while (IMU.isCalibrating()){
  wait(25, msec);
}
Brain.Screen.setCursor(3,1);
Brain.Screen.print("Inertial Calibration Complete");

// Set Velocity and clear encoders 
Drivetrain.setDriveVelocity(100, percent);
intake.setVelocity(100,percent);
}

/*---------------------------------------------------------------------------*/
/*                          Autonomous Functions                             */
/*                                                                           */
/*  Use This space to define functions to be used in autonomous,             */
/*  This includes control loops like PID, or Odomtry                         */                                                                    
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  intake.spin(reverse);
  Drivetrain.stop(hold);
  
  /* Skills Planning 
  1R - 0B
  drive to match loader
  4R - 3B
  drive to goal
  dump
  0R - 0B
  Grab 2 blocks on wall
  0R - 2B
  drive to goal
  dump
  0R - 0B
  Drive to park Zone
  Clear Park zone
  Park

  39 Solo
  
  */
  
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    
    // ........................................................................

    int direction = fabs(Controller.Axis1.position())/Controller.Axis1.position();
    // Example Arcade stick control, I reccomend tuning some of these values to fit your driving style
    
      int drivetrainRightSideSpeed = (Controller.Axis3.position() - (pow(1.047,fabs(Controller.Axis1.position()))*direction));
      int drivetrainLeftSideSpeed = (Controller.Axis3.position() + (pow(1.047, fabs(Controller.Axis1.position()))*direction));


      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        if (drive_left_bool) {
          drive_left.stop();
          drive_left_bool = false;
        }
      } else {
        drive_left_bool = true;
      }
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        if (drive_right_bool) {
          drive_right.stop();
          drive_right_bool = false;
        }
      } else {
        drive_right_bool = true;
      }

      if (drive_left_bool) {
        drive_left.setVelocity(drivetrainLeftSideSpeed, percent);
        drive_left.spin(forward);
      }

      if (drive_right_bool) {
        drive_right.setVelocity(drivetrainRightSideSpeed, percent);
        drive_right.spin(forward);
      }

      // Intake Contoller
      if (Controller.ButtonR1.pressing()) {
        intake.spin(fwd);
        intake_control_bool = false;
      } else if (Controller.ButtonR2.pressing()) {
        intake.spin(reverse);
        intake_control_bool = false;
      } else if (!intake_control_bool) {
        intake.stop();
        intake_control_bool = true;
      }

     // Intake Contoller
      if (Controller.ButtonL1.pressing()) {
        liftL.open();
        liftR.open();
        lift_toggle_bool = false;
        } else if (!lift_toggle_bool) {
          liftR.close();
          liftL.close();
          lift_toggle_bool = true;
     
      }

      // Basic Toggle Controller
      if (Controller.ButtonL1.pressing()) {
        door_toggle_bool = !door_toggle_bool;
        while (Controller.ButtonL1.pressing()){}
          if (door_toggle_bool) {
            door.open();
          } else {
            door.close();
          }
        }

    



    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }

}
//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}