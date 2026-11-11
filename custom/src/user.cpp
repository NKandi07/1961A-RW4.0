#include "vex.h"
#include "motor-control.h"
#include "driver-control.h"
#include "auton-selector.h"
#include "drive-log.h"
#include "../custom/include/autonomous.h"
#include "../custom/include/robot-config.h"

// Modify autonomous, driver, or pre-auton code below

void runAutonomous() {
  // Runs the routine picked on the Brain screen (or with the controller arrows while disabled).
  // Add routines to the list in runPreAutonomous() below.
  runSelectedAuton();
}

// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;

// Pneumatic toggle states and rising-edge tracking variables
// claw_state: false = closed (initial default), true = open
bool claw_state = false;
bool last_button_a = false;

// toggle_piston_state: false = retracted (initial default), true = extended
bool toggle_piston_state = false;
bool last_button_left_arrow = false;

void runDriver() {
  stopChassis(coast);
  heading_correction = false;

  while (true) {
    // [-100, 100] for controller stick axis values
    ch1 = controller_1.Axis1.position();
    ch2 = controller_1.Axis2.position();
    ch3 = controller_1.Axis3.position();
    ch4 = controller_1.Axis4.position();

    // true/false for controller button presses
    l1 = controller_1.ButtonL1.pressing();
    l2 = controller_1.ButtonL2.pressing();
    r1 = controller_1.ButtonR1.pressing();
    r2 = controller_1.ButtonR2.pressing();
    button_a = controller_1.ButtonA.pressing();
    button_b = controller_1.ButtonB.pressing();
    button_x = controller_1.ButtonX.pressing();
    button_y = controller_1.ButtonY.pressing();
    button_up_arrow = controller_1.ButtonUp.pressing();
    button_down_arrow = controller_1.ButtonDown.pressing();
    button_left_arrow = controller_1.ButtonLeft.pressing();
    button_right_arrow = controller_1.ButtonRight.pressing();

    // default tank drive or replace it with your preferred driver code here:
    driverArcade(ch3, ch1);
    // Arcade on the left stick:           driverArcade(ch3, ch4);
    // Split arcade (left fwd, right turn): driverArcade(ch3, ch1);
    // X-drive, mecanum or H-drive:         driverHolonomic(ch3, ch4, ch1);  or driverFieldCentric(ch3, ch4, ch1);
    // Tune feel with driver_curve and driver_deadband in custom/src/robot-config.cpp

    // Cascade lift motor control:
    // - R1: Fast lift upward at full power (12V)
    // - R2: Fast lift downward at 70% speed (8.4V instead of 12V)
    // - Up Arrow: Fine adjustment upward (4V)
    // - Down Arrow: Fine adjustment downward at 70% speed (2.1V instead of 3V)
    double cascade_position = cascade.position(deg);
    if (r1 && !r2 && cascade_position < cascade_max_height) {
      cascade.spin(reverse, 12, volt);
    } else if (r2 && !r1 && cascade_position > cascade_min_height) {
      cascade.spin(fwd, 8.4, volt); // 70% of original 12V speed
    } else if (button_up_arrow && !button_down_arrow && cascade_position < cascade_max_height) {
      cascade.spin(reverse, 4, volt);
    } else if (button_down_arrow && !button_up_arrow && cascade_position > cascade_min_height) {
      cascade.spin(fwd, 2.1, volt); // 70% of original 3V speed
    } else {
      cascade.stop(hold);
    }

    if (l1 && !l2) {
      toggle.spin(reverse, 8, volt);
    } else if (l2 && !l1) {
      toggle.spin(fwd, 8, volt);
    } else {
      toggle.stop(hold);
    }

    // Claw pneumatic control:
    // Controlled ONLY by Button A as a toggle (Button B removed).
    // Uses rising-edge detection (triggering only when the button is freshly pressed)
    // to prevent rapid cycling while the button is held.
    if (button_a && !last_button_a) {
      claw_state = !claw_state;        // Invert state: open <-> closed
      claw_piston.set(claw_state);     // Apply new state to 3-wire solenoid B
    }
    last_button_a = button_a;          // Update previous button state for edge detection

    // Toggle piston pneumatic control:
    // Controlled ONLY by Left Arrow as a toggle.
    // Uses rising-edge detection to prevent rapid cycling while held.
    if (button_left_arrow && !last_button_left_arrow) {
      toggle_piston_state = !toggle_piston_state;  // Invert state: extended <-> retracted
      toggle_piston.set(toggle_piston_state);       // Apply new state to 3-wire solenoid A
    }
    last_button_left_arrow = button_left_arrow;    // Update previous button state for edge detection

    wait(10, msec);
  }
}

void runPreAutonomous() {
    // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }

  // X-drive or mecanum: useHolonomicDrive(left_front, right_front, left_back, right_back);
  // H-drive: useHDrive(strafe_motor);   (see the docs: Guide > Drive Types)

  // odom tracking (picks the setup from using_horizontal_tracker / using_vertical_tracker)
  startOdometry();

  // Optional: MCL uses the distance sensors to keep odometry from drifting while driving.
  // Needs field-centered coordinates, so start each auton with setPose() at the real field position.
  // startMcl();
  // Initialize pneumatics to known initial states (closed/retracted)
  claw_piston.set(false);
  claw_state = false;
  toggle_piston.set(false);
  toggle_piston_state = false;
  // Autonomous routines for the selector, in the order they appear on the Brain screen
  //addAuton("Left 3 Pin", southAuton);
  addAuton("Side for South", southSideAuto); 
  //addAuton("Skills for South", southSkills); 
  //addAuton("Odom Test", exampleAuton2);
  //addAuton("Red Goal Rush", redGoalRush);
  //addAuton("Async Example", asyncExample);
  //addAuton("Path Example", pathExample);
  //addAuton("Profiled Example", profiledExample);
  //addAuton("Drive Calibration", calibrateDrive);  // For the simulator (see the docs: Simulator > Calibrating)
  startAutonSelector();
}