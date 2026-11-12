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

// Manual autonomous trigger edge tracking
bool last_button_x = false;

// Auto-lift cascade after claw activates (grabs)
bool auto_lifting = false;
double auto_lift_start_pos = 0;
int auto_lift_delay_timer = 0; // Delay ticks (10ms each) to let claw seal before lifting
int auto_lift_timeout = 0;     // Safety timeout ticks (10ms each) to prevent runaway lift
const double auto_lift_degrees = 400;  // Amount to lift after grabbing (~1 inch off floor to clear tiles)
const double auto_lift_voltage = 5.0; // Controlled lift speed to prevent momentum overshoot

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
    // - Auto-lift: Automatically raises cascade ~75 deg (~1 inch) after claw grabs
    double cascade_position = cascade.position(deg);
    if (r1 && !r2 && cascade_position < cascade_max_height) {
      cascade.spin(reverse, 12, volt);
      auto_lifting = false; // Driver manual input cancels auto-lift
    } else if (r2 && !r1 && cascade_position > cascade_min_height) {
      cascade.spin(fwd, 8.4, volt); // 70% of original 12V speed
      auto_lifting = false; // Driver manual input cancels auto-lift
    } else if (button_up_arrow && !button_down_arrow && cascade_position < cascade_max_height) {
      cascade.spin(reverse, 4, volt);
      auto_lifting = false; // Driver manual input cancels auto-lift
    } else if (button_down_arrow && !button_up_arrow && cascade_position > cascade_min_height) {
      cascade.spin(fwd, 2.1, volt); // 70% of original 3V speed
      auto_lifting = false; // Driver manual input cancels auto-lift
    } else if (auto_lifting) {
      if (auto_lift_delay_timer > 0) {
        auto_lift_delay_timer--;       // Wait for claw pneumatics to seal tight before lifting
        auto_lift_start_pos = cascade.position(deg); // Keep updating start position right until motion begins
        cascade.stop(hold);
      } else if (std::fabs(cascade.position(deg) - auto_lift_start_pos) < auto_lift_degrees && auto_lift_timeout > 0) {
        auto_lift_timeout--;
        cascade.spin(reverse, auto_lift_voltage, volt); // Smooth, controlled lift upward
      } else {
        auto_lifting = false;          // Target height reached or safety timeout
        cascade.stop(hold);
      }
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

      // When claw activates/clamps onto a pin (!claw_state), automatically lift cascade slightly
      // so the pin clears the floor and doesn't drag while driving.
      if (!claw_state) {
        auto_lift_delay_timer = 15;    // 150ms delay for pneumatic claw to grip firmly
        auto_lift_start_pos = cascade.position(deg);
        auto_lift_timeout = 35;        // Max 350ms duration safety cap
        auto_lifting = true;
      }
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

    // Manual autonomous trigger:
    // Press Button X during Driver mode to run the currently selected autonomous routine
    // (such as Skills for South) without needing field control or a competition switch.
    if (button_x && !last_button_x) {
      stopChassis(coast);
      runSelectedAuton();
    }
    last_button_x = button_x;

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
  //addAuton("Side for South", southSideAuto); 
  addAuton("Skills for South", southSkills); 
  //addAuton("Odom Test", exampleAuton2);
  //addAuton("Red Goal Rush", redGoalRush);
  //addAuton("Async Example", asyncExample);
  //addAuton("Path Example", pathExample);
  //addAuton("Profiled Example", profiledExample);
  //addAuton("Drive Calibration", calibrateDrive);  // For the simulator (see the docs: Simulator > Calibrating)
  startAutonSelector();
}