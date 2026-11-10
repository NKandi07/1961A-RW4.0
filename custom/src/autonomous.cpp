#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>

#include "../include/autonomous.h"
#include "motor-control.h"
#include "path-following.h"

// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Then add each routine to the auton selector with addAuton() in runPreAutonomous() (custom/src/user.cpp)
// Format: returnType functionName() { code }

/*void exampleAuton() {
  // Use this for tuning linear and turn pid
  driveTo(60, 3000);
  turnToAngle(90, 2000);
  turnToAngle(135, 2000);
  turnToAngle(150, 2000);
  turnToAngle(160, 2000);
  turnToAngle(165, 2000);
  turnToAngle(0, 2000);
  driveTo(-60, 3000);
}

void exampleAuton2() {
  moveToPoint(24, 24, 1, 2000, false);
  moveToPoint(48, 48, 1, 2000, true);
  moveToPoint(24, 24, -1, 2000, true);
  moveToPoint(0, 0, 1, 2000, true);
  correct_angle = 0;
  driveTo(24, 2000, false, 8);
  turnToAngle(90, 800, false);
  turnToAngle(180, 800, true);
}*/
void southSideAuto(){
  toggle.setVelocity(100, pct);
  toggle.spinFor(reverse, 2, rev);
  cascade.setPosition(0, deg); // Call while physically down
  wait(500, msec); //Brings piston down
  driveTo(-19, 3000, true, 8);
  cascade.setVelocity(80, pct);
  cascade.spinToPosition(1000, deg);
  turnToAngle(270, 2000, true, 8);
  driveTo(-18.5, 2000, true, 8);
  wait(250, msec);
  driveTo(-6.35, 1000, false, 3);
  wait(250, msec);
  stopChassis(hold);
  driveTo(1.25, 1000, true, 4);
  wait(250, msec);
  cascade.spinToPosition(0, deg);
  wait(250, msec);
  claw_piston.set(true);
  wait(500, msec); // Wait for the claw to fully open
  driveTo(5.75, 2000, true, 8);
  cascade.spinToPosition(0, deg);
  /*
  turnToAngle(210, 2000, true, 8);
  driveTo(-18, 2000, true, 8);
  driveTo(-6.25, 2000, true, 2);
  claw_piston.set(false);
  wait(250, msec); // Wait for the claw to fully close
  cascade.spinToPosition(1600, deg);
  wait(250, msec);
  turnToAngle(-22.5, 2000, true, 4);
  driveTo(-12, 2000, true, 8);
  cascade.spinToPosition(400, deg);
  claw_piston.set(true);
  wait(1500, msec); // Wait for the claw to fully open
  cascade.spinToPosition(800, deg);*/
}

void southSkills(){
  toggle.setVelocity(100, pct);
  toggle.spinFor(reverse, 2, rev);
  cascade.setPosition(0, deg); // Call while physically down
  wait(500, msec); //Brings piston down
  driveTo(-19, 3000, true, 8);
  cascade.setVelocity(80, pct);
  cascade.spinToPosition(1000, deg);
  turnToAngle(90, 2000, true, 8);
  driveTo(-18.5, 2000, true, 8);
  wait(250, msec);
  driveTo(-6.35, 1000, false, 3);
  wait(250, msec);
  stopChassis(hold);
  driveTo(1.45, 1000, true, 4);
  wait(250, msec);
  cascade.spinToPosition(0, deg);
  wait(250, msec);
  claw_piston.set(true);
  wait(500, msec); // Wait for the claw to fully open
  /*driveTo(5.75, 2000, true, 8);
  cascade.spinToPosition(0, deg);
  turnToAngle(30, 2000, true, 8);
  driveTo(-18, 2000, true, 8);
  driveTo(-6.25, 2000, true, 2);
  claw_piston.set(false);
  wait(250, msec); // Wait for the claw to fully close
  cascade.spinToPosition(1600, deg);
  wait(250, msec);
  turnToAngle(167.5, 2000, true, 4);
  driveTo(-12, 2000, true, 8);
  cascade.spinToPosition(400, deg);
  claw_piston.set(true);
  wait(1500, msec); // Wait for the claw to fully open
  cascade.spinToPosition(800, deg);
  turnToAngle(150, 2000, false, 5);
  driveTo(30, 3000, true, 6);*/
}

void southAuton(){
  toggle.setVelocity(100, pct);
  toggle.spinFor(reverse, 2, rev);
  cascade.setPosition(0, deg); // Call while physically down
  wait(500, msec); //Brings piston down
  driveTo(-19, 3000, true, 8);
  cascade.setVelocity(80, pct);
  cascade.spinToPosition(1000, deg);
  turnToAngle(90, 2000, true, 8);
  driveTo(-18.5, 2000, true, 8);
  wait(250, msec);
  driveTo(-6.35, 1000, false, 3);
  wait(250, msec);
  stopChassis(hold);
  driveTo(1.2, 1000, true, 4);
  wait(250, msec);
  cascade.spinToPosition(0, deg);
  wait(250, msec);
  claw_piston.set(true);
  wait(500, msec); // Wait for the claw to fully open
  driveTo(5.75, 2000, true, 8);
  cascade.spinToPosition(0, deg);
  turnToAngle(30, 2000, true, 8);
  driveTo(-18, 2000, true, 8);
  driveTo(-6.25, 2000, true, 2);
  claw_piston.set(false);
  wait(250, msec); // Wait for the claw to fully close
  cascade.spinToPosition(1600, deg);
  wait(250, msec);
  turnToAngle(167.5, 2000, true, 4);
  driveTo(-12, 2000, true, 8);
  cascade.spinToPosition(400, deg);
  claw_piston.set(true);
  wait(500, msec); 
  // Wait for the claw to fully open
}

/*
 * asyncExample
 * Shows background motions: start a move, run mechanisms while it drives, and stop it early with a sensor.
 
void asyncExample() {
  setPose(0, 0, 0);

  // Start driving in the background, then start the intake 12 inches into the move
  runAsync([]{ moveToPoint(0, 48, 1, 3000); });
  waitUntilTraveled(12);
  intake_motor.spin(fwd, 12, volt);

  // Stop the move early once the intake sensor sees an object
  waitUntil(!isMotionRunning() || intake_distance.objectDistance(mm) < 50);
  cancelMotion();
  waitForMotion();
  intake_motor.stop(hold);

  // Turn around and drive back, raising the arm once the turn is halfway done
  runAsync([]{
    turnToAngle(180, 1500);
    moveToPoint(0, 0, 1, 3000);
  });
  waitUntilTurned(90);
  cascade.spin(fwd, 6, volt);
  waitForMotion();
  cascade.stop(hold);
}
*/  
/*
 * pathExample
 * Drives a smooth S-curve with pure pursuit, then backs along it with RAMSETE.
 * Paths use field coordinates, so the auton starts with setPose().
 
void pathExample() {
  setPose(-36, -60, 0);
  Path out = pathFromPoints({{-36, -60}, {-36, -32}, {-12, -22}, {12, -30}, {34, -24}});
  followPath(out, 5000);
  Path back = pathFromPoints({{34, -24}, {12, -30}, {-12, -22}, {-36, -32}, {-36, -52}});
  followPathRamsete(back, 5000, true);
}*/

/*
 * profiledExample
 * Straight drives and turns with motion profiles: they speed up and slow down smoothly.
 
void profiledExample() {
  driveProfiled(36, 3000);
  turnProfiled(90, 2000);
  driveProfiled(24, 3000);
  turnProfiled(0, 2000);
  driveProfiled(-36, 3000);
}*/

double cascade_pid_target = 0, cascade_load_target = 60, cascade_store_target = 250, cascade_score_target = 470;

/*
 * cascadePID
 * Runs a single PID update for the cascade motor to reach the specified target position.
 * - cascade_target: Desired cascade position (degrees).
 */
void cascadePID(double cascade_target) {
  PID pidcascade = PID(cascade_kp, cascade_ki, cascade_kd); // Initialize PID controller for cascade
  pidcascade.setTarget(cascade_target);   // Set target position
  pidcascade.setIntegralMax(0);
  pidcascade.setIntegralRange(1);
  pidcascade.setSmallBigErrorTolerance(1, 1);
  pidcascade.setSmallBigErrorDuration(0, 0);
  pidcascade.setDerivativeTolerance(100);
  pidcascade.setArrive(true);
  cascade.spin(fwd, pidcascade.update(cascade.position(deg)), volt); // Apply PID output to cascade motor
}

/*
 * cascadePIDLoop
 * Continuously runs the cascade PID control in a separate thread, keeping the cascade at the target position.
 */
void cascadePIDLoop() {
  while(true) {
    cascadePID(cascade_pid_target); // Continuously update cascade position
    wait(10, msec);
  }
}

/*
 * rushClamp
 * Waits until the clamp distance sensor detects an object within 85mm, then closes the claw and lowers the rush arm.
 * Used for quickly grabbing a mobile goal at the start of autonomous.
 
void rushClamp() {
  while(clamp_distance.objectDistance(mm) > 85) { // Wait for object to be close enough
    wait(10, msec);
  }
  claw.set(true);        // Close the claw to grab the goal
  rush_arm.set(false);   // Lower the rush arm
}*/

/*
 * intakeThread
 * Runs the intake until an object is detected by the optical or distance sensor, then stops the intake.
 * Used for picking up rings or other objects during autonomous.
 
void intakeThread(){
  optical_sensor.setLight(ledState::on);      // Turn on optical sensor light
  optical_sensor.setLightPower(100);          // Set light power to max
  while(!optical_sensor.isNearObject() && intake_distance.objectDistance(mm) > 50){
    wait(10, msec);                           // Wait until object is detected
  }
  intake_motor.stop(hold);                    // Stop intake motor and hold
}*/

/*
 * redGoalRush
 * 2024-2025 World Championship runner-up(1698V) autonomous routine.
 * This routine executes a complex sequence to rush, grab, and score mobile goals and rings.
 * It uses multiple threads for simultaneous arm, clamp, and intake control.
 
void redGoalRush() {
  cascade.setPosition(cascade_load_target, deg);       // Set cascade to load position
  correct_angle = getInertialHeading();                // Sync correct_angle with inertial sensor
  cascade_pid_target = cascade_store_target;           // Set cascade PID target to store position

  thread al = thread(cascadePIDLoop);                  // Start cascade PID loop in a thread
  thread rc = thread(rushClamp);                       // Start clamp routine in a thread
  intake_motor.spin(fwd, 12, volt);                    // Start intake motor at full speed
  thread it = thread(intakeThread);                    // Start intake sensor thread
  rush_arm.set(true);                                  // Raise rush arm

  driveTo(33, 1100, true);                             // Drive forward to first goal
  moveToPoint(-2, 10, -1, 15000, false);               // Pull the goal back
  stopChassis(hold);                                   // Stop chassis and hold position

  rc.interrupt();                                      // Stop clamp thread (goal should be clamped)
  rush_arm.set(true);                                  // Raise rush arm again (ensure up)
  claw.set(false);                                     // Open claw to release goal
  wait(100, msec);                                     // Brief pause

  correct_angle = normalizeTarget(-20);                // Adjust target heading for next maneuver
  driveTo(3, 800, true, 8);                            // Drive forward slightly
  driveTo(-5, 1000, true);                             // Back up

  rush_arm.set(false);                                 // Lower rush arm
  wait(200, msec);                                     // Wait for arm to lower

  turnToAngle(-90, 800, false);                        // Turn to face the goal backwards
  moveToPoint(0, 26, -1, 2000, false, 6);              // Move backwards into the goal
  driveChassis(-1.5, -1.5);                            // Slowly drive backward for alignment
  mogo_mech.set(true);                                 // Clamp mobile goal
  wait(100, msec);                                     // Wait for clamp

  it.interrupt();                                      // Stop intake thread (ring should be collected)
  intake_motor.spin(fwd, 12, volt);                    // Restart intake

  moveToPoint(1, 7, 1, 2000, true);                    // Move near corner to drop goal
  turnToAngle(-90, 350, true);                         // Turn to drop goal
  mogo_mech.set(false);                                // Release mobile goal clamp
  driveChassis(-4, 4);                                 // Turn a bit to align with next target
  wait(300, msec);                                     // Wait for spin

  intake_motor.spin(fwd, -12, volt);                   // Reverse intake to push disc in front away
  moveToPoint(-13, -4, 1, 1500, false, 10);            // Move forward to push disc out of the way
  turnToAngle(180, 800, false);                        // Turn to clamp goal
  intake_motor.spin(fwd, 0, volt);                     // Stop intake

  moveToPoint(-31, 26, -1, 2000, false, 6);            // Move backwards into the next goal
  driveChassis(-1.5, -1.5);                            // Slowly drive backward for alignment
  mogo_mech.set(true);                                 // Clamp mobile goal
  wait(100, msec);                                     // Wait for clamp

  turnToAngle(145, 300, true);                         // Turn to face corner
  moveToPoint(-4, -3, 1, 2000, false);                 // Move to corner
  intake_motor.spin(fwd, 12, volt);                    // Start intake

  correct_angle = normalizeTarget(135);                // Update heading for next maneuver
  driveTo(1000, 1500, false, 4);                       // Drive forward infinitely until timeout
  driveTo(-13, 2000, true, 6);                         // Back up
  driveTo(10, 2500, true, 3);                          // Drive forward to intake second corner ring

  wait(200, msec);                                     // Brief wait for intake

  moveToPoint(-11, 6, -1, 2000, false, 10);            // Move backward out of the corner
  turnToAngle(45, 400, true);                          // Turn to align for wallstake

  al.interrupt();                                      // Stop cascade PID thread
  cascade_pid_target = cascade_score_target - 100;     // Set cascade to scoring position
  thread al2 = thread(cascadePIDLoop);                 // Start cascade PID thread

  moveToPoint(12, 34, 1, 1700, true, 8);               // Move forward to final wallstake scoring position

  al2.interrupt();                                     // Stop cascade PID thread
  cascade.spin(fwd, 1, volt);                          // Spin cascade forward slightly

  turnToAngle(40, 200);                                // Final turn for alignment
  driveChassis(1, 1);                                  // Slow drive forward
}*/