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
  setPose(0, 0, 0);
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
  driveTo(-6, 1000, false, 3);
  wait(250, msec);
  stopChassis(hold);
  wait(250, msec);
  driveTo(1.5, 1000, true, 3);
  wait(250, msec);
  cascade.spinToPosition(525, deg);
  wait(250, msec);
  claw_piston.set(true); // Wait for the claw to fully open, scoring 1st pin (preload)
  wait(500, msec); 
  // Start driving forward 5.75" in the background
  runAsync([]{ driveTo(6.75, 2000, true, 8); });

  // Wait until the robot has moved forward 2 inches to safely clear the goal
  waitUntilTraveled(2);

  // Lower cascade down to 0 deg non-blocking while the robot continues driving
  cascade.spinToPosition(0, deg, false);

  // Wait for the forward drive motion to finish before starting the turn
  waitForMotion();
  turnToAngle(32.5, 2000, true, 8);
  driveTo(-18, 2000, true, 8);
  driveTo(-7, 1500, true, 2);
  claw_piston.set(false);
  wait(250, msec); // Wait for the claw to fully close
  cascade.spinToPosition(350, deg);         // Lift slightly to clear the floor before turning
  cascade.spinToPosition(1600, deg, false); // Finish lifting to 1600 deg async while turning & driving
  turnToAngle(167.5, 2000, true, 4);
  driveTo(-12, 2000, true, 6);
  cascade.setVelocity(75, pct);
  cascade.spinToPosition(900, deg);
  cascade.setVelocity(100, pct);
  wait(200, msec);
  claw_piston.set(true); // Wait for the claw to fully open, scoring 1st field pin-stack
  wait(500, msec); 
  turnToAngle(225, 2000, true, 5);
  driveTo(15, 3000, true, 8);
  turnToAngle(145, 2000, true, 5);
  cascade.spinToPosition(0, deg);
  driveTo(-17, 3000, true, 8);
  driveTo(-4, 2000, true, 2);
  wait(250, msec);
  claw_piston.set(false); //grabs pin
  wait(250, msec);
  cascade.spinToPosition(2000, deg);
  wait(250, msec); // Wait for the cascade to fully extend
  driveTo(-12, 2000, true, 5);
  wait(200, msec);
  turnToAngle(270, 2000, true, 4);
  driveTo(-19, 3000, true, 4);
  cascade.spinToPosition(1700, deg);
  wait(250, msec);
  claw_piston.set(true); // Wait for the claw to fully open, scoring 2nd field pin-stack
  wait(500, msec); 
  driveTo(34, 3000, true, 8);
  cascade.spinToPosition(0, deg);
  turnToAngle(180, 2000, true, 4);
  driveTo(-14.5, 3000, true, 6);
  stopChassis(hold);                // 1. Lock the chassis wheels firmly
  wait(200, msec);                 // 2. Settle robot before grabbing
  claw_piston.set(false);          // 3. Grab the pin
  wait(400, msec);                 // 4. Give claw pneumatics time to close tight
  cascade.setVelocity(50, pct);
  cascade.spinToPosition(400, deg);// 5. Lift cascade up to clear the floor
  cascade.setVelocity(50, pct);
  wait(400, msec);                 // 6. Settle lift before driving away
  driveTo(8, 3000, true, 4);
  cascade.spinToPosition(800, deg);
  driveTo(6, 3000, true, 6);
  turnToAngle(270, 2000, true, 6);
  cascade.spinToPosition(2500, deg);
  wait(250, msec); // Wait for the cascade to fully extend
  driveTo(-32, 3000, true, 4);
  stopChassis(hold); 
  wait(250, msec);
  cascade.spinToPosition(2100, deg);
  wait(250, msec);
  claw_piston.set(true); // Wait for the claw to fully open, scoring 3rd field pin-stack (matchloaded)
  wait(500, msec);
  driveTo(-3, 3000, true, 3);
  southSkillsPart2();
}

/*
 * southSkillsPart2
 * Runs the second half of southSkills starting from the right side of the 1st Alliance Goal
 * (facing 270 deg). Can also be run standalone from the Auton Selector ("Skills Part 2")
 * by placing the robot backed up against the right side of the 1st Alliance Goal facing 270 deg.
 */
void southSkillsPart2() {
  // If started standalone from the selector at (0, 0, 0), initialize heading to 270 deg
  if (hypot(x_pos, y_pos) < 2.0 && fabs(getInertialHeading() - 270.0) > 45.0) {
    setPose(0, 0, 270);
  }

  // Anchor odometry coordinates to the known position at the right side of the 1st Alliance Goal.
  // Note: In Part 1, driveTo(34) and driveTo(-14.5) used unscaled drive encoders (~0.68 real in per unit),
  // so 34 driveTo units ≈ 23.0 real odom inches, and 14.5 driveTo units ≈ 9.8 real odom inches.
  static double goal1_x, goal1_y, loader_x, loader_grab_y, loader_clear_y;
  static double goal2_x, goal2_prep_y, goal2_score_y, toggle_x, toggle_y;
  goal1_x = x_pos;
  goal1_y = y_pos;
  loader_x = goal1_x - 26.6;       // 26.6" forward (along 270 deg) from Goal 1 aligner stop to Matchloader (58.7" - 32.1")
  loader_grab_y = goal1_y + 9.0;   // 9.0" back (at 180 deg) onto the Matchloader (stops before riding up)
  loader_clear_y = goal1_y + 2.6;  // Pull ~6.4" forward out of Matchloader before 45-deg diagonal
  goal2_x = loader_x + 11.6;       // 11.6" inward (+X) from Matchloader (= goal1_x - 15.0", centered on Goal 2)
  goal2_prep_y = goal1_y - 9.0;    // End of 45-deg diagonal (+11.6" X, -11.6" Y from loader_clear_y), 6" in front of Goal 2
  goal2_score_y = goal1_y - 15.0;  // 15.0" backward (-Y) from Goal 1 (23.54" goal spacing minus ~8.5" half-bot + aligner)
  toggle_x = goal1_x - 31.8;       // South-wall Toggle X (front of 12.625" bot against South wall at 270 deg)
  toggle_y = goal1_y - 46.7;       // South-wall Toggle Y (2 full tiles = 46.7" from Goal 1 to center of South wall)

  // ============================================================================
  // 1. First Matchload Trip: Grab 1 single pin & score on 2nd Alliance Goal
  // ============================================================================

  // Drive forward (along 270 deg / -X) to align with the Matchloader while dropping cascade
  runAsync([]{ moveToPoint(loader_x, goal1_y, 1, 2500, true, 10); });
  wait(100, msec); // Clear goal post before dropping cascade
  cascade.setVelocity(100, pct);
  cascade.spinToPosition(0, deg, false);
  waitForMotion();

  // Turn so back (claw) faces the Matchloader (+Y), approach at 5V, then creep very slowly at 2V
  turnToAngle(180, 1200, true, 8);
  moveToPoint(loader_x, loader_grab_y - 3.0, -1, 1400, false, 5);
  moveToPoint(loader_x, loader_grab_y, -1, 1300, true, 2);
  stopChassis(hold);
  wait(150, msec);
  claw_piston.set(false);             // Clamp onto 1 single pin
  wait(250, msec);
  cascade.setVelocity(80, pct);
  cascade.spinToPosition(400, deg, false); // Lift cascade to clear the matchloader
  wait(250, msec);

  // Drive forward out of the matchloader (-Y), raising cascade for 1st pin score
  moveToPoint(loader_x, loader_clear_y, 1, 1800, true, 8);
  cascade.setVelocity(100, pct);
  cascade.spinToPosition(900, deg, false);

  // Turn and drive at a 45-degree angle (135 deg: +X inward, -Y toward 2nd Alliance Goal)
  turnToAngle(135, 1000, true, 8);
  moveToPoint(goal2_x, goal2_prep_y, 1, 1800, true, 8);

  // Turn backward (0 deg so back/claw faces 180 deg toward 2nd Alliance Goal),
  // drive 2.5" further back into Goal 2 to physically square up/reset against the goal (1100ms timeout cap),
  // then nudge 0.4" (quarter-to-half inch) forward before dropping the single pin
  turnToAngle(0, 1000, true, 7);
  moveToPoint(goal2_x, goal2_score_y - 2.5, -1, 1100, true, 5);
  stopChassis(hold);
  wait(150, msec);
  moveToPoint(goal2_x, y_pos + 0.4, 1, 700, true, 4); // 0.4" forward nudge off the goal
  setPose(goal2_x, goal2_score_y + 0.4, getInertialHeading()); // Reset odom Y on Goal 2 contact
  cascade.spinToPosition(525, deg, false);
  wait(250, msec);
  claw_piston.set(true);              // Release 1st pin onto 2nd Alliance Goal
  wait(250, msec);

  // ============================================================================
  // 2. Second Matchload Trip: Grab another pin stack & stack on 2nd Alliance Goal
  // ============================================================================

  // Drive forward away from 2nd Alliance Goal, drop cascade, and return to Matchloader
  runAsync([]{ moveToPoint(loader_x, goal1_y, 1, 2200, true, 10); });
  wait(200, msec); // Clear 2nd goal before dropping cascade
  cascade.setVelocity(100, pct);
  cascade.spinToPosition(0, deg, false);
  waitForMotion();

  // Turn and back onto the Matchloader: approach at 5V, then creep very slowly at 2V
  turnToAngle(180, 1000, true, 8);
  moveToPoint(loader_x, loader_grab_y - 3.0, -1, 1400, false, 5);
  moveToPoint(loader_x, loader_grab_y, -1, 1300, true, 2);
  stopChassis(hold);
  wait(150, msec);
  claw_piston.set(false);             // Grab 2nd matchload pin stack
  wait(250, msec);
  cascade.setVelocity(80, pct);
  cascade.spinToPosition(400, deg, false); // Clear matchloader
  wait(250, msec);

  // Drive forward out of Matchloader and raise cascade to stack height
  moveToPoint(loader_x, loader_clear_y, 1, 1500, true, 8);
  cascade.setVelocity(100, pct);
  cascade.spinToPosition(2000, deg, false);

  // Drive at a 45-degree angle (135 deg) back to line up with 2nd Alliance Goal
  turnToAngle(135, 900, true, 8);
  moveToPoint(goal2_x, goal2_prep_y, 1, 1500, true, 8);

  // Turn backward (0 deg), back into 2nd Alliance Goal (1100ms timeout cap), nudge 0.5" forward, and score stack
  turnToAngle(0, 1000, true, 7);
  moveToPoint(goal2_x, goal2_score_y - 1.5, -1, 1100, true, 5);
  stopChassis(hold);
  wait(150, msec);
  moveToPoint(goal2_x, y_pos + 0.5, 1, 700, true, 4);
  cascade.spinToPosition(1700, deg, false);
  wait(250, msec);
  claw_piston.set(true);              // Score 2nd pin stack onto 2nd Alliance Goal
  wait(250, msec);

  // ============================================================================
  // 3. Spin South-Wall Toggle Twice (to Red) & Park in Midfield
  // ============================================================================

  // Pull straight forward (0 deg) clear of Goal 2, then move out to the South-wall lane
  // (loader_x - 4.0) before driving down to the Toggle so Goal 2 is never in the path
  moveToPoint(goal2_x, goal2_prep_y + 2.0, 1, 1000, true, 8);
  cascade.setVelocity(100, pct);
  cascade.spinToPosition(0, deg, false);
  turnToAngle(270, 900, true, 8);
  moveToPoint(loader_x - 4.0, y_pos, 1, 1400, true, 8);        // Step out clear of Goal 2
  turnToAngle(180, 900, true, 8);
  moveToPoint(loader_x - 4.0, toggle_y, 1, 2000, true, 10);    // Drive straight down South-wall lane to Toggle Y

  // Face the South-Wall Toggle directly (270 deg: front facing Toggle, claw facing away toward Midfield),
  // then drive forward into the wall so the front bumper physically squares up against the wall
  turnToAngle(270, 900, true, 8);
  driveTo(20, 1100, false, 6);        // Ram forward until flush against the South wall (1100ms timeout cap)
  driveChassis(2, 2);                 // Keep gentle forward pressure against the Toggle while spinning
  toggle.setVelocity(100, pct);
  toggle.spinFor(reverse, 2, rev);    // 1st spin
  toggle.spinFor(reverse, 2, rev);    // 2nd spin (makes it Red)
  stopChassis(hold);
  wait(100, msec);

  // Move far back into Midfield (+X in robot frame = toward field center (0, 0))
  moveToPoint(toggle_x + 56.0, toggle_y, -1, 2500, true, 12);
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
  driveTo(-6, 1000, false, 3);
  wait(250, msec);
  stopChassis(hold);
  wait(250, msec);
  driveTo(1.5, 1000, true, 3);
  wait(250, msec);
  cascade.spinToPosition(525, deg);
  wait(250, msec);
  claw_piston.set(true); // Wait for the claw to fully open, scoring 1st pin (preload)
  wait(500, msec); 
  // Start driving forward 5.75" in the background
  runAsync([]{ driveTo(6.75, 2000, true, 8); });

  // Wait until the robot has moved forward 2 inches to safely clear the goal
  waitUntilTraveled(2);

  // Lower cascade down to 0 deg non-blocking while the robot continues driving
  cascade.spinToPosition(0, deg, false);

  // Wait for the forward drive motion to finish before starting the turn
  waitForMotion();
  turnToAngle(32.5, 2000, true, 8);
  driveTo(-18, 2000, true, 8);
  driveTo(-7, 1500, true, 2);
  claw_piston.set(false);
  wait(250, msec); // Wait for the claw to fully close
  cascade.spinToPosition(350, deg);         // Lift slightly to clear the floor before turning
  cascade.spinToPosition(1600, deg, false); // Finish lifting to 1600 deg async while turning & driving
  turnToAngle(167.5, 2000, true, 4);
  driveTo(-12, 2000, true, 6);
  cascade.setVelocity(75, pct);
  cascade.spinToPosition(900, deg);
  cascade.setVelocity(100, pct);
  wait(200, msec);
  claw_piston.set(true); // Wait for the claw to fully open, scoring 1st field pin-stack
  wait(500, msec); 
  turnToAngle(225, 2000, true, 5);
  driveTo(15, 3000, true, 8);
  turnToAngle(145, 2000, true, 5);
  cascade.spinToPosition(0, deg);
  driveTo(-17, 3000, true, 8);
  driveTo(-4, 2000, true, 2);
  wait(250, msec);
  claw_piston.set(false); //grabs pin
  wait(250, msec);
  cascade.spinToPosition(2000, deg);
  wait(250, msec); // Wait for the cascade to fully extend
  driveTo(-12, 2000, true, 5);
  wait(200, msec);
  turnToAngle(270, 2000, true, 4);
  driveTo(-19, 3000, true, 4);
  cascade.spinToPosition(1700, deg);
  wait(250, msec);
  claw_piston.set(true); 
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

/*
 * calibrateVerticalOffset
 * Spins the robot slowly in place for 5 full rotations to measure the effective
 * tracking wheel offsets from center (and shows current odom drift).
 */
void calibrateVerticalOffset() {
  setPose(0, 0, 0);
  double start_imu_deg = inertial_sensor.rotation(degrees);
  double start_vert_deg = vertical_tracker.position(degrees);
  double start_horiz_deg = horizontal_tracker.position(degrees);

  const double target_deg = 5.0 * 360.0; // 5 full rotations
  const int timeout_msec = 20000;
  int elapsed = 0;

  // Spin slowly in place clockwise so wheels don't slip
  driveChassis(3.5, -3.5);
  while (elapsed < timeout_msec) {
    double d_deg = (inertial_sensor.rotation(degrees) - start_imu_deg) * imu_scale;
    if (fabs(d_deg) >= target_deg) {
      break;
    }

    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 30, "Calibrating Odom... %.2f / 5.00 turns", d_deg / 360.0);
    Brain.Screen.printAt(10, 60, "Odom X: %.2f in   Y: %.2f in", x_pos, y_pos);
    if (fabs(d_deg) > 180.0) {
      double d_rad = d_deg * M_PI / 180.0;
      double vert_in = (vertical_tracker.position(degrees) - start_vert_deg) * vertical_tracker_diameter * M_PI / 360.0;
      Brain.Screen.printAt(10, 90, "Live vertical offset: %.4f in", -vert_in / d_rad);
    }
    wait(50, msec);
    elapsed += 50;
  }

  stopChassis(brake);
  wait(400, msec); // Let robot settle before final reading

  double d_deg = (inertial_sensor.rotation(degrees) - start_imu_deg) * imu_scale;
  double d_rad = d_deg * M_PI / 180.0;
  double vert_in = (vertical_tracker.position(degrees) - start_vert_deg) * vertical_tracker_diameter * M_PI / 360.0;
  double horiz_in = (horizontal_tracker.position(degrees) - start_horiz_deg) * horizontal_tracker_diameter * M_PI / 360.0;

  double vert_offset = (fabs(d_rad) > 1e-3) ? (-vert_in / d_rad) : 0.0;
  double horiz_offset = (fabs(d_rad) > 1e-3) ? (-horiz_in / d_rad) : 0.0;

  Brain.Screen.clearScreen();
  Brain.Screen.printAt(10, 30, "DONE (%.2f turns)", d_deg / 360.0);
  Brain.Screen.printAt(10, 60, "vertical_tracker_dist_from_center = %.4f", vert_offset);
  if (using_horizontal_tracker) {
    Brain.Screen.printAt(10, 90, "horizontal_tracker_dist_from_center = %.4f", horiz_offset);
  }
  Brain.Screen.printAt(10, 120, "Final Odom X: %.2f in   Y: %.2f in", x_pos, y_pos);

  printf("Calibrate Odom DONE (%.2f turns)\n", d_deg / 360.0);
  printf("vertical_tracker_dist_from_center = %.4f\n", vert_offset);
  if (using_horizontal_tracker) {
    printf("horizontal_tracker_dist_from_center = %.4f\n", horiz_offset);
  }
  printf("Final Odom X: %.2f in, Y: %.2f in\n", x_pos, y_pos);
}

/*
 * testOdomMove
 * Out-and-back box test that works on any floor without a field:
 * 1. Mark the robot's starting corner with a piece of tape at (0, 0, 0).
 * 2. Drives to (18, 24), turns to 0 deg, and pauses 2s so you can check (18, 24).
 * 3. Drives to (0, 24), backs returns to (0, 0), and turns to 0 deg.
 * Measure how close the corner lands back on your starting tape mark!
 */
void testOdomMove() {
  setPose(0, 0, 0);
  moveToPoint(18, 24, 1, 3500, true, 8);
  turnToAngle(0, 1500);
  wait(2000, msec); // Pause 2s at (18, 24, 0 deg)

  // Return to (0, 0, 0) via (0, 24) to test full-loop odometry drift
  moveToPoint(0, 24, -1, 2500, true, 8);
  moveToPoint(0, 0, -1, 3000, true, 8);
  turnToAngle(0, 1500);
  wait(300, msec);

  Brain.Screen.clearScreen();
  Brain.Screen.printAt(10, 30, "Return-to-Zero Test Done!");
  Brain.Screen.printAt(10, 60, "Odom X: %.2f in   Y: %.2f in", x_pos, y_pos);
  Brain.Screen.printAt(10, 90, "Heading: %.2f deg", getInertialHeading());
  Brain.Screen.printAt(10, 120, "Check distance from start tape mark!");

  printf("Test Odom Move -> Final Odom: (%.2f, %.2f), Heading: %.2f deg\n",
         x_pos, y_pos, getInertialHeading());
}