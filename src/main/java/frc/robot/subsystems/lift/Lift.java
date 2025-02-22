package frc.robot.subsystems.lift;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.thethriftybot.ThriftyNova;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LiftConstants;

/* NOTES */
/*
 * - The lift is using the new Thrify Nova controllers. You need to install ThriftyLib to use them.
 * - We should look into the Nova's to see if they support Motion Profiling, because the lift is scary powerful.
 * - None of the lift states are implemented yet?
 * - The lift subsystem shouldn't take in the operator controller.
 * - The subsystem is just for defining mechanism control. Such as setting power, handling pid calulations, etc. The actual usage,
 *   such as when to move the lift, should be handled in a command in either the RobotContainer or in a command group.
 * - LiftConstants is a static class, so you can access the constants without instantiating it (without using new to create an object).
 */

public class Lift extends SubsystemBase {

  private ThriftyNova liftMotorOne; // Motors to accuate the lift.
  private ThriftyNova liftMotorTwo;

  // //Lift states.
  enum liftStates {
    FREE_MOVEMENT, // Lift is at the level where coral can be intaked via end effector.
    LOW_CORAL, // Lift is at the low coral level.
    MEDIUM_CORAL, // Lift is at the medium coral level.
    HIGH_CORAL, // Lift is at the high coral level.
    BASE_CORAL, // Lift is at the base coral level.
  }

  // States
  liftStates states = liftStates.FREE_MOVEMENT; // States for the lift.

  // Constructor to intialize motors and other electronics.
  public Lift() {

    // Motors
    liftMotorOne = new ThriftyNova(LiftConstants.liftMotorOneCanID); // Linear lift motors.
    liftMotorTwo = new ThriftyNova(LiftConstants.liftMotorTwoCanID);

  }

  // Controls the free movement of the lift using the sticks; If and only if we are in the correct
  // state "FREE_MOVEMENT".
  public void freeMovement(double rightY) {
    // If the current state is free movement, allow the lift to freely move.
    if (states == liftStates.FREE_MOVEMENT) {

      // If the lift is not at max height, then enable upward movement and apply it here.
      if (liftMotorOne.getPosition() < LiftConstants.maxLiftPosition) {
        // The lift is below maximum position. If the controller is trying to move the lift down, do
        // so.
        if (rightY > 0.0) {
          liftMotorOne.setPercent(rightY);
          liftMotorTwo.setPercent(rightY);
        }
      }

      // If the lift is not at its minimum position, then allow movement downward.
      if (liftMotorOne.getPosition() > LiftConstants.minLiftPosition) {
        // The lift is above minimum position. If the controller is requesting the lift to move
        // down, then do so.
        if (rightY < 0.0) {
          liftMotorOne.setPercent(rightY);
          liftMotorTwo.setPercent(rightY);
        }
      }

      // If the joystick is at zero or very close to it (borderline stick drift) stop the lift from
      // moving
      if (rightY < 0.05 && rightY > -0.05) {
        liftMotorOne.setPercent(0);
        liftMotorTwo.setPercent(0);
      }
    }
  }

  // Gets the lift to a specific position based on the first argument.
  // Positions: base coral (0), low coral (1), high coral (2), medium coral (3).
  public void travelToPosition(int pos) {}

  // Runs the lift to high coral using encoders.
  public void toHighCoral() {

    states = liftStates.HIGH_CORAL; // Current state is traveling to high coral.

    // Run the lift motors in a pid loop to high coral.
    // liftMotorOne_pidController.setReference(liftConstants.maxLiftPosition, ControlType.kPosition,
    // 0);
    // liftMotorTwo_pidController.setReference(liftConstants.maxLiftPosition, ControlType.kPosition,
    // 0);
  }
}
