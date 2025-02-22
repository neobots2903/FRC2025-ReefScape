package frc.robot.subsystems.lift;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.thethriftybot.Conversion;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.Conversion.PositionUnit;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.EncoderType;
import com.thethriftybot.ThriftyNova.PIDSlot;

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

  // How many inches the elavator tranvels
  // per radian that the motor turns
  public static double inchesPerRad = 2.5; 
  // where theta is rotation in radians and x is travel in inches:
  // - theta(x) = x / inchesPerRad
  // - x(theta) = theta * inchesPerRad

  //Motors
  private ThriftyNova liftMotorOne; // Motors to accuate the lift.
  private ThriftyNova liftMotorTwo;

  //Converter for ThriftyNova motors
  private Conversion converter;


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

    converter = new Conversion(PositionUnit.RADIANS, EncoderType.INTERNAL);

    // Motors
    liftMotorOne = configMotor(LiftConstants.liftMotorOneCanID, false); //Linear lift motors.
    liftMotorTwo = configMotor(LiftConstants.liftMotorTwoCanID, true);

  }

  private ThriftyNova configMotor(int canId, boolean inverted) {
    ThriftyNova motor = new ThriftyNova(canId);

    motor.setBrakeMode(true);
    motor.setInverted(inverted);
    motor.setRampUp(0.25);
    motor.setRampDown(0.05);
    motor.setMaxOutput(1, 0.25);
    motor.setSoftLimits(0, 7 * Math.PI);
    motor.enableSoftLimits(true);
    motor.setMaxCurrent(CurrentType.SUPPLY, 50);
    motor.useEncoderType(EncoderType.INTERNAL);
    motor.usePIDSlot(PIDSlot.SLOT0);
    motor.pid0.setP(0.5);
    motor.pid0.setI(0);
    motor.pid0.setD(0);
    motor.pid0.setFF(1.2);

    return motor;
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
          liftMotorTwo.setPercent(-rightY);
        }
      }

      // If the lift is not at its minimum position, then allow movement downward.
      if (liftMotorOne.getPosition() > LiftConstants.minLiftPosition) {
        // The lift is above minimum position. If the controller is requesting the lift to move
        // down, then do so.
        if (rightY < 0.0) {
          liftMotorOne.setPercent(rightY);
          liftMotorTwo.setPercent(-rightY);
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


  //Run the lift to height.
  public void runLiftToPos(int pos) {
    double motorPosition = converter.toMotor(pos / inchesPerRad);
    liftMotorOne.setPosition(pos);
    liftMotorTwo.setPosition(pos);
  }

}
