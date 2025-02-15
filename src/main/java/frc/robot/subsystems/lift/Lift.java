package frc.robot.subsystems.lift;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.LiftConstants;

public class Lift extends SubsystemBase {

  // Pid loop parameters
  SparkMaxConfig pidConfig = new SparkMaxConfig(); // Parameters to control the Neo motor pid loops.

  // Pid controllers
  SparkClosedLoopController liftMotorOne_pidController; // Pid controllers for both spark max motor.
  SparkClosedLoopController liftMotorTwo_pidController;

  // Motors
  private SparkMax m_liftMotorOne; // Motors to accuate the lift.
  private SparkMax m_liftMotorTwo;

  // Encoders
  private RelativeEncoder m_liftMotorOne_Encoder;

  // Motor configs
  private SparkMaxConfig liftMotorOneConfig; // Configs for the motors moving the lift.
  private SparkMaxConfig liftMotorTwoConfig;

  // Controllers
  CommandXboxController operatorController; // Operator controller.

  // Constants
  LiftConstants liftConstants = new LiftConstants(); // Constants for lift

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
  public Lift(CommandXboxController operatorController) {
    // Intialize motors
    m_liftMotorOne =
        new SparkMax(
            ClimbConstants.leftGrip_motorPort,
            MotorType.kBrushless); // Motors intialized for the lift.
    m_liftMotorTwo = new SparkMax(ClimbConstants.rightGrip_motorPort, MotorType.kBrushless);

    // Motor configs
    liftMotorOneConfig = new SparkMaxConfig(); // Configuartions for lift motors.
    liftMotorTwoConfig = new SparkMaxConfig();

    // Apply configurations and parameters to motor configs
    liftMotorOneConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);
    liftMotorTwoConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);

    // Set configs onto the motors.
    m_liftMotorOne.configure(
        liftMotorOneConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_liftMotorTwo.configure(
        liftMotorTwoConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // Setup pid controllers
    liftMotorOne_pidController = m_liftMotorOne.getClosedLoopController();
    liftMotorTwo_pidController = m_liftMotorTwo.getClosedLoopController();

    // Encoder for the lifts movements
    m_liftMotorOne_Encoder = m_liftMotorOne.getEncoder(); // Get the encoder for the 1st lift motor.
    m_liftMotorOne_Encoder.setPosition(0); // Reset the encoder.

    // Initalize the operator controller
    this.operatorController = operatorController;

    // Pid loop setup
    pidConfig
        .closedLoop
        .p(liftConstants.kP)
        .i(liftConstants.kI)
        .d(liftConstants.kD)
        .outputRange(liftConstants.kMinOutput, liftConstants.kMaxOutput);
  }

  // Controls the free movement of the lift using the sticks; If and only if we are in the correct
  // state "FREE_MOVEMENT".
  public void freeMovement(double rightY) {
    // If the current state is free movement, allow the lift to freely move.
    if (states == liftStates.FREE_MOVEMENT) {

      // If the lift is not at max height, then enable upward movement and apply it here.
      if (m_liftMotorOne_Encoder.getPosition() < liftConstants.maxLiftPosition) {
        // The lift is below maximum position. If the controller is trying to move the lift down, do
        // so.
        if (rightY > 0.0) {
          m_liftMotorOne.set(rightY);
          m_liftMotorTwo.set(rightY);
        }
      }

      // If the lift is not at its minimum position, then allow movement downward.
      if (m_liftMotorOne_Encoder.getPosition() > liftConstants.minLiftPosition) {
        // The lift is above minimum position. If the controller is requesting the lift to move
        // down, then do so.
        if (rightY < 0.0) {
          m_liftMotorOne.set(rightY);
          m_liftMotorTwo.set(rightY);
        }
      }

      // If the joystick is at zero or very close to it (borderline stick drift) stop the lift from
      // moving
      if (rightY < 0.05 && rightY > -0.05) {
        m_liftMotorOne.set(0);
        m_liftMotorTwo.set(0);
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
