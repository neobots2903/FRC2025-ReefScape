package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

/* NOTES */
/*
 * - When creating a TalonFX controller object, the canbus option will always be "rio" in our case.
 * - This functionality is tightly coupled with the climb subsystem, so I'd recommend combining them together.
 * - The naming is a bit confusing. This is in the intake folder, but the end-effector is the intake I think?
 * - We should write a method that converts units (inches?) to rotations to more easily set the position.
 */

/*
 * Class to manage the Ramp Mechanism for the intake.
 */
public class RampMechanism extends SubsystemBase {

  // Motors
  // Motors for pivot
  private SparkMax m_pivotMotorOne;
  private SparkMax m_pivotMotorTwo;

  // Configurations
  private SparkMaxConfig pivotMotorOneConfig;
  private SparkMaxConfig pivotMotorTwoConfig;

  // Pid loop controllers
  private SparkClosedLoopController pivotMotorOnePidController;
  private SparkClosedLoopController pivotMotorTwoPidController;

  // Encoders
  private RelativeEncoder pivotMotorOneEncoder;
  private RelativeEncoder pivotMotorTwoEncoder;

  // Constructor, init hardware and other software components here;
  // Run setups if needed.
  public RampMechanism() {

    // Setup motor configurations and variables (init motors)
    initMotorConfigurations();
  }

  // Sets up motor configurations for all motors.
  void initMotorConfigurations() {

    // Init motors for pivot
    m_pivotMotorOne = new SparkMax(ClimbConstants.pivotMotorOne_motorPort, MotorType.kBrushless);
    m_pivotMotorTwo = new SparkMax(ClimbConstants.pivotMotorTwo_motorPort, MotorType.kBrushless);

    // Init configs for pivot motors
    pivotMotorOneConfig = new SparkMaxConfig();
    pivotMotorTwoConfig = new SparkMaxConfig();
    pivotMotorOneConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);
    pivotMotorTwoConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);

    // Set converstion factor for position conversion because of gear ratio.
    pivotMotorOneConfig.encoder.positionConversionFactor(3.6);
    pivotMotorTwoConfig.encoder.positionConversionFactor(3.6);

    // Setup encoders
    pivotMotorOneEncoder = m_pivotMotorOne.getEncoder();
    pivotMotorTwoEncoder = m_pivotMotorTwo.getEncoder();

    // Get pid loop controllers from both motors.
    pivotMotorOnePidController = m_pivotMotorOne.getClosedLoopController();
    pivotMotorTwoPidController = m_pivotMotorTwo.getClosedLoopController();

    // Setup closed loop pid controllers.
    pivotMotorOneConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1);

    pivotMotorTwoConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1);

    // Add configs to pivot motors
    m_pivotMotorOne.configure(
        pivotMotorOneConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_pivotMotorTwo.configure(
        pivotMotorTwoConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // Rotates the ramp to a specific position in degrees.
  // Starting position when intializing the robot is 0 degrees.
  void rotateRamp(double degrees) {
    pivotMotorOnePidController.setReference(degrees, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    pivotMotorTwoPidController.setReference(-degrees, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
}
