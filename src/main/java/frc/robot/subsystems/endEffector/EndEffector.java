package frc.robot.subsystems.endEffector;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for the end effector with intake wheels and limit switches. This subsystem provides
 * direct control of the motors and access to sensor states.
 */
public class EndEffector extends SubsystemBase {
  // Motors
  private final SparkMax leftIntakeMotor;
  private final SparkMax rightIntakeMotor;

  // Limit switches
  private final DigitalInput intakeLimitSwitch;
  private final DigitalInput outtakeLimitSwitch;

  // Current state tracking
  private IntakeState currentState = IntakeState.STOPPED;

  /** Possible states of the intake motors */
  public enum IntakeState {
    INTAKE,
    OUTTAKE,
    STOPPED
  }

  /** Constructs the EndEffector subsystem and initializes hardware */
  public EndEffector() {
    // Initialize motors
    leftIntakeMotor =
        new SparkMax(EndEffectorConstants.endEffectorMotorOne_Port, MotorType.kBrushed);
    rightIntakeMotor =
        new SparkMax(EndEffectorConstants.endEffectorMotorTwo_Port, MotorType.kBrushed);

    // Configure motors
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake).inverted(false);
    leftIntakeMotor.configure(
        leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake).inverted(true);
    rightIntakeMotor.configure(
        rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize limit switches
    intakeLimitSwitch = new DigitalInput(EndEffectorConstants.intakeLimitSwitchPort);
    outtakeLimitSwitch = new DigitalInput(EndEffectorConstants.outtakeLimitSwitchPort);
  }

  /**
   * Runs intake wheels outward to eject game pieces at the specified speed factor
   *
   * @param speedFactor A multiplier for the base speed (0.0-1.0)
   */
  public void outtake(double speedFactor) {
    double speed = EndEffectorConstants.endEffectorSpeed * speedFactor;
    leftIntakeMotor.set(speed);
    rightIntakeMotor.set(speed);
    currentState = IntakeState.INTAKE;
  }

  /** Runs intake wheels outward to eject game pieces at the default speed */
  public void outtake() {
    outtake(1.0);
  }

  /** Runs intake wheels inward to collect game pieces */
  public void intake() {
    leftIntakeMotor.set(-EndEffectorConstants.endEffectorSpeed);
    rightIntakeMotor.set(-EndEffectorConstants.endEffectorSpeed);
    currentState = IntakeState.OUTTAKE;
  }

  public void revStop() {
    leftIntakeMotor.set(1);
    rightIntakeMotor.set(1);
    stop();
  }

  /** Stops the intake wheels */
  public void stop() {
    leftIntakeMotor.set(0);
    rightIntakeMotor.set(0);
    currentState = IntakeState.STOPPED;
  }

  /**
   * @return true if the intake limit switch is triggered (game piece detected at intake position)
   */
  public boolean isIntakeLimitSwitchTriggered() {
    return intakeLimitSwitch.get();
  }

  /**
   * @return true if the outtake limit switch is triggered (game piece detected at outtake position)
   */
  public boolean isOuttakeLimitSwitchTriggered() {
    return outtakeLimitSwitch.get();
  }

  /**
   * @return the current state of the intake motors
   */
  public IntakeState getCurrentState() {
    return currentState;
  }

  @Override
  public void periodic() {
    // Log sensor values and state
    Logger.recordOutput("EndEffector/IntakeLimitSwitch", isIntakeLimitSwitchTriggered());
    Logger.recordOutput("EndEffector/OuttakeLimitSwitch", isOuttakeLimitSwitchTriggered());
    Logger.recordOutput("EndEffector/State", currentState.toString());
    Logger.recordOutput("EndEffector/LeftMotor/Output", leftIntakeMotor.getAppliedOutput());
    Logger.recordOutput("EndEffector/RightMotor/Output", rightIntakeMotor.getAppliedOutput());
    Logger.recordOutput("EndEffector/LeftMotor/Current", leftIntakeMotor.getOutputCurrent());
    Logger.recordOutput("EndEffector/RightMotor/Current", rightIntakeMotor.getOutputCurrent());
  }
}
