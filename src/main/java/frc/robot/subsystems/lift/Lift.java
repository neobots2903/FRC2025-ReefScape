package frc.robot.subsystems.lift;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;
import org.littletonrobotics.junction.Logger;

public class Lift extends SubsystemBase {
  private SparkMax leftMotor;
  private SparkMax rightMotor;
  private SparkClosedLoopController leftClosedLoopController;
  private SparkClosedLoopController rightClosedLoopController;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private double currentSetpoint = 0.0;

  public Lift() {
    // Initialize motors
    leftMotor = new SparkMax(LiftConstants.liftMotorOneCanID, MotorType.kBrushless);
    rightMotor = new SparkMax(LiftConstants.liftMotorTwoCanID, MotorType.kBrushless);

    // Configure motors - invert the left motor instead of negating its setpoint later
    configureMotor(leftMotor, true);
    configureMotor(rightMotor, false);

    // Get controllers and encoders after configuration
    leftClosedLoopController = leftMotor.getClosedLoopController();
    leftEncoder = leftMotor.getEncoder();
    rightClosedLoopController = rightMotor.getClosedLoopController();
    rightEncoder = rightMotor.getEncoder();
  }

  /**
   * Configures a SparkMax motor with standardized settings
   *
   * @param motor The SparkMax motor to configure
   * @param inverted Whether the motor direction should be inverted
   */
  private void configureMotor(SparkMax motor, boolean inverted) {
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    // Configure encoder
    motorConfig.encoder.positionConversionFactor(LiftConstants.POSITION_CONVERSION_FACTOR);

    // Configure closed loop controller with PID values
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(LiftConstants.PID_P)
        .i(LiftConstants.PID_I)
        .d(LiftConstants.PID_D)
        .outputRange(LiftConstants.OUTPUT_MIN, LiftConstants.OUTPUT_MAX);

    // Set motor inversion if needed
    if (inverted) {
      motorConfig.inverted(inverted);
    }

    // Apply configuration
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void runLiftToPos(double pos) {
    this.currentSetpoint = pos;
    leftClosedLoopController.setReference(
        this.currentSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    rightClosedLoopController.setReference(
        this.currentSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void periodic() {
    // Log motor currents
    Logger.recordOutput("Lift/LeftMotor/Current", leftMotor.getOutputCurrent());
    Logger.recordOutput("Lift/RightMotor/Current", rightMotor.getOutputCurrent());

    // Log motor positions
    Logger.recordOutput("Lift/LeftMotor/Position", leftEncoder.getPosition());
    Logger.recordOutput("Lift/RightMotor/Position", rightEncoder.getPosition());

    // Log motor velocities
    Logger.recordOutput("Lift/LeftMotor/Velocity", leftEncoder.getVelocity());
    Logger.recordOutput("Lift/RightMotor/Velocity", rightEncoder.getVelocity());

    // Log applied output
    Logger.recordOutput("Lift/LeftMotor/AppliedOutput", leftMotor.getAppliedOutput());
    Logger.recordOutput("Lift/RightMotor/AppliedOutput", rightMotor.getAppliedOutput());

    // Log temperatures
    Logger.recordOutput("Lift/LeftMotor/Temperature", leftMotor.getMotorTemperature());
    Logger.recordOutput("Lift/RightMotor/Temperature", rightMotor.getMotorTemperature());

    // Log control metrics
    Logger.recordOutput("Lift/Setpoint", currentSetpoint);
    Logger.recordOutput("Lift/LeftError", currentSetpoint - leftEncoder.getPosition());
    Logger.recordOutput("Lift/RightError", currentSetpoint - rightEncoder.getPosition());

    // Log motor synchronization (difference between motors)
    Logger.recordOutput(
        "Lift/PositionDifference", leftEncoder.getPosition() - rightEncoder.getPosition());
  }
}
