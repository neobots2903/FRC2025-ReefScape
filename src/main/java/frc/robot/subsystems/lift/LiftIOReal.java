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
import frc.robot.Constants.LiftConstants;

public class LiftIOReal implements LiftIO {
  private SparkMax leftMotor;
  private SparkMax rightMotor;
  private SparkClosedLoopController leftClosedLoopController;
  private SparkClosedLoopController rightClosedLoopController;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  /** Constructs the LiftIOReal hardware interface */
  public LiftIOReal() {
    // Initialize motors
    leftMotor = new SparkMax(LiftConstants.liftMotorOneCanID, MotorType.kBrushless);
    rightMotor = new SparkMax(LiftConstants.liftMotorTwoCanID, MotorType.kBrushless);

    // Initial configuration
    configureLift();

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
   * @param follower Whether this motor is a follower
   * @param followID ID of the motor to follow (if follower is true)
   */
  private void configureMotor(SparkMax motor, boolean inverted, boolean follower, int followID) {
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

    motorConfig.smartCurrentLimit(40);

    if (follower) {
      motorConfig.follow(followID, inverted);
    }

    // Set motor inversion if needed
    if (inverted) {
      motorConfig.inverted(inverted);
    }

    // Apply configuration
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void configureLift() {
    // Configure motors - invert the left motor instead of negating its setpoint later
    configureMotor(leftMotor, true, true, LiftConstants.liftMotorTwoCanID);
    configureMotor(rightMotor, false, false, 0);
  }

  @Override
  public void updateInputs(LiftIOInputs inputs) {
    // Read hardware values and update inputs struct
    inputs.leftPositionInches = leftEncoder.getPosition();
    inputs.rightPositionInches = rightEncoder.getPosition();
    inputs.leftVelocityInchesPerSec = leftEncoder.getVelocity();
    inputs.rightVelocityInchesPerSec = rightEncoder.getVelocity();
    inputs.leftCurrentAmps = leftMotor.getOutputCurrent();
    inputs.rightCurrentAmps = rightMotor.getOutputCurrent();
    inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
  }

  @Override
  public void setPosition(double positionInches) {
    // Send position command to the motor
    rightClosedLoopController.setReference(
        positionInches, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  @Override
  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }
}
