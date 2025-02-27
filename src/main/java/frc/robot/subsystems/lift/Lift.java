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
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Lift extends SubsystemBase {

  private SparkMax leftMotor;
  private SparkMax rightMotor;
  private SparkMaxConfig leftMotorConfig;
  private SparkMaxConfig rightMotorConfig;
  private SparkClosedLoopController leftClosedLoopController;
  private SparkClosedLoopController rightClosedLoopController;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private double currentSetpoint;

  public Lift() {
    /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    leftMotor = new SparkMax(Constants.LiftConstants.liftMotorOneCanID, MotorType.kBrushless);
    leftClosedLoopController = leftMotor.getClosedLoopController();
    leftEncoder = leftMotor.getEncoder();

    rightMotor = new SparkMax(Constants.LiftConstants.liftMotorTwoCanID, MotorType.kBrushless);
    rightClosedLoopController = rightMotor.getClosedLoopController();
    rightEncoder = rightMotor.getEncoder();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    leftMotorConfig = new SparkMaxConfig();
    rightMotorConfig = new SparkMaxConfig();

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    leftMotorConfig.encoder.positionConversionFactor(0.25 * (2 * Math.PI)); // 0.25 * (Math.PI/180)

    rightMotorConfig.encoder.positionConversionFactor(
        0.25 * (2 * Math.PI)); // (0.25 * 180) / Math.PI

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    leftMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-0.3, 1);

    rightMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-0.3, 1);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    leftMotor.configure(
        leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(
        rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void runLiftToPos(double pos) {
    this.currentSetpoint = pos;
    leftClosedLoopController.setReference(
        -this.currentSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    rightClosedLoopController.setReference(
        this.currentSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Lift/RightMotor/Current", rightMotor.getOutputCurrent());
    Logger.recordOutput("Lift/RightMotor/Position", rightEncoder.getPosition());
    Logger.recordOutput("Lift/Setpoint", currentSetpoint);
  }
}
