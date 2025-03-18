package frc.robot.subsystems.lift;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;
import org.littletonrobotics.junction.Logger;

public class Lift extends SubsystemBase {
  private final LiftIO io;
  private final LiftIO.LiftIOInputs inputs = new LiftIO.LiftIOInputs();

  private LiftPosition currentTargetPosition = LiftPosition.BOTTOM;
  private double currentSetpoint = LiftPosition.BOTTOM.getPosition();

  private static final double POSITION_TOLERANCE = 0.5; // inches

  // Enum for lift positions
  public enum LiftPosition {
    BOTTOM(LiftConstants.BOTTOM),
    LEVEL_ONE(LiftConstants.L_ONE),
    LEVEL_TWO(LiftConstants.L_TWO),
    LEVEL_THREE(LiftConstants.L_THREE),
    LEVEL_FOUR(LiftConstants.L_FOUR);

    private final double position;

    LiftPosition(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }

  /**
   * Creates a new Lift subsystem.
   *
   * @param io The lift IO interface implementation to use
   */
  public Lift(LiftIO io) {
    this.io = io;
    io.configureLift();
  }

  /**
   * Moves the lift to a predefined position.
   *
   * @param position The target position
   */
  public void runLift(LiftPosition position) {
    this.currentTargetPosition = position;
    this.currentSetpoint = position.getPosition();
    io.setPosition(this.currentSetpoint);
  }

  /**
   * Moves the lift to a specific height in inches.
   *
   * @param position The position in inches
   */
  public void runLiftToPos(double position) {
    this.currentSetpoint = position;

    // Update the current target position if it matches one of our predefined positions
    for (LiftPosition liftPos : LiftPosition.values()) {
      if (Math.abs(position - liftPos.getPosition()) < 0.01) {
        this.currentTargetPosition = liftPos;
        break;
      }
    }

    io.setPosition(this.currentSetpoint);
  }

  /**
   * Gets the current target position.
   *
   * @return The current target position
   */
  public LiftPosition getCurrentTargetPosition() {
    return currentTargetPosition;
  }

  /**
   * Gets the current actual position in inches.
   *
   * @return The current position in inches
   */
  public double getCurrentPosition() {
    // Average the two encoders for a more reliable reading
    return (inputs.leftPositionInches + inputs.rightPositionInches) / 2.0;
  }

  /**
   * Checks if the lift is at its target position within tolerance.
   *
   * @return true if the lift is at the target position
   */
  public boolean isAtTargetPosition() {
    double currentAvgPosition = getCurrentPosition();
    return Math.abs(currentAvgPosition - currentSetpoint) < POSITION_TOLERANCE;
  }

  @Override
  public void periodic() {
    // Update inputs from hardware
    io.updateInputs(inputs);

    // Log data
    Logger.recordOutput("Lift/LeftMotor/Current", inputs.leftCurrentAmps);
    Logger.recordOutput("Lift/RightMotor/Current", inputs.rightCurrentAmps);
    Logger.recordOutput("Lift/LeftMotor/Position", inputs.leftPositionInches);
    Logger.recordOutput("Lift/RightMotor/Position", inputs.rightPositionInches);
    Logger.recordOutput("Lift/LeftMotor/Velocity", inputs.leftVelocityInchesPerSec);
    Logger.recordOutput("Lift/RightMotor/Velocity", inputs.rightVelocityInchesPerSec);
    Logger.recordOutput("Lift/LeftMotor/AppliedVolts", inputs.leftAppliedVolts);
    Logger.recordOutput("Lift/RightMotor/AppliedVolts", inputs.rightAppliedVolts);
    Logger.recordOutput("Lift/SetpointInches", currentSetpoint);
    Logger.recordOutput("Lift/AtSetpoint", isAtTargetPosition());
  }
}
