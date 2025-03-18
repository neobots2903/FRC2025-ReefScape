package frc.robot.subsystems.lift;

public interface LiftIO {
  /** Container for inputs from lift hardware */
  public static class LiftIOInputs {
    public double leftPositionInches = 0.0;
    public double rightPositionInches = 0.0;
    public double leftVelocityInchesPerSec = 0.0;
    public double rightVelocityInchesPerSec = 0.0;
    public double leftCurrentAmps = 0.0;
    public double rightCurrentAmps = 0.0;
    public double leftAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;
  }

  /** Updates the set of inputs */
  public default void updateInputs(LiftIOInputs inputs) {}

  /** Configure lift hardware */
  public default void configureLift() {}

  /** Set closed-loop position target */
  public default void setPosition(double positionInches) {}

  /** Zero the lift encoders */
  public default void resetEncoders() {}

  /** Stop the motors */
  public default void stop() {}
}
