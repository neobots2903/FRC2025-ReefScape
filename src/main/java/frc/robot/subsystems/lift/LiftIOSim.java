package frc.robot.subsystems.lift;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.LiftConstants;

public class LiftIOSim implements LiftIO {
  // Simulation objects
  private final ElevatorSim elevatorSim;
  private final PIDController controller;
  private double targetPosition = 0.0;
  private double appliedVolts = 0.0;

  public LiftIOSim() {
    // Create elevator simulation with newer WPILib constructor
    elevatorSim =
        new ElevatorSim(
            DCMotor.getNEO(2),
            LiftConstants.GEAR_RATIO,
            Units.lbsToKilograms(LiftConstants.CARRIAGE_MASS_LBS),
            Units.inchesToMeters(LiftConstants.DRUM_RADIUS_INCHES),
            0.0,
            Units.inchesToMeters(LiftConstants.L_FOUR),
            true,
            0.0);

    // Create PID controller for simulation
    controller = new PIDController(LiftConstants.PID_P, LiftConstants.PID_I, LiftConstants.PID_D);

    controller.setTolerance(0.01); // Small position tolerance in meters
  }

  // The rest of the class can remain unchanged
  @Override
  public void updateInputs(LiftIOInputs inputs) {
    // Calculate control output
    double pidOutput =
        controller.calculate(elevatorSim.getPositionMeters(), Units.inchesToMeters(targetPosition));

    // Apply voltage limits
    double voltage = MathUtil.clamp(pidOutput, -12.0, 12.0);
    appliedVolts = voltage;

    // Update simulation (20ms update)
    elevatorSim.setInputVoltage(voltage);
    elevatorSim.update(0.020);

    // Convert simulation values to our IO struct
    double posInches = Units.metersToInches(elevatorSim.getPositionMeters());
    double velInchesPerSec = Units.metersToInches(elevatorSim.getVelocityMetersPerSecond());
    double currentAmps = elevatorSim.getCurrentDrawAmps() / 2.0; // Split current between motors

    // Update inputs structure with simulated values
    inputs.leftPositionInches = posInches;
    inputs.rightPositionInches = posInches;
    inputs.leftVelocityInchesPerSec = velInchesPerSec;
    inputs.rightVelocityInchesPerSec = velInchesPerSec;
    inputs.leftCurrentAmps = currentAmps;
    inputs.rightCurrentAmps = currentAmps;
    inputs.leftAppliedVolts = appliedVolts;
    inputs.rightAppliedVolts = appliedVolts;
  }

  @Override
  public void setPosition(double positionInches) {
    targetPosition = positionInches;
  }

  @Override
  public void resetEncoders() {
    elevatorSim.setState(0.0, 0.0);
    controller.reset();
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }
}
