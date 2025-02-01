package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RampMechanismConstants;

/*
 * Class to manage the Ramp Mechanism for the intake.
 */
public class RampMechanism extends SubsystemBase {

  private final PositionVoltage m_positionVoltage =
      new PositionVoltage(0)
          .withSlot(0); // No idea for what this does, used to go to a position with the encoder?

  // Motors
  private TalonFX m_RampMechanism_PivotMotor; // Pivot motor for the Ramp Mechanism.

  // Configurations
  private TalonFXConfiguration rampMechanismPivotMotorConfig; // Configuration for the pivot motor.
  // CurrentLimitsConfigs rampMechanismPivotMotor_LimitConfiguration; //Limits configuration for the
  // kraken motor.

  // Constructor, init hardware and other software components here;
  // Run setups if needed.
  public RampMechanism() {
    // Intialize motor objects
    m_RampMechanism_PivotMotor =
        new TalonFX(
            RampMechanismConstants.rampMechanismPivot_motorPort,
            "krakenX60"); // Intializing motor for the Ramp Mechanism pivot.

    // Setup motor configurations and variables (init motors)
    initMotorConfigurations();
  }

  // Runs the Ramp to position for a hang.
  public void toHangPosition() {
    m_RampMechanism_PivotMotor.setControl(
        m_positionVoltage.withPosition(.25 * RampMechanismConstants.rampMechanismPivot_gearRatio));
  }

  // Runs the Ramp to a position to intake.
  public void toIntakePosition() {
    m_RampMechanism_PivotMotor.setControl(
        m_positionVoltage.withPosition(0 * RampMechanismConstants.rampMechanismPivot_gearRatio));
  }

  // Sets up motor configurations for all motors.
  void initMotorConfigurations() {

    /*
     *Setup configuration for the RampMechanism pivot motor.
     */
    rampMechanismPivotMotorConfig = new TalonFXConfiguration();

    rampMechanismPivotMotorConfig.Slot0.kP =
        RampMechanismConstants
            .rampMechanismPivot_kP; // An error of 1 rotation results in 2.4 V output
    rampMechanismPivotMotorConfig.Slot0.kI =
        RampMechanismConstants.rampMechanismPivot_kI; // No output for integrated error
    rampMechanismPivotMotorConfig.Slot0.kD =
        RampMechanismConstants.rampMechanismPivot_kD; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
    rampMechanismPivotMotorConfig
        .Voltage
        .withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-8));

    rampMechanismPivotMotorConfig.Slot1.kP =
        RampMechanismConstants
            .rampMechanismPivot_kP1; // An error of 1 rotation results in 60 A output
    rampMechanismPivotMotorConfig.Slot1.kI =
        RampMechanismConstants.rampMechanismPivot_kI1; // No output for integrated error
    rampMechanismPivotMotorConfig.Slot1.kD =
        RampMechanismConstants.rampMechanismPivot_kD1; // A velocity of 1 rps results in 6 A output
    // Peak output of 120 A
    rampMechanismPivotMotorConfig
        .TorqueCurrent
        .withPeakForwardTorqueCurrent(
            Amps.of(RampMechanismConstants.rampMechanismPivot_stallAmperage))
        .withPeakReverseTorqueCurrent(
            Amps.of(-RampMechanismConstants.rampMechanismPivot_stallAmperage));

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_RampMechanism_PivotMotor.getConfigurator().apply(rampMechanismPivotMotorConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    /* Make sure we start at 0 */
    m_RampMechanism_PivotMotor.setPosition(0);
  }
}
