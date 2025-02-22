package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.RampMechanismConstants;

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
  //Motors for pivot
  private SparkMax m_pivotMotorOne;
  private SparkMax m_pivotMotorTwo;

  // Configurations
  private SparkMaxConfig pivotMotorOneConfig;
  private SparkMaxConfig pivotMotorTwoConfig;


  // Constructor, init hardware and other software components here;
  // Run setups if needed.
  public RampMechanism() {
    

    // Setup motor configurations and variables (init motors)
    initMotorConfigurations();
  }

  // Runs the Ramp to position for a hang.
  public void toHangPosition() {
    
  }

  // Runs the Ramp to a position to intake.
  public void toIntakePosition() {
    
  }

  // Sets up motor configurations for all motors.
  void initMotorConfigurations() {

    //Init motors for pivot
    m_pivotMotorOne = new SparkMax(ClimbConstants.pivotMotorOne_motorPort, MotorType.kBrushless);
    m_pivotMotorTwo = new SparkMax(ClimbConstants.pivotMotorOne_motorPort, MotorType.kBrushless);
  
    //Init configs for pivot motors
    pivotMotorOneConfig = new SparkMaxConfig();
    pivotMotorTwoConfig = new SparkMaxConfig();
    pivotMotorOneConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);
    pivotMotorTwoConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);
  
    //Add configs to pivot motors
    m_pivotMotorOne.configure(pivotMotorOneConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_pivotMotorTwo.configure(pivotMotorTwoConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
