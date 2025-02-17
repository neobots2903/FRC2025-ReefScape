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

/* NOTES */
/*
 * - The end effector (wheel) motors are Brushed motors, so we'd need to pass in MotorType.kBrushed.
 * - The alge descorer doesn't exist yet.
 * - Need to figure out if the limit switches are normally open or normally closed. The logic might just be inverted.
 * - If you want to use the periodic method, it needs to be named "periodic" and needs to use the @Override annotation.
 */

public class EndEffector extends SubsystemBase {

  // Motors
  private SparkMax m_endEffectorOne; // Motors for the end effector.
  private SparkMax m_endEffectorTwo;
  private SparkMax m_algeDescorer; // Motor for the algeDescorer

  // Motor configs
  SparkMaxConfig m_endEffectorOne_config; // Configs for the end effector motors.
  SparkMaxConfig m_endEffectorTwo_config;
  SparkMaxConfig m_algeDescorer_config; // Config for the Alge Descorer.

  // Limit switchs
  DigitalInput intakeLimitSwitch; // Limit switches for the End Effector intaking and outtaking.
  DigitalInput outtakeLimitSwitch;

  // Data
  int framesSinceIntakeLimitSwitch = 0;
  public boolean outtaking =
      false; // If true, the End Effector is currently outtaking, if false, it is not.
  public boolean runAlgeDescorer = false; // If true, run the alge descorer, else, do not.

  // Constructor.
  public EndEffector() {

    // Intialize motors
    m_endEffectorOne =
        new SparkMax(EndEffectorConstants.endEffectorMotorOne_Port, MotorType.kBrushless);
    m_endEffectorTwo =
        new SparkMax(EndEffectorConstants.endEffectorMotorTwo_Port, MotorType.kBrushless);
    m_algeDescorer =
        new SparkMax(EndEffectorConstants.algeDescorerMotor_Port, MotorType.kBrushless);

    // Intialize configs for the end effector motors
    m_endEffectorOne_config = new SparkMaxConfig();
    m_endEffectorTwo_config = new SparkMaxConfig();
    m_endEffectorOne_config.smartCurrentLimit(30).idleMode(IdleMode.kBrake);
    m_endEffectorTwo_config.smartCurrentLimit(30).idleMode(IdleMode.kBrake);

    // Intialize config for the Alge Descorer motors
    m_algeDescorer_config = new SparkMaxConfig();
    m_algeDescorer_config.smartCurrentLimit(30).idleMode(IdleMode.kBrake);

    // Push config onto end effector motors
    m_endEffectorOne.configure(
        m_endEffectorOne_config,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    m_endEffectorTwo.configure(
        m_endEffectorTwo_config,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    // Push config onto the alge Descorer
    m_algeDescorer.configure(
        m_algeDescorer_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // Configure Limit Switches
    intakeLimitSwitch = new DigitalInput(EndEffectorConstants.intakeLimitSwitchPort);
    outtakeLimitSwitch = new DigitalInput(EndEffectorConstants.outtakeLimitSwitchPort);
  }

  // Runs frame/cycle of the application;
  // Used to check limit switchs and run other systems in the background for the End Effector.
  public void periodicMain() {

    // Monitor and control the intake/outtake system.
    if (outtaking == false) {
      intakeControl();
    } else if (outtaking == true) {
      outakeControl();
    }
  }

  // Outtake until the outtake limit switch is not active.
  public void outakeControl() {
    if (outtakeLimitSwitch.get()) {
      m_endEffectorOne.set(
          -EndEffectorConstants
              .endEffectorSpeed); // If the outtake switch is actuated, spin the motors.
      m_endEffectorTwo.set(EndEffectorConstants.endEffectorSpeed);
    } else if (outtakeLimitSwitch.get()
        == false) { // If the outake switch is off, cut the motors and set the outtaking status back
      // to false.
      m_endEffectorOne.set(0.0);
      m_endEffectorTwo.set(0.0);

      outtaking = false;
    }
  }

  // If the intake limit switch is activated, intake until the outtake limit switch is down and the
  // intake is not.
  public void intakeControl() {

    // If the intake limit switch is pressed, intake until it is no longer pressed.
    if (intakeLimitSwitch.get()) {
      m_endEffectorOne.set(EndEffectorConstants.endEffectorSpeed);
      m_endEffectorTwo.set(-EndEffectorConstants.endEffectorSpeed);

      framesSinceIntakeLimitSwitch =
          0; // Since the intake limit switch was pressed, set the frames since last press to zero.
    } else if (framesSinceIntakeLimitSwitch == 1) {
      m_endEffectorOne.set(
          0.0); // Set the end effector motors to off if its been 1 frame since the limit switch was
      // pressed (limit switch is pressed)
      m_endEffectorTwo.set(0.0);
    }

    framesSinceIntakeLimitSwitch++; // Increase the frames that have passed since the intake limit
    // switch was actuated.
  }

  // Runs the descorer when called.
  public void runAlgeDescorer() {
    m_algeDescorer.set(1.0);
  }

  // Stops the Alge Descorer.
  public void cutAlgeDescorer() {
    m_algeDescorer.set(0.0);
  }
}
