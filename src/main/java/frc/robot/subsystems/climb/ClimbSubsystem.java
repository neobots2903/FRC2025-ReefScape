// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

// TODO:
/*
 * Need to find out motor controller type (Spark Max, Talon, Victor SP, Victor SPX).
 * Find out which encoders the build team will grace us with, if any...
 */

// Hardware:
/*
 * 2 CIM for climb.
 * 1 mini CIM for engage.
 */

// NOTES:
/*
 * Could also do position control, but I was thinking that Current would be easier to
 * work with and less likely to kill motors. Empirically find amps under normal climb, then
 * use as target current.
 */

public class ClimbSubsystem extends SubsystemBase {

  //Grip motors for hang.
  private final SparkMax m_leftGrip;
  private final SparkMax m_rightGrip;

  //Config for grip motors for hang.
  private SparkMaxConfig leftGripConfig;
  private SparkMaxConfig rightGripConfig;

  public ClimbSubsystem() {
    
    //Intialize grip motors for hang.
    m_leftGrip = new SparkMax(ClimbConstants.leftGrip_motorPort, MotorType.kBrushless);
    m_rightGrip = new SparkMax(ClimbConstants.rightGrip_motorPort, MotorType.kBrushless);
  
    //Configs for grip motors for hang.
    leftGripConfig = new SparkMaxConfig();
    rightGripConfig = new SparkMaxConfig();

    //Configure motors for grip hanging.
    leftGripConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);
    rightGripConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);

    //Set configurations onto hang grip motors.
    m_leftGrip.configure(leftGripConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rightGrip.configure(rightGripConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  //Clamps the grip motors togther to grip the cage for asscent.
  public void runGripMotors() {
      m_leftGrip.set(ClimbConstants.kGripMotorSpeeds);
      m_rightGrip.set(-ClimbConstants.kGripMotorSpeeds);
  }

  //Opens the grip motors to release grip on hang (Deep cage)
  public void openGripMotors() {
    m_leftGrip.set(ClimbConstants.kGripMotorOpenSpeeds);
    m_rightGrip.set(-ClimbConstants.kGripMotorOpenSpeeds);
  }

  //Stops the grip motors (Sets power to 0)
  public void stopGripMotors() {
    m_leftGrip.set(ClimbConstants.kGripMotorStopped);
    m_rightGrip.set(ClimbConstants.kGripMotorStopped);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
