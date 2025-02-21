// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Translation2d zeroTranslation2d = new Translation2d();
  public static final Rotation2d zeroRotation2d = new Rotation2d();
  public static final Pose2d zeroPose2d = new Pose2d();
  public static final Pose3d zeroPose3d = new Pose3d();

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class ClimbConstants {
    public static final int leftGrip_motorPort = 11; // Neo + spark
    public static final int rightGrip_motorPort = 12; // Neo + Spark
    public static final int rotateIntakeFunnel_motorPort = 16; // Krakern
    public static final double kGripMotorSpeeds = 0.1; // Speeds for the grip motors on the climb.
    public static final double kGripMotorOpenSpeeds = -0.1; // Speeds for the grip motors to open.
    public static final double kGripMotorStopped = 0.0; // Speed to stop the grip motors.

    // current constants
    public static final double kClimbPercentEnabled = 0.2;
    public static final double kClimbPercentDisabled = 0.0;
  }

  public static final class RampMechanismConstants {

    // TalonFX (Kraken) pivot motor
    public static final int rampMechanismPivot_motorPort = 13;
    public static final int rampMechanismPivot_stallAmperage = 50;
    public static final boolean rampMechanismPivot_EnableStatorCurrentLimit = true;
    public static final double rampMechanismPivot_kP = 0.0;
    public static final double rampMechanismPivot_kI = 0.0;
    public static final double rampMechanismPivot_kD = 0.0;
    public static final double rampMechanismPivot_kP1 = 0.0;
    public static final double rampMechanismPivot_kI1 = 0.0;
    public static final double rampMechanismPivot_kD1 = 0.0;
    public static final double rampMechanismPivot_gearRatio =
        12.0; // Gear ratio for the pivot motor for the ramp. For example, if this was was 8, gear
    // ratio would be 1:8 (8 full motor rotations for 1 shaft rotation.)
  }

  public static final class LiftConstants {
    public static final double maxLiftPosition = 20.0;
    public static final double minLiftPosition = 0.0;
    public static final double kP = 0.1;
    public static final double kI = 0.1;
    public static final double kD = 0.1;
    public static final double kMinOutput = 0.1;
    public static final double kMaxOutput = 0.5;
  }

  public static final class EndEffectorConstants {
    public static final int endEffectorMotorOne_Port = 15;
    public static final int endEffectorMotorTwo_Port = 16;
    public static final int intakeLimitSwitchPort = 0;
    public static final int outtakeLimitSwitchPort = 1;
    public static final double endEffectorSpeed = 0.2;
    public static final int algeDescorerMotor_Port = 17;
  }
}
