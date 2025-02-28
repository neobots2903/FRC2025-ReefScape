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

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
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

  public static final class RampMechanismConstants {
    // Neo 1.1 with Spark Max
    public static final int pivotMotorOne_motorPort = 29;
    public static final int pivotMotorTwo_motorPort = 30;
    public static final int rampMechanismPivot_stallAmperage = 50;
    public static final boolean rampMechanismPivot_EnableStatorCurrentLimit = true;
    public static final double rampMechanismPivot_kP = 0.05;
    public static final double rampMechanismPivot_kI = 0.0;
    public static final double rampMechanismPivot_kD = 0.0;
    public static final double rampMechanismPivot_gearRatio = 100.0;
    public static final double OUTPUT_MIN = -0.5;
    public static final double OUTPUT_MAX = 0.5;

    // ROTATIONS
    public static final double PULL_CAGE_DOWN = -5.0;
    public static final double ROTATION_INTAKE = -37.0;
    public static final double ROTATION_HANG = -119.0;
  }

  public static final class LiftConstants {
    public static final int liftMotorOneCanID = 20;
    public static final int liftMotorTwoCanID = 21;

    // PID and motor configuration constants
    // Come back to tuning PID once other mechanisms are working. Good enough for now.
    public static final double INCHES_PER_RADIAN =
        ((1.0 / 7.0) * (1.751 / 2.0))
            * 2; // 7:1 gear ratio, 1.751" pitch(?) diameter, times 2 for second stage of lift
    public static final double POSITION_CONVERSION_FACTOR =
        INCHES_PER_RADIAN
            * (2 * Math.PI); // Convert from inches per radian to inches per revolution
    public static final double PID_P = 0.1;
    public static final double PID_I = 0.0;
    public static final double PID_D = 0.25;
    public static final double OUTPUT_MIN = -0.5;
    public static final double OUTPUT_MAX = 1.0;

    // Lift Heights (Inches)
    private static final double BASE_LIFT = 7.5;
    public static final double L_ONE = 15 - BASE_LIFT;
    public static final double L_TWO = 25.5 - BASE_LIFT;
    public static final double L_THREE = 41 - BASE_LIFT;
    public static final double L_FOUR = 67.5 - BASE_LIFT;
    public static final double BOTTOM = 0.1;
  }

  public static final class EndEffectorConstants {
    public static final int endEffectorMotorOne_Port = 22;
    public static final int endEffectorMotorTwo_Port = 23;
    public static final int intakeLimitSwitchPort = 0;
    public static final int outtakeLimitSwitchPort = 1;
    public static final double endEffectorSpeed = 0.2;
    public static final int algeDescorerMotor_Port = 24;
  }

  public static final class ClimbConstants {
    // Motor ports
    public static final int leftGrip_motorPort = 27; // Neo + spark
    public static final int rightGrip_motorPort = 28; // Neo + Spark

    // PID and motor configuration
    public static final double CLAW_GEAR_RATIO = 49.0; // 49:1 gear ratio
    public static final double CLAW_PID_P = 0.05;
    public static final double CLAW_PID_I = 0.0;
    public static final double CLAW_PID_D = 0.0;
    public static final double CLAW_OUTPUT_MIN = -0.75;
    public static final double CLAW_OUTPUT_MAX = 0.75;

    // Claw position setpoints (degrees)
    public static final double CLAW_OPEN_POSITION = 0.0;
    public static final double CLAW_GRIP_POSITION = 85.0;

    // Left 30, Right 85...

    // Legacy constants (can be removed later)
    public static final double kGripMotorSpeeds = 0.1;
    public static final double kGripMotorOpenSpeeds = -0.1;
    public static final double kGripMotorStopped = 0.0;
    public static final double kClimbPercentEnabled = 0.2;
    public static final double kClimbPercentDisabled = 0.0;
  }
}
