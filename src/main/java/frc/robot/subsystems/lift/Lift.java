package frc.robot.subsystems.lift;

import org.littletonrobotics.junction.Logger;

import com.thethriftybot.Conversion;
import com.thethriftybot.Conversion.PositionUnit;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.EncoderType;
import com.thethriftybot.ThriftyNova.PIDSlot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
  // Physical constants
  private static final double INCHES_PER_RADIAN = 2.5;
  private static final double POSITION_TOLERANCE = 0.25; // inches

  // Motor configuration constants
  private static final double RAMP_UP_TIME = 0.25;
  private static final double RAMP_DOWN_TIME = 0.05;
  private static final double FORWARD_MAX_OUTPUT = 1.0;
  private static final double REVERSE_MAX_OUTPUT = 0.25;
  private static final double SOFT_LIMIT_MIN = 0;
  private static final double SOFT_LIMIT_MAX = 7 * Math.PI;
  private static final int CURRENT_LIMIT = 50;

  // PID constants
  private static final double kP = 0.5;
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double kFF = 1.2;

  public enum SetPoint {
    BOTTOM(0),
    L1(18),
    L2(32),
    L3(42),
    L4(56);

    final double position;

    SetPoint(double position) {
      this.position = position;
    }
  }

  private final ThriftyNova m_leftMotor;
  private final ThriftyNova m_rightMotor;
  private final Conversion converter;

  private SetPoint currentSetPoint = SetPoint.BOTTOM;
  private double currentPosition = 0;

  // Singleton instance
  private static volatile Lift instance;

  public static synchronized Lift getInstance() {
    if (instance == null) {
      instance = new Lift();
    }
    return instance;
  }

  private Lift() {
    converter = new Conversion(PositionUnit.RADIANS, EncoderType.INTERNAL);

    m_leftMotor = configureMotor(100, false); // CAN ID 100, not inverted
    m_rightMotor = configureMotor(101, true); // CAN ID 101, inverted
  }

  private ThriftyNova configureMotor(int canId, boolean inverted) {
    ThriftyNova motor = new ThriftyNova(canId);

    // Basic motor configuration
    motor.setBrakeMode(true);
    motor.setInverted(inverted);
    motor.setRampUp(RAMP_UP_TIME);
    motor.setRampDown(RAMP_DOWN_TIME);
    motor.setMaxOutput(FORWARD_MAX_OUTPUT, REVERSE_MAX_OUTPUT);

    // Safety limits
    motor.setSoftLimits(SOFT_LIMIT_MIN, SOFT_LIMIT_MAX);
    motor.enableSoftLimits(true);
    motor.setMaxCurrent(CurrentType.SUPPLY, CURRENT_LIMIT);

    // Encoder and PID configuration
    motor.useEncoderType(EncoderType.INTERNAL);
    motor.usePIDSlot(PIDSlot.SLOT0);

    // PID configuration
    motor.pid0.setP(kP);
    motor.pid0.setI(kI);
    motor.pid0.setD(kD);
    motor.pid0.setFF(kFF);

    motor.clearErrors();
    return motor;
  }

  public void setPosition(SetPoint setPoint) {
    this.currentSetPoint = setPoint;
    double motorPosition = converter.toMotor(setPoint.position / INCHES_PER_RADIAN);
    m_leftMotor.setPosition(motorPosition);
    m_rightMotor.setPosition(motorPosition);
  }

  public boolean atSetpoint() {
    return Math.abs(currentPosition - currentSetPoint.position) < POSITION_TOLERANCE;
  }

  @Override
  public void periodic() {
    currentPosition = converter.fromMotor(m_leftMotor.getPosition()) * INCHES_PER_RADIAN;

    // Dashboard telemetry
    Logger.recordOutput("Elevator/Position", currentPosition);
    Logger.recordOutput("Elevator/Current", m_leftMotor.getStatorCurrent());
    Logger.recordOutput("Elevator/Target", currentSetPoint.position);
    Logger.recordOutput("Elevator/AtSetpoint", atSetpoint());
  }
}
