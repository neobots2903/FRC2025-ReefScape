package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class RumbleFeedbackSubsystem extends SubsystemBase {
  private final Drive drive;
  private final CommandXboxController controller;

  // Rumble configuration
  private static final double ALIGNMENT_THRESHOLD = Units.inchesToMeters(1.5);
  private static final double LATERAL_THRESHOLD = Units.inchesToMeters(6);
  private static final double WEAK_RUMBLE = 0.4;
  private static final double STRONG_RUMBLE = 1.0;

  // Tracking state
  private Pose2d targetPose = null;
  private boolean alignmentActive = false;

  public RumbleFeedbackSubsystem(Drive drive, CommandXboxController controller) {
    this.drive = drive;
    this.controller = controller;
  }

  /** Start providing alignment feedback toward the specified pose */
  public void startAlignment(Pose2d targetPose) {
    this.targetPose = targetPose;
    this.alignmentActive = true;
    System.out.println("Rumble feedback activated with target: " + targetPose);
  }

  /** Stop providing alignment feedback */
  public void stopAlignment() {
    if (alignmentActive) {
      controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
      alignmentActive = false;
      System.out.println("Rumble feedback deactivated");
    }
  }

  @Override
  public void periodic() {
    // Skip if not in alignment mode or if in simulation
    if (!alignmentActive || targetPose == null || Constants.currentMode == Constants.Mode.SIM) {
      return;
    }

    Pose2d currentPose = drive.getPose();
    double dx = currentPose.getX() - targetPose.getX();
    double dy = currentPose.getY() - targetPose.getY();

    double distance = Math.sqrt(dx * dx + dy * dy);
    double targetHeading = targetPose.getRotation().getRadians();
    double lateralError = Math.sin(targetHeading) * dx - Math.cos(targetHeading) * dy;

    Logger.recordOutput("Alignment/DistanceError", distance);
    Logger.recordOutput("Alignment/LateralError", lateralError);

    // Apply rumble based on position
    if (distance < ALIGNMENT_THRESHOLD) {
      // Well aligned - strong rumble both sides
      controller.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, STRONG_RUMBLE);
      controller.getHID().setRumble(GenericHID.RumbleType.kRightRumble, STRONG_RUMBLE);
    } else if (lateralError > LATERAL_THRESHOLD) {
      // Too far right - rumble left
      controller.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, WEAK_RUMBLE);
      controller.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 0);
    } else if (lateralError < -LATERAL_THRESHOLD) {
      // Too far left - rumble right
      controller.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0);
      controller.getHID().setRumble(GenericHID.RumbleType.kRightRumble, WEAK_RUMBLE);
    } else {
      // Within lateral threshold but not perfectly aligned
      controller.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, WEAK_RUMBLE * 0.6);
      controller.getHID().setRumble(GenericHID.RumbleType.kRightRumble, WEAK_RUMBLE * 0.6);
    }
  }
}
