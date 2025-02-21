package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team6328.GeomUtil;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class DriveToPose extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<Pose2d> currentRobotPose;
  private final ProfiledPIDController driveController;
  private final ProfiledPIDController thetaController;

  private Translation2d lastSetpointTranslation;

  private static final double MIN_RADIUS = 0.2; // 20 cm stop threshold
  private static final double MAX_RADIUS = 0.4; // 0.4 meter full speed threshold

  public DriveToPose(Drive drive, Supplier<Pose2d> targetPose, Supplier<Pose2d> currentRobotPose) {
    this.drive = drive;
    this.poseSupplier = targetPose;
    this.currentRobotPose = currentRobotPose;
    addRequirements(drive);

    driveController =
        new ProfiledPIDController(1, 0.0, 0.0, new TrapezoidProfile.Constraints(2.0, 3.0));
    thetaController =
        new ProfiledPIDController(0.1, 0.0, 0.0, new TrapezoidProfile.Constraints(4.0, 6.0));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    var pose = poseSupplier.get();
    var currentPose = currentRobotPose.get();
    driveController.reset(currentPose.getTranslation().getDistance(pose.getTranslation()));
    thetaController.reset(currentPose.getRotation().getRadians());
    lastSetpointTranslation = currentPose.getTranslation();
  }

  @Override
  public void execute() {
    var currentPose = currentRobotPose.get();
    var targetPose = poseSupplier.get();
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScaler =
        MathUtil.clamp((currentDistance - MIN_RADIUS) / (MAX_RADIUS - MIN_RADIUS), 0.0, 1.0);

    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);

    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(currentDistance, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;

    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    Translation2d driveVelocity =
        new Pose2d(
                Constants.zeroTranslation2d,
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
            .getTranslation();

    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    if (Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians())
        < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            -driveVelocity.getX(), -driveVelocity.getY(), -(thetaVelocity / 2)); // Theta is bad...
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  @Override
  public boolean isFinished() {
    return driveController.atGoal() && thetaController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
