package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.lift.Lift;
import org.littletonrobotics.junction.Logger;

/** Contains simple autonomous routines for the robot. */
public class AutoCommands {

  private static final double ROBOT_OFFSET = 36.0 / 2; // Half robot length in inches

  private AutoCommands() {
    // Utility class - prevent instantiation
  }

  /**
   * Creates a command that scores a coral game piece by raising the lift, outtaking the game piece,
   * and then lowering the lift.
   *
   * @param drive The drive subsystem
   * @param lift The lift subsystem
   * @param endEffector The end effector subsystem
   * @return The command sequence
   */
  public static Command ScoreCoral(Lift lift, EndEffector endEffector) {
    return Commands.sequence(
            // Step 1: Raise lift to position
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting lift to L4")),
            Commands.runOnce(() -> lift.runLiftToPos(LiftConstants.L_FOUR)),

            // Wait for lift to get to position (1.5 second should be enough)
            Commands.waitSeconds(1.5),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed lift to L4")),

            // Step 3: Outtake the game piece
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting outtake")),
            new InstantCommand(() -> endEffector.intake()), // Push out coral
            Commands.waitSeconds(1.25), // Wait for action to complete
            new InstantCommand(() -> endEffector.stop()), // Stop intake mech
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed outtake")),

            // Step 4: Lower lift to L1
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting lift to L1")),
            Commands.runOnce(() -> lift.runLiftToPos(LiftConstants.L_ONE)),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed lift to L1")))
        .withName("ScoreCoral");
  }

  public static Command RemoveAlgae(Lift lift, EndEffector endEffector, double reefPos) {
    return Commands.sequence(
            // Step 1: Raise lift to position
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting lift to pos")),
            Commands.runOnce(() -> lift.runLiftToPos(reefPos)),

            // Wait for lift to get to position (1 second should be enough)
            Commands.waitSeconds(1),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed lift to pos")),

            // Step 3: Remove algae (not written yet)
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting removal")),
            // Command goes here!!!
            Commands.waitSeconds(1.25), // Wait for action to complete
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed removal")),

            // Step 4: Lower lift to L1
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting lift to L1")),
            Commands.runOnce(() -> lift.runLiftToPos(LiftConstants.L_ONE)),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed lift to L1")))
        .withName("RemoveAlgae");
  }

  /**
   * Creates a simple autonomous command that: 1. Drives forward 5ish feet 2. Raises the lift pos 3.
   * Outtakes the game piece
   *
   * <p>Reef is 7'4" from the start line. Robot is 36" long. (52?" total)
   *
   * @param drive The drive subsystem
   * @param lift The lift subsystem
   * @param endEffector The end effector subsystem
   * @return The autonomous command sequence
   */
  public static Command simpleCoral(
      Drive drive, Lift lift, EndEffector endEffector, double reefPos) {
    return Commands.sequence(
            // Step 1: Drive forward.
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting drive forward")),
            DriveCommands.driveDistance(drive, 52.0 + ROBOT_OFFSET), // From robot front.
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed drive forward")),

            // Step 2: Raise lift to position
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting lift to Pos")),
            Commands.runOnce(() -> lift.runLiftToPos(reefPos)),

            // Wait for lift to get to position (1.5 second should be enough)
            Commands.waitSeconds(1.5),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed lift to Pos")),

            // Step 3: Outtake the game piece
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting outtake")),
            new InstantCommand(() -> endEffector.intake()), // Push out coral
            Commands.waitSeconds(1.25), // Wait for action to complete
            new InstantCommand(() -> endEffector.stop()), // Stop intake mech
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed outtake")),

            // Step 4: Back up from reef
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting backup")),
            DriveCommands.driveDistance(drive, -6.0),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed backup")),

            // Step 5: Prepare for teleop (maintain lift position)
            Commands.runOnce(
                () -> Logger.recordOutput("Auto/Status", "Reduce lift position for teleop")),
            Commands.runOnce(() -> lift.runLiftToPos(LiftConstants.L_ONE)),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Auto sequence complete")),

            // Step 6: Reset robot pose to match field
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                (drive
                                    .getPose()
                                    .getRotation()
                                    .minus(new Rotation2d(Math.toRadians(-180)))))),
                    drive)
                .ignoringDisable(true))
        .withName("Simple Coral Auto");
  }
}
