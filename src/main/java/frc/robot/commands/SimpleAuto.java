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
public class SimpleAuto {

  private static final double ROBOT_OFFSET =
      30.0 / 2; // Half robot length in inches (36 WITH BUMBERS, NEED TO TEST!!!)

  private SimpleAuto() {
    // Utility class - prevent instantiation
  }

  /**
   * Creates a simple autonomous command that: 1. Drives forward 5ish feet 2. Raises the lift to L3
   * position 3. Outtakes the game piece
   *
   * <p>Reef is 7'4" from the start line. Robot is 30" long. (58" total)
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
            DriveCommands.driveDistance(drive, 54.0 + ROBOT_OFFSET), // From robot front.
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed drive forward")),

            // Step 2: Raise lift to L3 position
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting lift to L3")),
            Commands.runOnce(() -> lift.runLiftToPos(reefPos)),

            // Wait for lift to get to position (1.5 second should be enough)
            Commands.waitSeconds(1.5),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed lift to L3")),

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

    /*Commands.runOnce(
        () ->
            drive.setPose(
                new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
        drive)
    .ignoringDisable(true)*/

    // (drive.getPose().getRotation().minus(new Rotation2d(-180))))
  }
}
