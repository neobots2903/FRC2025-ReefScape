package frc.robot.commands;

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

  private SimpleAuto() {
    // Utility class - prevent instantiation
  }

  /**
   * Creates a simple autonomous command that: 1. Drives forward 5 feet 2. Raises the lift to L1
   * position 3. Outtakes the game piece
   *
   * <p>Reef is 7'4" from the start line. Robot is 30" long. (58" total)
   *
   * @param drive The drive subsystem
   * @param lift The lift subsystem
   * @param endEffector The end effector subsystem
   * @return The autonomous command sequence
   */
  public static Command simpleCoral(Drive drive, Lift lift, EndEffector endEffector) {
    return Commands.sequence(
            // Step 1: Drive forward.
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting drive forward")),
            DriveCommands.driveDistance(drive, 58.0),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed drive forward")),

            // Step 2: Raise lift to L3 position
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting lift to L3")),
            Commands.runOnce(() -> lift.runLiftToPos(LiftConstants.L_THREE)),

            // Wait for lift to get to position (1.5 second should be enough)
            Commands.waitSeconds(2),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed lift to L3")),

            // Step 3: Outtake the game piece
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting outtake")),
            new InstantCommand(() -> endEffector.intake()), // Push out coral
            Commands.waitSeconds(1.25), // Wait for action to complete
            new InstantCommand(() -> endEffector.stop()), // Stop intake mech
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed outtake")),

            // Step 4: Back up from reef
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting backup")),
            DriveCommands.driveDistance(drive, -2.0), 
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed backup")),
            
            // Step 5: Prepare for teleop (maintain lift position)
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Reduce lift position for teleop")),
            Commands.runOnce(() -> lift.runLiftToPos(LiftConstants.BOTTOM)),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Auto sequence complete")))
        .withName("Simple Coral Auto");
  }
}
