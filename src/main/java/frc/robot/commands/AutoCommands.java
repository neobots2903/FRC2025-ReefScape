package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.lift.Lift;
import org.littletonrobotics.junction.Logger;

/** Contains simple autonomous routines for the robot. */
public class AutoCommands {

  private AutoCommands() {
    // Utility class - prevent instantiation
  }

  /**
   * Creates a command that scores a coral game piece by raising the lift, shooting the game piece,
   * and then lowering the lift.
   *
   * @param lift The lift subsystem
   * @param endEffector The end effector subsystem
   * @return The command sequence
   */
  public static Command ScoreCoral(Lift lift, EndEffector endEffector) {
    return Commands.sequence(
            // Step 1: Raise lift to position
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting lift to L4")),
            // Commands.runOnce(() -> lift.runLiftToPos(LiftConstants.L_FOUR)),
            IntakeCommands.safeLiftToPosition(endEffector, lift, LiftConstants.L_FOUR),

            // Wait for lift to get to position (1.25 second should be enough)
            Commands.waitSeconds(1.25),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed lift to L4")),

            // Step 3: Shoot the game piece
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting to shoot piece")),
            IntakeCommands.shootGamePiece(endEffector), // Start shooting
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed shooting piece")),

            // Step 4: Lower lift to L2
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting lift to L2")),
            Commands.runOnce(() -> lift.runLiftToPos(LiftConstants.L_TWO)),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed lift to L2")))
        .withName("ScoreCoral");
  }

  /**
   * Creates a command that removes algae from a reef by raising the lift to the specified position,
   * running the algae removal motor while driving backward, and then lowering the lift.
   *
   * @param drive The drive subsystem needed for the pullback motion
   * @param lift The lift subsystem
   * @param endEffector The end effector subsystem
   * @param reefPos The lift position for algae removal
   * @return The command sequence
   */
  public static Command RemoveAlgae(
      Drive drive, Lift lift, EndEffector endEffector, double reefPos) {
    return Commands.sequence(
            // Step 1: Raise lift to position
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting lift to pos")),
            Commands.runOnce(() -> lift.runLiftToPos(reefPos)),

            // Wait for lift to get to position (1 second should be enough)
            Commands.waitSeconds(1),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed lift to pos")),

            // Step 3: Run algae removal motor WHILE driving back 6 inches. It's a lift and pull
            // motion.
            Commands.runOnce(
                () -> Logger.recordOutput("Auto/Status", "Starting algae removal with pullback")),
            Commands.parallel(
                // Remove algae with the mechanism
                IntakeCommands.removeAlgaeGently(endEffector),

                // Drive backward slowly while removing algae
                Commands.sequence(
                    // Wait briefly before starting to drive back (let mechanism make contact)
                    Commands.waitSeconds(0.3),
                    // Drive backward slowly (6 inches)
                    DriveCommands.driveDistance(drive, -6.0) // 6 inches backward
                    )),

            // Move the algae remover back to stowed position
            Commands.runOnce(() -> endEffector.moveToStow()),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed algae removal")),

            // Step 4: Lower lift to L1
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting lift to L1")),
            Commands.runOnce(() -> lift.runLiftToPos(LiftConstants.L_ONE)),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed lift to L1")))
        .withName("RemoveAlgae");
  }
}
