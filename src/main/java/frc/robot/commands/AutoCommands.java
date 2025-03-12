package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LiftConstants;
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
            Commands.runOnce(() -> lift.runLiftToPos(LiftConstants.L_FOUR)),

            // Wait for lift to get to position (1.5 second should be enough)
            Commands.waitSeconds(1.5),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed lift to L4")),

            // Step 3: Shoot the game piece
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting to shoot piece")),
            new InstantCommand(() -> endEffector.runMotors()), // Run motors to shoot coral
            Commands.waitSeconds(1.25), // Wait for action to complete
            new InstantCommand(() -> endEffector.stop()), // Stop motors
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed shooting piece")),

            // Step 4: Lower lift to L1
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting lift to L1")),
            Commands.runOnce(() -> lift.runLiftToPos(LiftConstants.L_ONE)),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed lift to L1")))
        .withName("ScoreCoral");
  }

  /**
   * Creates a command that removes algae from a reef by raising the lift to the specified position,
   * running the algae removal motor, and then lowering the lift.
   *
   * @param lift The lift subsystem
   * @param endEffector The end effector subsystem
   * @param reefPos The lift position for algae removal
   * @return The command sequence
   */
  public static Command RemoveAlgae(Lift lift, EndEffector endEffector, double reefPos) {
    return Commands.sequence(
            // Step 1: Raise lift to position
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting lift to pos")),
            Commands.runOnce(() -> lift.runLiftToPos(reefPos)),

            // Wait for lift to get to position (1 second should be enough)
            Commands.waitSeconds(1),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed lift to pos")),

            // Step 3: Run algae removal motor
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting algae removal")),
            new InstantCommand(() -> endEffector.setAlgaeMotorSpeed(0.75)), // Start algae removal
            Commands.waitSeconds(1.25), // Wait for action to complete
            new InstantCommand(() -> endEffector.setAlgaeMotorSpeed(0)), // Stop algae removal
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed algae removal")),

            // Step 4: Lower lift to L1
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting lift to L1")),
            Commands.runOnce(() -> lift.runLiftToPos(LiftConstants.L_ONE)),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed lift to L1")))
        .withName("RemoveAlgae");
  }
}
