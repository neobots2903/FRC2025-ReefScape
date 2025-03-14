package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.RampMechanism;
import frc.robot.subsystems.lift.Lift;
import org.littletonrobotics.junction.Logger;

/** Contains simple autonomous routines for the robot. */
public class AutoCommands {

  private AutoCommands() {
    // Utility class - prevent instantiation
  }

  // Move the timeout constant to class level so it's accessible to both the command and
  // withTimeout()
  private static final double INTAKE_TIMEOUT = 5.0; // Max time to run intake

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
            // Step 1: Raise lift to position with coral safety checks
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting lift to L4")),
            LiftCommands.positionCoralAndLift(endEffector, lift, Lift.LiftPosition.LEVEL_FOUR),

            // No need to wait extra time - safeCoralLift already waits for position to be reached
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed lift to L4")),

            // Step 2: Shoot the game piece
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting to shoot piece")),
            IntakeCommands.shootGamePiece(endEffector),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed shooting piece")),

            // Step 3: Lower lift to L2 with coral safety checks
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Starting lift to L2")),
            // Use safeCoralLift for lowering as well
            LiftCommands.safeCoralLift(endEffector, lift, Lift.LiftPosition.LEVEL_TWO),
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Completed lift to L2")))
        .withName("ScoreCoral");
  }

  /*
   *              _______ BARGE SIDE _______
   *             /                          \
   *            /            L2              \
   *           /   L3                    L3   \
   * LEFT SIDE |                              | RIGHT SIDE
   *           \   L2                    L2   /
   *            \            L3              /
   *             \______ DRIVER SIDE _______/
   *
   */

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

  /**
   * Creates a command that automatically intakes a coral piece. The command ensures the ramp is in
   * intake position and the lift is at the bottom before running the intake until a piece is
   * detected.
   *
   * @param lift The lift subsystem
   * @param endEffector The end effector subsystem
   * @param ramp The ramp mechanism subsystem
   * @return The command sequence
   */
  public static Command IntakeCoral(Lift lift, EndEffector endEffector, RampMechanism ramp) {
    return Commands.sequence(
            // Step 1: Make sure ramp is in intake position and algae mechanism is stowed
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Setting up for intake")),
            Commands.runOnce(() -> ramp.moveToIntakePosition()),
            Commands.runOnce(() -> endEffector.moveToStow()),

            // Step 2: Move lift to bottom position with safety checks
            Commands.runOnce(() -> Logger.recordOutput("Auto/Status", "Moving lift to bottom")),
            LiftCommands.safeCoralLift(endEffector, lift, Lift.LiftPosition.BOTTOM),

            // Step 3: Wait briefly to ensure everything is in position
            Commands.waitSeconds(0.25),
            Commands.runOnce(
                () -> Logger.recordOutput("Auto/Status", "Setup complete, starting intake")),

            // Step 4: Run intake until piece is detected
            IntakeCommands.captureGamePiece(endEffector),

            // Step 5: If piece was detected, run capture to position it properly
            Commands.runOnce(
                () -> Logger.recordOutput("Auto/Status", "Moving piece to front position")))
        .withName("AutoIntake");
  }
}
