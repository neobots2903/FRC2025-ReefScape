package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.endEffector.EndEffector;
import org.littletonrobotics.junction.Logger;

/**
 * Contains simple autonomous routines for the robot.
 */
public class SimpleAuto {

  private SimpleAuto() {
    // Utility class - prevent instantiation
  }
  
  /**
   * Creates a simple autonomous command that:
   * 1. Drives forward 5 feet
   * 2. Raises the lift to L1 position
   * 3. Outtakes the game piece
   * 
   * @param drive The drive subsystem
   * @param lift The lift subsystem
   * @param endEffector The end effector subsystem
   * @return The autonomous command sequence
   */
  public static Command simpleCoral(Drive drive, Lift lift, EndEffector endEffector) {
    return Commands.sequence(
        // Step 1: Drive forward 5 feet (60 inches)
        Commands.runOnce(() -> Logger.recordOutput("Auto", "Starting drive forward")),
        DriveCommands.driveDistance(drive, 60.0),
        Commands.runOnce(() -> Logger.recordOutput("Auto", "Completed drive forward")),
        
        // Step 2: Raise lift to L1 position
        Commands.runOnce(() -> Logger.recordOutput("Auto", "Starting lift to L1")),
        Commands.runOnce(() -> lift.runLiftToPos(LiftConstants.L_ONE)),
        
        // Wait for lift to get to position (1 second should be enough)
        Commands.waitSeconds(1.0),
        Commands.runOnce(() -> Logger.recordOutput("Auto", "Completed lift to L1")),
        
        // Step 3: Outtake the game piece
        Commands.runOnce(() -> Logger.recordOutput("Auto", "Starting outtake")),
        IntakeCommands.outtakeGamePiece(endEffector),
        Commands.runOnce(() -> Logger.recordOutput("Auto", "Completed outtake"))
    ).withName("Simple Coral Auto");
  }
}
