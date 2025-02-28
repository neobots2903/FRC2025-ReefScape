package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endEffector.EndEffector;
import org.littletonrobotics.junction.Logger;

public class IntakeCommands {

  /**
   * Creates a command that intakes a game piece until it reaches the outtake position. The command
   * runs the intake wheels until the intake limit switch is NOT pressed AND the outtake limit
   * switch IS pressed, indicating the game piece has moved fully to the outtake position.
   *
   * @param endEffector The end effector subsystem to use
   * @return The command
   */
  public static Command intakeGamePiece(EndEffector endEffector) {
    return new Command() {
      @Override
      public void initialize() {
        Logger.recordOutput("Commands/IntakeGamePiece", "Started");
        // Start intake wheels
        endEffector.intake();
      }

      @Override
      public void execute() {
        // Nothing needed here - motors keep running at the speed set in initialize()
      }

      @Override
      public boolean isFinished() {
        // Finish when piece reaches outtake position:
        // - Intake switch NOT triggered (piece no longer at intake)
        // - Outtake switch IS triggered (piece has reached outtake position)
        boolean intakeSwitchClear = !endEffector.isIntakeLimitSwitchTriggered();
        boolean outtakeSwitchTriggered = endEffector.isOuttakeLimitSwitchTriggered();

        Logger.recordOutput("Commands/IntakeGamePiece/IntakeSwitchClear", intakeSwitchClear);
        Logger.recordOutput(
            "Commands/IntakeGamePiece/OuttakeSwitchTriggered", outtakeSwitchTriggered);

        return intakeSwitchClear && outtakeSwitchTriggered;
      }

      @Override
      public void end(boolean interrupted) {
        // Stop motors when command ends
        endEffector.stop();
        Logger.recordOutput(
            "Commands/IntakeGamePiece", "Ended - " + (interrupted ? "Interrupted" : "Completed"));
      }

      @Override
      public String getName() {
        return "IntakeGamePiece";
      }
    }.withName("IntakeGamePiece").withTimeout(3.0); // Timeout after 3 seconds for safety
  }

  /**
   * Creates a command that runs the intake until a game piece passes completely through the intake limit switch.
   * The sequence is:
   * 1. Start intake
   * 2. Wait for intake limit switch to become active (piece detected)
   * 3. Wait for intake limit switch to become inactive again (piece has passed through)
   * 4. Stop intake
   *
   * @param endEffector The end effector subsystem to use
   * @return The command
   */
  public static Command intakeUntilPiecePassesThrough(EndEffector endEffector) {
    return new Command() {
      // Track whether we've seen the limit switch activate
      private boolean hasDetectedPiece = false;
      
      @Override
      public void initialize() {
        Logger.recordOutput("Commands/IntakeUntilPiecePassesThrough", "Started");
        // Start intake wheels
        endEffector.intake();
        // Reset detection state
        hasDetectedPiece = false;
      }

      @Override
      public void execute() {
        // Update our state if we detect a piece
        if (endEffector.isIntakeLimitSwitchTriggered()) {
          hasDetectedPiece = true;
          Logger.recordOutput("Commands/IntakeUntilPiecePassesThrough", "Piece Detected");
        }
      }

      @Override
      public boolean isFinished() {
        // Finish when:
        // 1. We have detected a piece at some point (hasDetectedPiece is true)
        // 2. AND now the intake switch is clear again (piece has passed through)
        boolean pieceHasPassedThrough = hasDetectedPiece && !endEffector.isIntakeLimitSwitchTriggered();
        
        Logger.recordOutput("Commands/IntakeUntilPiecePassesThrough/HasDetectedPiece", hasDetectedPiece);
        Logger.recordOutput("Commands/IntakeUntilPiecePassesThrough/SwitchClear", !endEffector.isIntakeLimitSwitchTriggered());
        
        return pieceHasPassedThrough;
      }

      @Override
      public void end(boolean interrupted) {
        // Stop motors when command ends
        endEffector.stop();
        Logger.recordOutput(
            "Commands/IntakeUntilPiecePassesThrough", 
            "Ended - " + (interrupted ? "Interrupted" : "Completed"));
      }

      @Override
      public String getName() {
        return "IntakeUntilPiecePassesThrough";
      }
    }.withName("IntakeUntilPiecePassesThrough").withTimeout(5.0); // Timeout after 5 seconds for safety
  }
}
