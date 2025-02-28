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
        endEffector.revStop();
        // endEffector.stop();
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
   * Creates a command that outtakes a game piece completely. The command runs the outtake wheels
   * until both limit switches are NOT pressed, indicating the game piece has been fully ejected.
   *
   * @param endEffector The end effector subsystem to use
   * @return The command
   */
  public static Command outtakeGamePiece(EndEffector endEffector) {
    return new Command() {
      @Override
      public void initialize() {
        Logger.recordOutput("Commands/OuttakeGamePiece", "Started");
        // Start outtake wheels
        endEffector.intake();
      }

      @Override
      public boolean isFinished() {
        // Finish when both switches are clear (piece fully ejected)
        boolean bothSwitchesClear =
            !endEffector.isIntakeLimitSwitchTriggered()
                && !endEffector.isOuttakeLimitSwitchTriggered();

        Logger.recordOutput("Commands/OuttakeGamePiece/BothSwitchesClear", bothSwitchesClear);

        return bothSwitchesClear;
      }

      @Override
      public void end(boolean interrupted) {
        // Stop motors when command ends
        endEffector.stop();
        Logger.recordOutput(
            "Commands/OuttakeGamePiece", "Ended - " + (interrupted ? "Interrupted" : "Completed"));
      }

      @Override
      public String getName() {
        return "OuttakeGamePiece";
      }
    }.withName("OuttakeGamePiece").withTimeout(2.0); // Timeout after 2 seconds for safety
  }
}
