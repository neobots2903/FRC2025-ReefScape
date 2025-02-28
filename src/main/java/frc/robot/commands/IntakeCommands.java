package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
   * Creates a command that runs the intake until a game piece passes completely through the intake
   * limit switch, then reverses briefly to pull it back to a secure position.
   *
   * @param endEffector The end effector subsystem to use
   * @return The command
   */
  public static Command intakeUntilPiecePassesThrough(EndEffector endEffector) {
    return new Command() {
      // Track command state - fix local enum declaration (no access modifier)
      enum IntakeState {
        WAITING_FOR_DETECTION,
        WAITING_FOR_PASS_THROUGH,
        REVERSING
      }

      private IntakeState state = IntakeState.WAITING_FOR_DETECTION;

      // Timer for the reversing phase
      private final Timer reverseTimer = new Timer();
      private static final double REVERSE_TIME_SECONDS = 0.15; // Time to run motors in reverse
      private static final double REVERSE_SPEED_FACTOR =
          0.4; // Speed factor for reverse (slower than intake)

      @Override
      public void initialize() {
        Logger.recordOutput("Commands/IntakeUntilPiecePassesThrough", "Started");
        // Start intake wheels
        endEffector.intake();
        // Initial state
        state = IntakeState.WAITING_FOR_DETECTION;
        reverseTimer.stop();
        reverseTimer.reset();
      }

      @Override
      public void execute() {
        switch (state) {
          case WAITING_FOR_DETECTION:
            // Wait for the piece to trigger the limit switch
            if (endEffector.isIntakeLimitSwitchTriggered()) {
              Logger.recordOutput("Commands/IntakeUntilPiecePassesThrough", "Piece Detected");
              state = IntakeState.WAITING_FOR_PASS_THROUGH;
            }
            break;

          case WAITING_FOR_PASS_THROUGH:
            // Wait for the piece to pass through the limit switch
            if (!endEffector.isIntakeLimitSwitchTriggered()) {
              Logger.recordOutput(
                  "Commands/IntakeUntilPiecePassesThrough",
                  "Piece Passed Through - Starting Reverse");
              // Start the reversing phase
              state = IntakeState.REVERSING;
              // Run motors in opposite direction at reduced speed
              endEffector.outtake(REVERSE_SPEED_FACTOR);
              // Start timer for the reversing phase
              reverseTimer.reset();
              reverseTimer.start();
            }
            break;

          case REVERSING:
            // Let the reversing timer run
            break;
        }
      }

      @Override
      public boolean isFinished() {
        // Finish only after the reversing phase completes
        return (state == IntakeState.REVERSING) && reverseTimer.hasElapsed(REVERSE_TIME_SECONDS);
      }

      @Override
      public void end(boolean interrupted) {
        // Stop motors when command ends
        endEffector.stop();
        reverseTimer.stop();
        Logger.recordOutput(
            "Commands/IntakeUntilPiecePassesThrough",
            "Ended - " + (interrupted ? "Interrupted" : "Completed"));
      }

      @Override
      public String getName() {
        return "IntakeUntilPiecePassesThrough";
      }
    }.withName("IntakeUntilPiecePassesThrough")
        .withTimeout(5.0); // Timeout after 5 seconds for safety
  }
}
