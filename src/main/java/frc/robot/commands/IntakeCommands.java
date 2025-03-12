package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.lift.Lift;
import org.littletonrobotics.junction.Logger;

public class IntakeCommands {

  /**
   * Creates a command that captures a game piece from the back of the robot and moves it to the
   * front position for scoring. The command runs the motors until the front sensor is triggered and
   * the back sensor is clear.
   *
   * @param endEffector The end effector subsystem to use
   * @return The command
   */
  public static Command captureGamePiece(EndEffector endEffector) {
    return new Command() {
      @Override
      public void initialize() {
        Logger.recordOutput("Commands/CaptureGamePiece", "Started");
        // Run motors to move piece from back to front
        endEffector.runMotors(0.75);
      }

      @Override
      public void execute() {
        // Motors keep running at the speed set in initialize()
        // Log sensor states for debugging
        Logger.recordOutput(
            "Commands/CaptureGamePiece/FrontSensor", endEffector.isFrontSensorTriggered());
        Logger.recordOutput(
            "Commands/CaptureGamePiece/BackSensor", endEffector.isBackSensorTriggered());
      }

      @Override
      public boolean isFinished() {
        // Finish when:
        // 1. Front sensor IS triggered (piece has reached front position)
        // 2. Back sensor is NOT triggered (piece has cleared the back)
        boolean frontSensorTriggered = endEffector.isFrontSensorTriggered();
        boolean backSensorClear = !endEffector.isBackSensorTriggered();

        return frontSensorTriggered && backSensorClear;
      }

      @Override
      public void end(boolean interrupted) {
        // Stop motors when command ends
        endEffector.stop();
        Logger.recordOutput(
            "Commands/CaptureGamePiece", "Ended - " + (interrupted ? "Interrupted" : "Completed"));
      }
    }.withName("CaptureGamePiece").withTimeout(3.0); // Timeout after 3 seconds for safety
  }

  /**
   * Creates a command that shoots a game piece out the front of the robot. The command will run
   * until the piece is no longer detected by either sensor.
   *
   * @param endEffector The end effector subsystem to use
   * @return The command
   */
  public static Command shootGamePiece(EndEffector endEffector) {
    return new Command() {
      private final Timer exitTimer = new Timer();
      private boolean pieceDetected = false;
      private boolean timerRunning = false;
      private static final double EXIT_CONFIRMATION_TIME = 0.1; // Time to confirm piece has exited

      @Override
      public void initialize() {
        Logger.recordOutput("Commands/ShootGamePiece", "Started");
        // Run motors at full speed to shoot piece out the front
        endEffector.runMotors(1.0);
        exitTimer.stop();
        exitTimer.reset();
        timerRunning = false;
        pieceDetected = endEffector.isFrontSensorTriggered() || endEffector.isBackSensorTriggered();
      }

      @Override
      public void execute() {
        boolean currentlyDetected =
            endEffector.isFrontSensorTriggered() || endEffector.isBackSensorTriggered();

        // If we had a piece but now don't detect it, start exit timer
        if (pieceDetected && !currentlyDetected) {
          if (!timerRunning) {
            exitTimer.reset();
            exitTimer.start();
            timerRunning = true;
          }
        } else if (currentlyDetected) {
          // Reset timer if piece is detected again
          exitTimer.stop();
          exitTimer.reset();
          timerRunning = false;
        }

        pieceDetected = currentlyDetected;
      }

      @Override
      public boolean isFinished() {
        // Finish when piece is no longer detected by either sensor for a brief period
        // This confirms the piece has fully exited
        return timerRunning && exitTimer.hasElapsed(EXIT_CONFIRMATION_TIME);
      }

      @Override
      public void end(boolean interrupted) {
        // Stop motors when command ends
        endEffector.stop();
        exitTimer.stop();
        timerRunning = false;
        Logger.recordOutput(
            "Commands/ShootGamePiece", "Ended - " + (interrupted ? "Interrupted" : "Completed"));
      }
    }.withName("ShootGamePiece").withTimeout(2.0); // Timeout after 2 seconds for safety
  }

  /**
   * Creates a command that prepares the game piece for scoring by ensuring it's properly
   * positioned. This command will gently pulse the motors until the piece is at the front sensor
   * but not at the back.
   *
   * @param endEffector The end effector subsystem to use
   * @return The command
   */
  public static Command prepareForScoring(EndEffector endEffector) {
    return new Command() {
      private static final double PULSE_SPEED = 0.3; // Gentler speed for positioning
      private static final double PULSE_DURATION = 0.1; // Time for each pulse
      private final Timer pulseTimer = new Timer();
      private boolean isPulsing = false;

      @Override
      public void initialize() {
        Logger.recordOutput("Commands/PrepareForScoring", "Started");
        pulseTimer.stop();
        pulseTimer.reset();
        isPulsing = false;
      }

      @Override
      public void execute() {
        boolean frontSensorTriggered = endEffector.isFrontSensorTriggered();
        boolean backSensorClear = !endEffector.isBackSensorTriggered();

        // Ideal position: Front triggered, back clear
        if (frontSensorTriggered && backSensorClear) {
          endEffector.stop();
          isPulsing = false;
        }
        // If piece is too far back (back sensor triggered), pulse forward
        else if (!backSensorClear) {
          if (!isPulsing) {
            endEffector.runMotors(PULSE_SPEED); // Pulse motors to move piece forward
            pulseTimer.reset();
            pulseTimer.start();
            isPulsing = true;
          } else if (pulseTimer.hasElapsed(PULSE_DURATION)) {
            endEffector.stop();
            isPulsing = false;
          }
        }
        // If piece is not detected at front, pulse forward
        else if (!frontSensorTriggered) {
          if (!isPulsing) {
            endEffector.runMotors(PULSE_SPEED); // Pulse motors to move piece forward
            pulseTimer.reset();
            pulseTimer.start();
            isPulsing = true;
          } else if (pulseTimer.hasElapsed(PULSE_DURATION)) {
            endEffector.stop();
            isPulsing = false;
          }
        }
      }

      @Override
      public boolean isFinished() {
        // Finish when piece is in ideal position and we're not in the middle of a pulse
        boolean frontSensorTriggered = endEffector.isFrontSensorTriggered();
        boolean backSensorClear = !endEffector.isBackSensorTriggered();

        return frontSensorTriggered && backSensorClear && !isPulsing;
      }

      @Override
      public void end(boolean interrupted) {
        endEffector.stop();
        pulseTimer.stop();
        Logger.recordOutput(
            "Commands/PrepareForScoring", "Ended - " + (interrupted ? "Interrupted" : "Completed"));
      }
    }.withName("PrepareForScoring").withTimeout(3.0); // Safety timeout
  }

  /**
   * Creates a command that ensures the lift is safe to move by checking that the back sensor is
   * clear. This prevents crushing a game piece with the lift.
   *
   * @param endEffector The end effector subsystem
   * @param lift The lift subsystem
   * @param liftPosition The target position for the lift
   * @return A command that only moves the lift when safe
   */
  public static Command safeLiftToPosition(
      EndEffector endEffector, Lift lift, double liftPosition) {
    return Commands.either(
            // If back sensor is clear, move lift to position
            Commands.runOnce(() -> lift.runLiftToPos(liftPosition)),

            // If back sensor is triggered, first prepare piece then move lift
            Commands.sequence(
                prepareForScoring(endEffector),
                Commands.runOnce(() -> lift.runLiftToPos(liftPosition))),

            // Condition to check if back sensor is clear
            () -> !endEffector.isBackSensorTriggered())
        .withName("SafeLiftToPosition");
  }

  /**
   * Creates a command that gently removes algae with a slow approach followed by position control.
   * Uses current detection to identify when contact is made with the algae.
   *
   * @param endEffector The end effector subsystem to use
   * @return The command sequence
   */
  public static Command removeAlgaeGently(EndEffector endEffector) {
    return new Command() {
      // Constants
      private static final double APPROACH_SPEED = 0.2; // Slow approach speed
      private static final double CURRENT_THRESHOLD = 10.0; // Amps to detect contact
      private static final double TARGET_POSITION = 125.0; // Target position in degrees

      // State variables
      private boolean contactDetected = false;
      private Timer contactDebounceTimer = new Timer();
      private boolean timerRunning = false;

      @Override
      public void initialize() {
        Logger.recordOutput("Commands/AlgaeRemoval", "Starting gentle approach");
        contactDetected = false;
        endEffector.setAlgaeMotorSpeed(APPROACH_SPEED); // Start with slow constant speed
        contactDebounceTimer.stop();
        contactDebounceTimer.reset();
        timerRunning = false;
      }

      @Override
      public void execute() {
        // Monitor current and position
        double current = endEffector.getAlgaeMotorCurrent();
        Logger.recordOutput("Commands/AlgaeRemoval/Current", current);

        if (!contactDetected && current > CURRENT_THRESHOLD) {
          // Start debounce timer when contact initially detected
          if (!timerRunning) {
            contactDebounceTimer.reset();
            contactDebounceTimer.start();
            timerRunning = true;
          }

          // If the current remains high for a short period, confirm contact
          if (contactDebounceTimer.hasElapsed(0.05)) {
            // Contact confirmed, switch to position control
            contactDetected = true;
            endEffector.setAlgaeMotorSpeed(0); // Stop the motor briefly
            Logger.recordOutput(
                "Commands/AlgaeRemoval", "Contact detected, switching to position control");
            endEffector.setPosition(TARGET_POSITION); // Use position control to final position
          }
        } else if (!contactDetected && timerRunning && current <= CURRENT_THRESHOLD) {
          // Reset debounce timer if current drops below threshold during debounce
          contactDebounceTimer.reset();
          contactDebounceTimer.stop();
          timerRunning = false;
        }
      }

      @Override
      public boolean isFinished() {
        if (!contactDetected) {
          return false; // Not finished until contact is detected
        }

        // After contact, finish when position is approximately reached
        double currentPos = endEffector.getAlgaePosition();
        double error = Math.abs(TARGET_POSITION - currentPos);
        return error < 5.0; // 5 degree tolerance
      }

      @Override
      public void end(boolean interrupted) {
        if (interrupted) {
          // If interrupted, stop the motor
          endEffector.setAlgaeMotorSpeed(0);
          Logger.recordOutput("Commands/AlgaeRemoval", "Interrupted");
        } else {
          Logger.recordOutput("Commands/AlgaeRemoval", "Completed successfully");
        }
      }
    }.withName("RemoveAlgaeGently").withTimeout(3.0); // Safety timeout
  }
}
