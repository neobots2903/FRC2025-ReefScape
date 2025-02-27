// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.RampMechanismConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.SimpleAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.RampMechanism;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.endEffector.EndEffector;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/* NOTES */
/*
 * - I'd probably instantiate the subsystems inside of the RobotContainer constructor instead of
 *   having them be fields. This way, you can use the RobotContainer to determine which
 *   subsystems to instantiate based on the current mode. Right now, they would all move under "REAL".
 */

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController =
      new CommandXboxController(1); // Controller for driver.

  // Subsystems
  ClimbSubsystem climb = new ClimbSubsystem(); // Subsystem for climb.
  RampMechanism ramp = new RampMechanism(); // System for ramp control.
  Lift lift = new Lift(); // Subsystem for the lift control.
  EndEffector endEffector = new EndEffector(); // Subsystem to control the end effector.

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Lift positions array should be double, not int
  private final double[] liftPositions = new double[]{
      LiftConstants.BOTTOM, 
      LiftConstants.L_ONE, 
      LiftConstants.L_TWO, 
      LiftConstants.L_THREE, 
      LiftConstants.L_FOUR
  };
  private int currentLiftPositionIndex = 0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    
    // Add our simple autonomous routines
    autoChooser.addOption( // Drive forward, raise lift, outtake (BACK BUMPER ON START LINE)
        "Simple Coral Auto", SimpleAuto.simpleCoral(drive, lift, endEffector));
    autoChooser.addOption( // Drive forward 5 feet
    "Drive Forward 5 feet", DriveCommands.driveDistance(drive, 60.0));
    autoChooser.addOption( // Drive forward 1 foot
        "Drive Forward 1 foot", DriveCommands.driveDistance(drive, 12.0));

    // // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureDriverControls();
    configureOperatorControls();
  }

  /**
   * Configure controls for the main driver
   * Driver responsibilities:
   * - Robot movement and orientation
   * - Emergency functions
   */
  private void configureDriverControls() {
    // Left stick: Translation control (forward/backward with Y-axis, left/right with X-axis)
    // Right stick: Rotation control (X-axis controls turning)
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> driverController.getLeftY(),   // Forward/backward
            () -> driverController.getLeftX(),   // Left/right
            () -> -driverController.getRightX()) // Rotation
        .withName("Default Drive Command"));

    // X button: Lock wheels in X pattern to prevent movement
    driverController.x()
        .onTrue(Commands.runOnce(drive::stopWithX, drive)
        .withName("Lock Wheels X-Pattern"));

    // B button: Reset gyro to 0 degrees (forward direction)
    driverController.b()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive)
            .ignoringDisable(true)
            .withName("Reset Gyro"));
  }

  /**
   * Configure controls for the operator
   * Operator responsibilities:
   * - Lift control
   * - Game piece manipulation
   * - Mechanism control
   */
  private void configureOperatorControls() {
    // === LIFT CONTROLS (D-PAD) ===
    // D-Pad Up: Move lift directly to highest position (L4)
    operatorController.povUp()
        .onTrue(Commands.runOnce(() -> {
            currentLiftPositionIndex = liftPositions.length - 1;
            lift.runLiftToPos(liftPositions[currentLiftPositionIndex]);
        })
        .withName("Lift to Top Position (L4)"));
    
    // D-Pad Down: Move lift directly to bottom position
    operatorController.povDown()
        .onTrue(Commands.runOnce(() -> {
            currentLiftPositionIndex = 0;
            lift.runLiftToPos(liftPositions[currentLiftPositionIndex]);
        })
        .withName("Lift to Bottom Position"));
    
    // D-Pad Left: Move lift down one position
    operatorController.povLeft()
        .onTrue(Commands.runOnce(() -> {
            currentLiftPositionIndex = Math.max(currentLiftPositionIndex - 1, 0);
            lift.runLiftToPos(liftPositions[currentLiftPositionIndex]);
        })
        .withName("Lift Position Down One"));
    
    // D-Pad Right: Move lift up one position
    operatorController.povRight()
        .onTrue(Commands.runOnce(() -> {
            currentLiftPositionIndex = Math.min(currentLiftPositionIndex + 1, liftPositions.length - 1);
            lift.runLiftToPos(liftPositions[currentLiftPositionIndex]);
        })
        .withName("Lift Position Up One"));

    // === RAMP MECHANISM CONTROLS (BUMPERS) ===
    // Left Bumper: Move ramp to intake position
    operatorController.leftBumper()
        .onTrue(Commands.runOnce(() -> ramp.moveToIntakePosition())
        .withName("Ramp to Intake Position"));
    
    // Right Bumper: Move ramp to hang position
    operatorController.rightBumper()
        .onTrue(Commands.runOnce(() -> ramp.moveToHangPosition())
        .withName("Ramp to Hang Position"));

    // === CLIMB CONTROLS (TRIGGERS) ===
    // Left Trigger: Move claws to open position
    operatorController.leftTrigger(0.25) // 0.25 threshold for activation
        .onTrue(Commands.runOnce(() -> climb.moveToOpenPosition())
        .withName("Claws Open"));
    
    // Right Trigger: Move claws to grip position
    operatorController.rightTrigger(0.25) // 0.25 threshold for activation
        .onTrue(Commands.runOnce(() -> climb.moveToGripPosition())
        .withName("Claws Grip"));

    // === GAME PIECE CONTROLS (FACE BUTTONS) ===
    // A button: Intake game piece (pulls piece in until properly positioned)
    operatorController.a()
        .onTrue(IntakeCommands.intakeGamePiece(endEffector)
        .withName("Intake Game Piece"));
    
    // B button: Outtake game piece (pushes piece out completely)
    operatorController.b()
        .onTrue(IntakeCommands.outtakeGamePiece(endEffector)
        .withName("Outtake Game Piece"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
