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

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*; // Move these to actual constants file

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.vision.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

// later.

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
  private SwerveDriveSimulation driveSimulation = null;
  private final Vision vision;
  private Lift lift;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController =
      new CommandXboxController(1); // Controller for driver.

  // Subsystems
  // ClimbSubsystem climb = new ClimbSubsystem(); // Subsystem for climb.
  // RampMechanism ramp = new RampMechanism(); // System for ramp control.
  // Lift lift = new Lift(operatorController); // Subsystem for the lift control.
  // EndEffector endEffector = new EndEffector(); // Subsystem to control the end effector.

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
                new ModuleIOTalonFX(TunerConstants.BackRight),
                (robotPose) -> {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));
        lift = Lift.getInstance();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (robotPose) -> {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> driverController.getLeftY(),
            () -> driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Switch to robot-relative drive when a is held (for vision)
    driverController // Assign to paddle probably?
        .a()
        .whileTrue(
            DriveCommands.joystickDriveRobotRelative(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    double distanceOffset = Units.inchesToMeters(9);

    // Auto align to right of reef tag
    driverController
        .rightBumper()
        .whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> vision.toggleLock(0)),
                new DriveToPose(
                    drive,
                    () ->
                        calculateOffsetFromCenter(
                            vision.getClosestTagPose(0), distanceOffset, true),
                    () -> drive.getPose())))
        .onFalse(new InstantCommand(() -> vision.toggleLock(0)));

    // Auto align to left of reef tag
    driverController
        .leftBumper()
        .whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> vision.toggleLock(0)),
                new DriveToPose(
                    drive,
                    () ->
                        calculateOffsetFromCenter(
                            vision.getClosestTagPose(0), distanceOffset, false),
                    () -> drive.getPose())))
        .onFalse(new InstantCommand(() -> vision.toggleLock(0)));

    // Auto align to coral station tag
    driverController
        .y()
        .whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> vision.toggleLock(1)),
                new DriveToPose(drive, () -> vision.getClosestTagPose(1), () -> drive.getPose())))
        .onFalse(new InstantCommand(() -> vision.toggleLock(1)));

    final Runnable resetOdometry =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
            : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

    driverController.b().onTrue(Commands.runOnce(resetOdometry).ignoringDisable(true));

    operatorController.povUp().onTrue(Commands.runOnce(() -> lift.setPosition(Lift.SetPoint.L1)));
    operatorController
        .povDown()
        .onTrue(Commands.runOnce(() -> lift.setPosition(Lift.SetPoint.BOTTOM)));
  }

  public Pose2d calculateOffsetFromCenter(
      Pose2d tagPose, double distanceFromCenter, boolean moveRight) {
    // Convert tag heading to radians
    double tagRadians = tagPose.getRotation().getRadians();

    // Define the offset angle (Right = -90 degrees, Left = +90 degrees)
    double offsetAngle = moveRight ? Math.PI / 2 : -Math.PI / 2;

    // Calculate the offset relative to the tag's orientation
    double xOffset =
        distanceFromCenter
            * (Math.cos(tagRadians) * Math.cos(offsetAngle)
                - Math.sin(tagRadians) * Math.sin(offsetAngle));
    double yOffset =
        distanceFromCenter
            * (Math.sin(tagRadians) * Math.cos(offsetAngle)
                + Math.cos(tagRadians) * Math.sin(offsetAngle));

    // Compute the final target pose in field coordinates
    double finalX = tagPose.getX() + xOffset;
    double finalY = tagPose.getY() + yOffset;

    // Ensure the robot faces the tag (by rotating 180 degrees from the tag's heading)
    Pose2d finalPose =
        new Pose2d(
            new Translation2d(finalX, finalY),
            new Rotation2d(Math.toRadians(tagPose.getRotation().getDegrees() + 180)));

    Logger.recordOutput("AutoAlign/TagPose", tagPose);
    Logger.recordOutput("AutoAlign/Offset", new Pose2d(xOffset, yOffset, new Rotation2d()));
    Logger.recordOutput("AutoAlign/FinalOffset", finalPose);

    // Return the final target pose
    return finalPose;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
