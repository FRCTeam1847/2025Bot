// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Levels;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
//   private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
//   private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem(elevatorSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS5Controller driverXbox = new CommandPS5Controller(0);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
//   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
//       () -> driverXbox.getLeftY() * -1,
//       () -> driverXbox.getLeftX() * -1)
//       .withControllerRotationAxis(driverXbox::getRightY)
//       .deadband(OperatorConstants.DEADBAND)
//       .scaleTranslation(0.8)
//       .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
//   SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
//       driverXbox::getRightY)
//       .headingWhile(true);

//   SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
//       () -> -driverXbox.getLeftY(),
//       () -> -driverXbox.getLeftX())
//       .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
//       .deadband(OperatorConstants.DEADBAND)
//       .scaleTranslation(0.8)
//       .allianceRelativeControl(true);
//   // Derive the heading axis with math!
//   SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
//       .withControllerHeadingAxis(() -> Math.sin(
//           driverXbox.getRawAxis(
//               2) * Math.PI)
//           * (Math.PI * 2),
//           () -> Math.cos(
//               driverXbox.getRawAxis(
//                   2) * Math.PI)
//               *
//               (Math.PI * 2))
//       .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureDefaultCommand();
    registerNamedCommands();
    configureBindings();
  }

  private void configureDefaultCommand() {
    manipulatorSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
            }, // Do nothing; the subsystem will maintain the last target angle
            manipulatorSubsystem));
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand(
        "CoralStation_Left",
        new RunCommand(
            () -> manipulatorSubsystem.setLevel(Levels.CoralStation, false),
            manipulatorSubsystem));
    NamedCommands.registerCommand(
        "L1_Left",
        new RunCommand(
            () -> manipulatorSubsystem.setLevel(Levels.L1, false),
            manipulatorSubsystem));
    NamedCommands.registerCommand(
        "L2_Left",
        new RunCommand(
            () -> manipulatorSubsystem.setLevel(Levels.L2, false),
            manipulatorSubsystem));
    NamedCommands.registerCommand(
        "L3_Left",
        new RunCommand(
            () -> manipulatorSubsystem.setLevel(Levels.L3, false),
            manipulatorSubsystem));
    NamedCommands.registerCommand(
        "L4_Left",
        new RunCommand(
            () -> manipulatorSubsystem.setLevel(Levels.L4, false),
            manipulatorSubsystem));
    NamedCommands.registerCommand(
        "CoralStation_Right",
        new RunCommand(
            () -> manipulatorSubsystem.setLevel(Levels.CoralStation, true),
            manipulatorSubsystem));
    NamedCommands.registerCommand(
        "L1_Right",
        new RunCommand(
            () -> manipulatorSubsystem.setLevel(Levels.L1, true),
            manipulatorSubsystem));
    NamedCommands.registerCommand(
        "L2_Right",
        new RunCommand(
            () -> manipulatorSubsystem.setLevel(Levels.L2, true),
            manipulatorSubsystem));
    NamedCommands.registerCommand(
        "L3_Right",
        new RunCommand(
            () -> manipulatorSubsystem.setLevel(Levels.L3, true),
            manipulatorSubsystem));
    NamedCommands.registerCommand(
        "L4_Right",
        new RunCommand(
            () -> manipulatorSubsystem.setLevel(Levels.L4, true),
            manipulatorSubsystem));
    NamedCommands.registerCommand(
        "Home",
        new RunCommand(
            () -> manipulatorSubsystem.setLevel(Levels.Home, false),
            manipulatorSubsystem));
  }

  private void configureBindings() {
    // Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    // Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    // Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    // Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);
    // Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);
    // Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(
    //     driveDirectAngleSim);

    // if (RobotBase.isSimulation()) {
    //   drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocitySim);
    // } else {
    //   drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // }

    // if (Robot.isSimulation()) {
    //   driverXbox.PS().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    //   driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    // }


    driverXbox.R1().onTrue(NamedCommands.getCommand("CoralStation_Right"));
    driverXbox.L1().onTrue(NamedCommands.getCommand("CoralStation_Left"));

    driverXbox.cross().onTrue(NamedCommands.getCommand("L1_Right"));
    driverXbox.circle().onTrue(NamedCommands.getCommand("L2_Right"));
    driverXbox.triangle().onTrue(NamedCommands.getCommand("L3_Right"));
    driverXbox.square().onTrue(NamedCommands.getCommand("L4_Right"));

    driverXbox.povDown().onTrue(NamedCommands.getCommand("L1_Left"));
    driverXbox.povRight().onTrue(NamedCommands.getCommand("L2_Left"));
    driverXbox.povUp().onTrue(NamedCommands.getCommand("L3_Left"));
    driverXbox.povLeft().onTrue(NamedCommands.getCommand("L4_Left"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Command() {
        
    }; //drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    //drivebase.setMotorBrake(brake);
  }
}
