// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Levels;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
        // File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),
        // "swerve");
        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve/kraken"));
        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(false);
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem(elevatorSubsystem,
                        intakeSubsystem);
        private final SendableChooser<Command> autoChooser;
        // Replace with CommandPS4Controller or CommandJoystick if needed
        private final CommandPS5Controller driverXbox = new CommandPS5Controller(0);

        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative
         * input stream.
         */
        // SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
        // .withControllerHeadingAxis(driverXbox::getRightX,
        // driverXbox::getRightY)
        // .headingWhile(true);
        // Derive the heading axis with math!
        // SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
        // .withControllerHeadingAxis(() -> Math.sin(
        // driverXbox.getRawAxis(
        // 2) * Math.PI)
        // * (Math.PI * 2),
        // () -> Math.cos(
        // driverXbox.getRawAxis(
        // 2) * Math.PI)
        // *
        // (Math.PI * 2))
        // .headingWhile(true);

        //// DRIVE Setup
        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverXbox.getLeftY() * -1,
                        () -> driverXbox.getLeftX() * -1)
                        .withControllerRotationAxis(driverXbox::getRightX)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -driverXbox.getLeftY(),
                        () -> -driverXbox.getLeftX())
                        .withControllerRotationAxis(() -> driverXbox.getRightX())
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                configureDefaultCommand();
                registerNamedCommands();

                drivebase.setupPathPlanner();
                autoChooser = AutoBuilder.buildAutoChooser();
                configureBindings();
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        private void configureDefaultCommand() {
                manipulatorSubsystem.setDefaultCommand(
                                new InstantCommand(
                                                () -> {
                                                },
                                                manipulatorSubsystem));
        }

        private void registerNamedCommands() {
                NamedCommands.registerCommand(
                                "Home",
                                new InstantCommand(
                                                () -> manipulatorSubsystem.setLevel(Levels.Home),
                                                manipulatorSubsystem));
                NamedCommands.registerCommand(
                                "CoralStation",
                                new InstantCommand(
                                                () -> manipulatorSubsystem.setLevel(Levels.CoralStation),
                                                manipulatorSubsystem));
                NamedCommands.registerCommand(
                                "L1",
                                new InstantCommand(
                                                () -> manipulatorSubsystem.setLevel(Levels.L1),
                                                manipulatorSubsystem));
                NamedCommands.registerCommand(
                                "L2",
                                new InstantCommand(
                                                () -> manipulatorSubsystem.setLevel(Levels.L2),
                                                manipulatorSubsystem));
                NamedCommands.registerCommand(
                                "L3",
                                new InstantCommand(
                                                () -> manipulatorSubsystem.setLevel(Levels.L3),
                                                manipulatorSubsystem));
                NamedCommands.registerCommand(
                                "L4",
                                new InstantCommand(
                                                () -> manipulatorSubsystem.setLevel(Levels.L4),
                                                manipulatorSubsystem));
                NamedCommands.registerCommand(
                                "Intake", manipulatorSubsystem.intakeCommand());
                NamedCommands.registerCommand(
                                "Release", manipulatorSubsystem.releaseCommand());
                NamedCommands.registerCommand(
                                "IntakeStop", manipulatorSubsystem.intakeStopCommand());

        }

        private void configureBindings() {
                // Command driveFieldOrientedDirectAngle =
                // drivebase.driveFieldOriented(driveDirectAngle);
                // Command driveSetpointGenSim =
                // drivebase.driveWithSetpointGeneratorFieldRelative(
                // driveDirectAngleSim);
                // Command driveSetpointGen =
                // drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
                // Command driveFieldOrientedDirectAngleSim =
                // drivebase.driveFieldOriented(driveDirectAngleSim);

                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);

                if (RobotBase.isSimulation()) {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocitySim);
                } else {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
                }

                // if (Robot.isSimulation()) {
                // driverXbox.PS().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new
                // Pose2d(3, 3, new Rotation2d()))));
                // driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

                // }

                driverXbox.R2().whileTrue(NamedCommands.getCommand("Intake"))
                                .onFalse(NamedCommands.getCommand("IntakeStop"));
                driverXbox.L2().whileTrue(NamedCommands.getCommand("Release"))
                                .onFalse(NamedCommands.getCommand("IntakeStop"));

                driverXbox.R1().onTrue(NamedCommands.getCommand("CoralStation"));
                driverXbox.L1().onTrue(NamedCommands.getCommand("Home"));

                driverXbox.cross().onTrue(NamedCommands.getCommand("L1"));
                driverXbox.circle().onTrue(NamedCommands.getCommand("L2"));
                driverXbox.triangle().onTrue(NamedCommands.getCommand("L4"));
                driverXbox.square().onTrue(NamedCommands.getCommand("L3"));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return autoChooser.getSelected();// drivebase.getAutonomousCommand("New Auto");
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
}
