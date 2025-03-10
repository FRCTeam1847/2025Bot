package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Levels;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ManipulatorSubsystem extends SubsystemBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private final LoggedMechanism2d combinedMechanism2d;
  private final LoggedMechanismRoot2d baseRoot;
  private final LoggedMechanismLigament2d elevatorLigament;

  private static final int BASE_X = 50;
  private static final int BASE_Y = 10;

  public ManipulatorSubsystem(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.intakeSubsystem = intakeSubsystem;

    combinedMechanism2d = new LoggedMechanism2d(100, 100);

    // Create the base root for the mechanism visualization
    baseRoot = combinedMechanism2d.getRoot("Base", BASE_X, BASE_Y);

    // Add an elevator ligament (vertical element) to the visualization
    elevatorLigament = baseRoot.append(new LoggedMechanismLigament2d("Elevator", 0, 0));

    SmartDashboard.putData("Combined Mechanism", combinedMechanism2d);
  }

  /**
   * Updates the mechanism visualization using only the elevator height.
   *
   * @param elevatorHeight The height of the elevator.
   */
  public void updateMechanism(double elevatorHeight) {
    // Update the elevator ligament length in the visualization.
    elevatorLigament.setLength(elevatorHeight);

    // Log the manipulator pose and visualization
    Logger.recordOutput("Field/Robot/ManipulatorMechanism", getManipulatorPose3d());
    Logger.recordOutput("Mechanism2d/ManipulatorMechanism", combinedMechanism2d);
  }

  /**
   * Returns the current pose of the manipulator.
   * This now only reflects the elevator's vertical position.
   *
   * @return The Pose3d of the manipulator.
   */
  public Pose3d getManipulatorPose3d() {
    // Conversion factor: inches to meters.
    final double inchesToMeters = 0.0254;

    // Calculate the elevator height in meters.
    double elevatorHeightMeters = (elevatorSubsystem.getTargetHeight() * 2) * inchesToMeters;
    Translation3d elevatorTranslation = new Translation3d(0, 0, elevatorHeightMeters);

    Rotation3d manipulatorRotation = new Rotation3d();

    return new Pose3d(elevatorTranslation, manipulatorRotation);
  }

  /**
   * Sets the elevator to a specified level.
   * Arm commands have been removed.
   *
   * @param level The desired level.
   */
  public void setLevel(Levels level) {
    System.out.println("Setting level to " + level);
    switch (level) {
      case CoralStation:
        elevatorSubsystem.setTargetHeight(1);
        break;
      case L1:
        elevatorSubsystem.setTargetHeight(1.5);
        break;
      case L2:
        elevatorSubsystem.setTargetHeight(6.25);
        break;
      case L3:
        elevatorSubsystem.setTargetHeight(14.75);
        break;
      case L4:
        elevatorSubsystem.setTargetHeight(27);
        break;
      default:
      case Home:
        elevatorSubsystem.setTargetHeight(0.2);
        break;
    }
  }

  public Command ScoreAtLevelCommand(Levels level) {
    System.out.println("Command Started");
    return new SequentialCommandGroup(
        // Step 1: Intake until coral is detected
        new InstantCommand(() -> intakeSubsystem.intake()), // Start intaking
        new WaitUntilCommand(this::hasCoral), // Wait until we detect the coral
        new InstantCommand(() -> intakeSubsystem.stopIntake()), // Stop intake once we have coral

        // Step 2: Move to the scoring level
        new InstantCommand(() -> setLevel(level)), // Set the target level
        new WaitUntilCommand(this::isAtHeight), // Wait until the elevator reaches the target height
        new WaitCommand(0.15), // wait for a bit

        // Step 3: Score the coral
        new InstantCommand(() -> intakeSubsystem.release()), // Release the coral
        new WaitCommand(0.15), // wait for a bit before continuing
        new WaitUntilCommand(() -> !hasCoral()), // Wait until the coral is no longer detected
        new InstantCommand(() -> intakeSubsystem.stopIntake()), // Stop intake once we have coral

        // Step 4: Return to home position
        new InstantCommand(() -> setLevel(Levels.Home)) // Move back to home position
    ).finallyDo((interrupted) -> {
      System.out.println("ScoreAtLevelCommand Ended. Stopping Intake.");
      intakeSubsystem.stopIntake();
    });

  }

  public Command releaseCommand() {
    return new InstantCommand(() -> intakeSubsystem.release());
  }

  public Command intakeCommand() {
    return intakeSubsystem.intakeCommand();
  }

  public Command intakeStopCommand() {
    return new InstantCommand(() -> intakeSubsystem.stopIntake());
  }

  public boolean isAtHeight() {
    return elevatorSubsystem.isAtHeight();
  }

  public boolean hasCoral() {
    return intakeSubsystem.isSensorTriggered();
  }

  @Override
  public void periodic() {
    double elevatorHeight = elevatorSubsystem.getTargetHeight();
    updateMechanism(elevatorHeight);
  }
}