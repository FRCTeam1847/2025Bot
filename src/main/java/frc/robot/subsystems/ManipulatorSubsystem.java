package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    final double inchesToMeters = 0.2;

    // Calculate the elevator height in meters.
    double elevatorHeightMeters = elevatorSubsystem.getTargetHeight() * inchesToMeters;
    Translation3d elevatorTranslation = new Translation3d(0, 0, elevatorHeightMeters + 0.55);

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
        elevatorSubsystem.setTargetHeight(0.25);
        break;
      case L1:
        elevatorSubsystem.setTargetHeight(0.6);
        break;
      case L2:
        elevatorSubsystem.setTargetHeight(2.1);
        break;
      case L3:
        elevatorSubsystem.setTargetHeight(4.1);
        break;
      case L4:
        elevatorSubsystem.setTargetHeight(6.85);
        break;
      default:
      case Home:
        elevatorSubsystem.setTargetHeight(0);
        break;
    }
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

  @Override
  public void periodic() {
    double elevatorHeight = elevatorSubsystem.getTargetHeight();
    updateMechanism(elevatorHeight);
  }
}