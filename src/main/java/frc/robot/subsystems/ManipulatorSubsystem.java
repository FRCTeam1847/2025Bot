package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Levels;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ManipulatorSubsystem extends SubsystemBase {
  private final ArmSubsystem armSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private final LoggedMechanism2d combinedMechanism2d;
  private final LoggedMechanismRoot2d baseRoot;
  private final LoggedMechanismLigament2d elevatorLigament;
  private final LoggedMechanismLigament2d armLigament;

  private static final int BASE_X = 50;
  private static final int BASE_Y = 10;

  private static final double releaseDelay = 0.2;
  private static final double intakeMaxDelay = 0.2;

  public ManipulatorSubsystem(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem,
      IntakeSubsystem intakeSubsystem) {
    this.armSubsystem = armSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.intakeSubsystem = intakeSubsystem;

    // Initialize Mechanism2d
    combinedMechanism2d = new LoggedMechanism2d(100, 100);

    // Base root of the mechanism
    baseRoot = combinedMechanism2d.getRoot("Base", BASE_X, BASE_Y);

    // Elevator ligament extending vertically
    elevatorLigament = baseRoot.append(new LoggedMechanismLigament2d("Elevator", 0, 90));
    // Arm ligament extending from the top of the elevator
    armLigament = elevatorLigament.append(new LoggedMechanismLigament2d("Arm", 20, 90));

    SmartDashboard.putData("Combined Mechanism", combinedMechanism2d);
  }

  public void updateMechanism(double elevatorHeight, double armAngle) {
    // Update elevator height
    elevatorLigament.setLength(elevatorHeight);

    // Update arm angle
    armLigament.setAngle(armAngle);
    // Logger.recordOutput("Field/Robot", new Pose3d(drivebase.getPose()));
    Logger.recordOutput("Field/Robot/ManipulatorMechanism", getManipulatorPose3d());
    // Log updates for visualization
    Logger.recordOutput("Mechanism2d/ManipulatorMechanism", combinedMechanism2d);

  }

  public Pose3d getManipulatorPose3d() {
    // Conversion factor: inches to meters.
    final double inchesToMeters = 0.2;

    // --- Elevator Contribution ---
    // Get the elevator height (in inches) and convert to meters.
    double elevatorHeightMeters = elevatorSubsystem.getTargetHeight() * inchesToMeters;
    // Use the elevator's position for the translation. (Here, only Z changes.)
    Translation3d elevatorTranslation = new Translation3d(0, 0, elevatorHeightMeters + .55);

    // --- Arm Contribution (Rotation Only) ---
    // Get the current arm angle (in degrees) and convert to radians.
    double armAngleRad = armSubsystem.getTargetAngle().getRadians();

    // Here we set the pitch (second parameter) to the arm angle.
    // Adjust the axis if your coordinate system differs.
    Rotation3d manipulatorRotation = new Rotation3d(0, armAngleRad, 0);

    // Return the full Pose3d with the elevator translation and the arm's rotation.
    return new Pose3d(elevatorTranslation, manipulatorRotation);
  }

  public void setLevel(Levels level, boolean left) {
    System.out.println("Setting level to " + level + " with left = " + left);
    switch (level) {
      case CoralStation:
        elevatorSubsystem.setTargetHeight(0);
        armSubsystem.setTargetAngle(Rotation2d.fromDegrees(left ? -45 : 45));
        break;
      case L1:
        elevatorSubsystem.setTargetHeight(0.6);
        armSubsystem.setTargetAngle(Rotation2d.fromDegrees(left ? -90 : 90));
        break;
      case L2:
        elevatorSubsystem.setTargetHeight(0.6);
        armSubsystem.setTargetAngle(Rotation2d.fromDegrees(left ? -60 : 60));
        break;
      case L3:
        elevatorSubsystem.setTargetHeight(2.5);
        armSubsystem.setTargetAngle(Rotation2d.fromDegrees(left ? -60 : 60));
        break;
      case L4:
        elevatorSubsystem.setTargetHeight(6.8);
        armSubsystem.setTargetAngle(Rotation2d.fromDegrees(left ? -110 : 90));
        break;
      default:
      case Home:
        elevatorSubsystem.setTargetHeight(0);
        armSubsystem.setTargetAngle(Rotation2d.fromDegrees(0));
        break;
    }
  }

  public Command releaseCommand() {
    return new InstantCommand(
        () -> intakeSubsystem.release());
  }

  public Command intakeCommand() {
    return new InstantCommand(
        () -> intakeSubsystem.intake());
  }

  public Command intakeStopCommand() {
    return new InstantCommand(
        () -> intakeSubsystem.stopIntake());
  }

  @Override
  public void periodic() {
    double elevatorHeight = elevatorSubsystem.getTargetHeight();
    double armAngle = armSubsystem.getTargetAngle().getDegrees();

    // Update the visualization
    updateMechanism(elevatorHeight, armAngle);
  }
}