package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Levels;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ManipulatorSubsystem extends SubsystemBase {
  // private final ArmSubsystem armSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  // private final SwerveSubsystem drivebase;

  private final LoggedMechanism2d combinedMechanism2d;
  private final LoggedMechanismRoot2d baseRoot;
  private final LoggedMechanismLigament2d elevatorLigament;
  private final LoggedMechanismLigament2d armLigament;

  private static final int BASE_X = 50;
  private static final int BASE_Y = 10;

  public ManipulatorSubsystem(ElevatorSubsystem elevatorSubsystem) {
    // this.armSubsystem = armSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    // this.drivebase = drivebase;

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

  public void updateMechanism(double elevatorHeight) {
    // Update elevator height
    elevatorLigament.setLength(elevatorHeight);

    // Update arm angle
    // armLigament.setAngle(90 - armAngle);
    // Logger.recordOutput("Field/Robot", new Pose3d(drivebase.getPose()));
    Logger.recordOutput("Field/Robot/ManipulatorMechanism", getManipulatorPose3d());
    // Log updates for visualization
    Logger.recordOutput("Mechanism2d/ManipulatorMechanism", combinedMechanism2d);

  }

  public Pose3d getManipulatorPose3d() {
    // Step 1: Get the robot's base pose in field coordinates
    // Pose2d basePose2d = drivebase.getPose(); // Assuming drivebase provides a
    // Pose2d
    Translation3d baseTranslation = new Translation3d(
        0,
        0,
        0);
    Rotation3d baseRotation = new Rotation3d(
        0,
        0,
        0);

    // // Conversion factor: Inches to Meters
    final double inchesToMeters = 0.0254;

    // // Step 2: Elevator translation (z-axis) in robot's local frame
    double elevatorHeightMeters = 0.0381 + elevatorSubsystem.getTargetHeight() * inchesToMeters;
    Translation3d elevatorTranslation = new Translation3d(0, 0, elevatorHeightMeters);

    // // Step 3: Arm translation relative to the elevator top
    // double armLengthMeters = armLigament.getLength() * inchesToMeters; // Arm
    // length in meters
    // double armAngleRadians = Math.toRadians(90 -
    // armSubsystem.getTargetAngle().getDegrees());
    // Translation3d armTranslation = new Translation3d(
    // armLengthMeters * Math.cos(armAngleRadians), // X offset in local frame
    // 0, // Y offset (assuming no lateral movement)
    // armLengthMeters * Math.sin(armAngleRadians) // Z offset
    // );

    // // Combine elevator and arm translations in the robot's local frame
    // Translation3d manipulatorTranslationInRobotFrame =
    // elevatorTranslation.plus(armTranslation);

    // // Step 4: Transform manipulator translation into field coordinates
    // Translation3d manipulatorTranslationInFieldFrame = new Translation3d(
    // manipulatorTranslationInRobotFrame.getX() * basePose2d.getRotation().getCos()
    // - manipulatorTranslationInRobotFrame.getY() *
    // basePose2d.getRotation().getSin(),
    // manipulatorTranslationInRobotFrame.getX() * basePose2d.getRotation().getSin()
    // + manipulatorTranslationInRobotFrame.getY() *
    // basePose2d.getRotation().getCos(),
    // manipulatorTranslationInRobotFrame.getZ());

    // // Combine with base translation to get the final Pose3d
    // Translation3d combinedTranslation =
    // baseTranslation.plus(manipulatorTranslationInFieldFrame);

    // Return the final Pose3d
    return new Pose3d(elevatorTranslation, baseRotation);
  }

  public void setLevel(Levels level, boolean left) {
    switch (level) {
      case CoralStation:
        elevatorSubsystem.setTargetHeight(0);
        // armSubsystem.setTargetAngle(Rotation2d.fromDegrees(!left ? 45 : 135));
        break;
      case L1:
        elevatorSubsystem.setTargetHeight(2);
        // armSubsystem.setTargetAngle(Rotation2d.fromDegrees(!left ? 0 : 180));
        break;
      case L2:
        elevatorSubsystem.setTargetHeight(4);
        // armSubsystem.setTargetAngle(Rotation2d.fromDegrees(!left ? 60 : 120));
        break;
      case L3:
        elevatorSubsystem.setTargetHeight(6);
        // armSubsystem.setTargetAngle(Rotation2d.fromDegrees(!left ? 60 : 120));
        break;
      case L4:
        elevatorSubsystem.setTargetHeight(10);
        // armSubsystem.setTargetAngle(Rotation2d.fromDegrees(!left ? 60 : 120));
        break;
      default:
      case Home:
        elevatorSubsystem.setTargetHeight(0);
        // armSubsystem.setTargetAngle(Rotation2d.fromDegrees(90));
        break;
    }
  }

  @Override
  public void periodic() {
    double elevatorHeight = elevatorSubsystem.getTargetHeight();
    // double armAngle = armSubsystem.getTargetAngle().getDegrees();

    // Update the visualization
    updateMechanism(elevatorHeight);

    // Optional: Log heights and angles for debugging
    // SmartDashboard.putNumber("Elevator Height", elevatorHeight);
    // SmartDashboard.putNumber("Arm Angle", armAngle);
  }
}