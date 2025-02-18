package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorSubsystem extends SubsystemBase {

  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final RelativeEncoder leftEncoder;
  private final SparkClosedLoopController leftClosedLoopController;

  private static final double GEAR_RATIO = 20.0; // 20:1 gearbox
  private static final double SPROCKET_DIAMETER_INCHES = 1.75; // Change this based on your actual sprocket diameter
  private static final double SPROCKET_CIRCUMFERENCE = SPROCKET_DIAMETER_INCHES * Math.PI; // inches per rev
  private static final double kP = 0.115;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double MAX_HEIGHT = 26.5;
  private double currentHeight = 0.0;

  public ElevatorSubsystem() {
    leftMotor = new SparkMax(11, MotorType.kBrushless);
    rightMotor = new SparkMax(10, MotorType.kBrushless);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);
    leftConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(kP).i(kI).d(kD).outputRange(-0.75, 0.75);
    leftConfig.inverted(false);
    leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.follow(11, true);

    rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    leftEncoder = leftMotor.getEncoder();
    leftClosedLoopController = leftMotor.getClosedLoopController();

    leftEncoder.setPosition(0);
  }

  public void setTargetHeight(double heightInches) {

    currentHeight = Math.max(0, Math.min(MAX_HEIGHT, heightInches)); // Clamp height

    // Convert height to motor rotations
    double requiredRotations = (currentHeight / SPROCKET_CIRCUMFERENCE) * GEAR_RATIO;

    // Set motor position control
    leftClosedLoopController.setReference(requiredRotations, ControlType.kPosition);
  }

  public double getTargetHeight() {
    return currentHeight;
  }

  public double getElevatorHeight() {
    double motorRotations = leftEncoder.getPosition(); // Returns rotations of the motor shaft
    double elevatorRotations = motorRotations / GEAR_RATIO; // Convert to rotations of the output shaft
    double elevatorHeight = elevatorRotations * SPROCKET_CIRCUMFERENCE; // Convert to height in inches

    return elevatorHeight;
  }

  @Override
  public void periodic() {
    // System.out.println("Current Height: " + getElevatorHeight() + " inches,
    // Target: " + getTargetHeight());
  }

  public void resetEncoder() {
    leftEncoder.setPosition(0);
  }
}
