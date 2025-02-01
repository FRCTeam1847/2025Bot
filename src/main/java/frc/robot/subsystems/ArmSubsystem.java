package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private final RelativeEncoder integratedEncoder;
  private final SparkClosedLoopController closedLoopController;
  private final DutyCycleEncoder absoluteEncoder;

  private final double kP = 0.1;
  private final double kI = 0.0;
  private final double kD = 0.0;
  private final double velocityFF = 0.211; // Example feedforward for velocity control

  // Gear ratio: 25:1 planetary * 3:1 chain = 75:1 total
  private final double gearRatio = 75.0;
  private final double angleOffset = 77;

  private Rotation2d targetAngle = Rotation2d.fromDegrees(90); // Default to 0 degrees

  public ArmSubsystem() {
    motor = new SparkMax(9, MotorType.kBrushless);

    SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig.encoder
        .positionConversionFactor(360.0 / gearRatio) // motor rotations -> output degrees
        .velocityConversionFactor(360.0 / gearRatio); // motor RPM -> deg/min on output

    // Configure closed loop parameters
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(kP)
        .i(kI)
        .d(kD)
        .outputRange(-1, 1)
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(velocityFF, ClosedLoopSlot.kSlot1)
        .outputRange(-0.25, 0.25, ClosedLoopSlot.kSlot1);
    motorConfig.closedLoopRampRate(0.2);
    // Apply the configuration
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    integratedEncoder = motor.getEncoder();
    closedLoopController = motor.getClosedLoopController();

    absoluteEncoder = new DutyCycleEncoder(9);

  }

  public void setTargetAngle(Rotation2d targetAngle) {
    this.targetAngle = targetAngle;
    closedLoopController.setReference(targetAngle.getDegrees(), SparkMax.ControlType.kMAXMotionPositionControl);
  }

  public Rotation2d getTargetAngle() {
    return targetAngle;
  }

  /**
   * Returns the arm angle as measured by the Spark Max’s integrated/relative
   * encoder.
   * If you set the positionConversionFactor to (360.0 / gearRatio), then this
   * will be
   * in degrees of actual arm rotation.
   */
  public Rotation2d getArmAngle() {
    return Rotation2d.fromDegrees(integratedEncoder.getPosition());
  }

  /**
   * Returns the absolute angle of the arm as measured by the DutyCycleEncoder on
   * the RIO.
   * This should give you a true absolute position (0-360) regardless of arm
   * motion during power-off.
   */
  public Rotation2d getAbsoluteAngle() {
    double rawPosition = absoluteEncoder.get(); // [0..1)
    double degrees = rawPosition * 360.0; // Convert 0..1 -> 0..360
    return Rotation2d.fromDegrees(degrees-angleOffset);
  }

  @Override
  public void periodic() {
    
  }
}
