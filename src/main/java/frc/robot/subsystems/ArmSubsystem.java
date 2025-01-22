// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController closedLoopController;

  private final double kP = 0.1;
  private final double kI = 0.0;
  private final double kD = 0.0;
  private final double velocityFF = 0.211; // Example feedforward for velocity control

  private final double gearRatio = 20.0; // Adjust based on your gearbox and sprocket ratio

  private Rotation2d targetAngle = Rotation2d.fromDegrees(0); // Default to 0 degrees

  // Mechanism visualization
  private final LoggedMechanism2d mechanism = new LoggedMechanism2d(100, 50); // Width, height (adjust as needed)
  private final LoggedMechanismRoot2d armRoot;
  private final LoggedMechanismLigament2d armLigament;

  public ArmSubsystem() {
    motor = new SparkMax(9, MotorType.kBrushless);

    // Create a configuration object
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    // Configure the encoder
    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    // Configure the closed loop controller
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

    encoder = motor.getEncoder();

    closedLoopController = motor.getClosedLoopController();

    // Initialize the mechanism visualization
    armRoot = mechanism.getRoot("ArmRoot", 50, 25); // Root at the center (adjust coordinates)
    armLigament = armRoot.append(new LoggedMechanismLigament2d("Arm", 20, 90)); // Length and angle (adjust length)
  }

  public void setTargetAngle(Rotation2d targetAngle) {
    this.targetAngle = targetAngle;
  }

  public Rotation2d getTargetAngle() {
    return targetAngle;
  }

  @Override
  public void periodic() {
    // Continuously maintain the target angle
    double targetPosition = targetAngle.getDegrees() * (gearRatio / 360.0);
    closedLoopController.setReference(targetPosition, SparkMax.ControlType.kPosition);

    // Update the arm angle in the visualization
    armLigament.setAngle(targetAngle.getDegrees());

    // Log to AdvantageScope
    // Logger.recordOutput("Arm/Angle", getArmAngle().getDegrees());
    // Logger.recordOutput("Arm/MotorOutput", motor.getAppliedOutput());
    // Logger.recordOutput("Arm/EncoderPosition", encoder.getPosition());
    Logger.recordOutput("ArmMechanism", mechanism);

    // // Optional: Display on SmartDashboard for debugging
    // SmartDashboard.putNumber("Arm Angle (Degrees)", getArmAngle().getDegrees());
    // SmartDashboard.putNumber("Arm Motor Output", motor.getAppliedOutput());
    // SmartDashboard.putNumber("Arm Encoder Position", encoder.getPosition());
  }

  public Rotation2d getArmAngle() {
    return Rotation2d.fromDegrees(encoder.getPosition() * 360.0 / gearRatio);
  }
}
