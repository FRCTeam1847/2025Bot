// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private LaserCan lc;
  private final SparkMax intakeMotor;
  private final double speed = 0.5;
  private double intakeSpeed = 0;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    intakeMotor = new SparkMax(12, MotorType.kBrushless);
    intakeMotor.configure(intakeMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    lc = new LaserCan(0);
    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  public void intake() {
    intakeSpeed = speed;

  }

  public void release() {
    intakeSpeed = -0.6;
  }

  public void stopIntake() {
    intakeSpeed = 0;
  }

  @Override
  public void periodic() {
    LaserCan.Measurement measurement = lc.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      System.out.println("The target is " + measurement.distance_mm + "mm away!");
      if (measurement.ambient < 100 && intakeSpeed > 0) {
        intakeSpeed = 0;
      }
    } 
    intakeMotor.set(intakeSpeed);
  }
}
