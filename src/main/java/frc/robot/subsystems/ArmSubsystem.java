package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder integratedEncoder;
    private final SparkClosedLoopController closedLoopController;

    private final double kP = 0.01;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double velocityFF = 0.211;

    // Gear ratio: 3:1 gearbox (Motor rotates 3x for each full arm rotation)
    private final double gearRatio = 3.0;

    // Arm movement limits
    private static final double MIN_ANGLE = 0.0;
    private static final double MAX_ANGLE = 45.0;

    private Rotation2d targetAngle = Rotation2d.fromDegrees(0); // Default position

    public ArmSubsystem() {
        motor = new SparkMax(13, MotorType.kBrushless);

        SparkMaxConfig motorConfig = new SparkMaxConfig();

        // **Corrected position conversion factor**
        motorConfig.encoder
                .positionConversionFactor(360.0 / gearRatio) // Each motor rotation corresponds to 120 degrees of arm
                                                             // movement
                .velocityConversionFactor(360.0 / gearRatio); // Correct velocity conversion

        // Configure closed-loop control
        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(kP)
                .i(kI)
                .d(kD)
                .outputRange(-0.5, 0.5)
                .velocityFF(velocityFF);

        motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        integratedEncoder = motor.getEncoder();
        closedLoopController = motor.getClosedLoopController();

        // Reset encoder position to zero at startup
        integratedEncoder.setPosition(0);
    }

    /**
     * Sets the target angle of the arm within limits.
     */
    public void setTargetAngle(Rotation2d targetAngle) {
        double angle = targetAngle.getDegrees();

        // Clamp angle within limits
        angle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));

        this.targetAngle = Rotation2d.fromDegrees(angle);
        closedLoopController.setReference(angle, SparkMax.ControlType.kPosition);
    }

    /**
     * Toggles between 0° and 45° based on the actual arm position.
     */
    public void toggleTargetAngle() {
        double currentAngle = getArmAngle().getDegrees(); // Use actual encoder value
        System.out.println("Toggle Check - Current Angle: " + currentAngle);

        if (currentAngle < 2.0) { // Use 2-degree buffer to ensure toggle happens
            setTargetAngle(Rotation2d.fromDegrees(45));
        } else {
            setTargetAngle(Rotation2d.fromDegrees(0));
        }
    }

    /**
     * Returns the current target angle.
     */
    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    /**
     * Returns the actual arm angle from the built-in NEO encoder.
     */
    public Rotation2d getArmAngle() {
        return Rotation2d.fromDegrees(integratedEncoder.getPosition());
    }

    /**
     * Command that moves the arm back and forth between 0° and 45° continuously.
     */
    public Command moveArmBackAndForth() {
        return run(() -> {
            double currentAngle = getArmAngle().getDegrees();
            double target = getTargetAngle().getDegrees();

            System.out.println("Current: " + currentAngle + " | Target: " + target);

            // If close to target, toggle to the other position
            if (Math.abs(currentAngle - target) < 2.0) { // Increase threshold from 1.0 to 2.0
                toggleTargetAngle();
            }
        }).repeatedly();
    }

    @Override
    public void periodic() {
        // Debugging: Print the current arm position
        System.out.println("Raw Encoder Position: " + integratedEncoder.getPosition());
        System.out.println("Arm Angle: " + getArmAngle().getDegrees());
    }
}
