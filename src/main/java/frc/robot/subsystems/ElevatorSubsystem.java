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
  // private final LoggedMechanism2d mechanism2d;
  // private final LoggedMechanismRoot2d elevatorRoot;
  // private final LoggedMechanismLigament2d elevatorLigament;

  private static final double DIST_PER_ROT_IN = (Math.PI * 2.0) / 20.0; // Spool diameter divided by gear ratio
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double maxHeight = 10.0; // Max height in inches
  private double currentHeight = 0.0;

  public ElevatorSubsystem() {
    leftMotor = new SparkMax(10, MotorType.kBrushless);
    rightMotor = new SparkMax(11, MotorType.kBrushless);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.encoder
        .positionConversionFactor(DIST_PER_ROT_IN)
        .velocityConversionFactor(DIST_PER_ROT_IN);
    leftConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(kP).i(kI).d(kD).outputRange(-1, 1);
    leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig rightConfig = leftConfig;
    rightConfig.follow(10);
    rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    leftEncoder = leftMotor.getEncoder();
    leftClosedLoopController = leftMotor.getClosedLoopController();

    // mechanism2d = new LoggedMechanism2d(100, 100);
    // elevatorRoot = mechanism2d.getRoot("ElevatorRoot", 50, 10);
    // elevatorLigament = elevatorRoot.append(new
    // LoggedMechanismLigament2d("Elevator", 0, 90));

    // SmartDashboard.putData("Elevator Mechanism", mechanism2d);

    leftEncoder.setPosition(0);
  }

  public void setTargetHeight(double heightInches) {
    currentHeight = heightInches;
    heightInches = Math.max(0, Math.min(maxHeight, heightInches)); // Clamp the height
    leftClosedLoopController.setReference(heightInches / DIST_PER_ROT_IN, ControlType.kPosition);
    // updateMechanism2d(heightInches);
  }

  public double getTargetHeight() {
    return currentHeight;
  }
  // private void updateMechanism2d(double heightInches) {
  // elevatorLigament.setLength(heightInches);
  // }

  @Override
  public void periodic() {
    // double currentHeight = leftEncoder.getPosition() * DIST_PER_ROT_IN;
    // SmartDashboard.putNumber("Elevator Height", currentHeight);
    // Logger.recordOutput("Mechanism2d/Elevator", mechanism2d);
  }

  public void resetEncoder() {
    leftEncoder.setPosition(0);
  }
}
