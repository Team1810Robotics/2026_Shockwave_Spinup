package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkMax Intake_Right = new SparkMax(Constants.Intake_Right, MotorType.kBrushless);
  private final SparkMax Intake_Left = new SparkMax(Constants.Intake_Left, MotorType.kBrushless);
  private final SparkMax Intake_Roller =
      new SparkMax(Constants.Intake_Roller, MotorType.kBrushless);
  private final DutyCycleEncoder Intake_Encoder = new DutyCycleEncoder(Constants.Intake_Encoder);
  private static final double GEAR_RATIO = 1.0;
  private static final double ROTATIONS_TO_DEGREES = 360.0 / GEAR_RATIO;
  private static final double ROTATIONS_TO_RADIANS = (2.0 * Math.PI) / GEAR_RATIO;
  private final SparkMaxConfig leftconfig = new SparkMaxConfig();
  private final SparkMaxConfig rightconfig = new SparkMaxConfig();
  private double targetPosition = Constants.Intake_Down;

  // NEW: PID
  private final PIDController intakePID =
      new PIDController(Constants.Intake_kP, Constants.Intake_kI, Constants.Intake_kD);

  private static final double POSITION_TOLERANCE = 0.01;
  private static final double MAX_PID_OUTPUT = 0.35;
  private boolean pidActive = false;

  public void setPoint(double position) {
    targetPosition = position;

    // NEW: Start PID movement
    intakePID.reset();
    pidActive = true;
  }

  public IntakeSubsystem() {
    leftconfig.inverted(true);
    rightconfig.inverted(false);

    Intake_Left.configure(
        leftconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Intake_Right.configure(
        rightconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intakeForward() {
    Intake_Roller.set(Constants.IntakeWheel_Velocity_Forward);
  }

  public void intakeBackward() {
    Intake_Roller.set(Constants.IntakeWheel_Velocity_Reverse);
  }

  public void intakeStop() {
    Intake_Roller.set(Constants.IntakeWheel_Velocity_Stop);
  }

  public double getMotorRotations() {
    return Intake_Encoder.get();
  }

  // Intake position in degrees
  public double getIntakeDegrees() {
    return getMotorRotations() * ROTATIONS_TO_DEGREES;
  }

  // Intake position in radians
  public double getIntakeRadians() {
    return getMotorRotations() * ROTATIONS_TO_RADIANS;
  }

  public void intakeUp() {
    Intake_Left.set(0.1);
    Intake_Right.set(0.1);
    Intake_Left.set(Constants.Intake_Up);
    Intake_Right.set(Constants.Intake_Up);
  }

  public void intakeDown() {
    Intake_Left.set(0.1);
    Intake_Right.set(0.1);
    Intake_Left.set(Constants.Intake_Down);
    Intake_Right.set(Constants.Intake_Down);
  }

  // NEW: Stop both intake arm motors
  public void stopArm() {
    Intake_Left.stopMotor();
    Intake_Right.stopMotor();
  }

  @Override
  public void periodic() {

    // NEW: Only calculate PID when movement is requested
    if (!pidActive) {
      return;
    }

    // Stop if encoder is disconnected
    if (!Intake_Encoder.isConnected()) {
      stopArm();
      pidActive = false;
      return;
    }

    double currentPosition = Intake_Encoder.get();

    // NEW: Stop when setpoint is reached
    if (Math.abs(targetPosition - currentPosition) <= POSITION_TOLERANCE) {
      stopArm();
      pidActive = false;
      return;
    }

    // NEW: Calculate PID motor output
    double output = intakePID.calculate(currentPosition, targetPosition);

    // Limit motor speed
    output = MathUtil.clamp(output, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);

    Intake_Left.set(output);
    Intake_Right.set(output);
  }
}
