package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {

  private final TalonFX hoodMotor = new TalonFX(Constants.Hood_Motor);
  private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(Constants.Hood_Encoder);
  private final DigitalInput bottomSwitch = new DigitalInput(Constants.Hood_Zero_Switch);

  private final PIDController hoodPID =
      new PIDController(Constants.Hood_kP, Constants.Hood_kI, Constants.Hood_kD);

  private final NetworkTable limelight =
      NetworkTableInstance.getDefault().getTable(Constants.Hood_Limelight_Name);

  // Encoder tracking and hood state
  private boolean rawInitialized = false;
  private boolean homed = false;
  private boolean automaticEnabled = false;
  private boolean visionValid = false;

  private double lastRaw = 0.0;
  private double continuousRotations = 0.0;
  private double zeroRotations = 0.0;
  private double targetPosition = Double.NaN;

  private double previousHeartbeat = Double.NaN;
  private double heartbeatUpdatedAt = Double.NEGATIVE_INFINITY;

  private int currentTagId = -1;

  public HoodSubsystem() {
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);
    hoodPID.setTolerance(Constants.Hood_Position_Tolerance);
  }

  // Bottom limit switch
  private boolean bottomPressed() {
    return Constants.Hood_Limit_Active_Low ? !bottomSwitch.get() : bottomSwitch.get();
  }

  // Calibrated encoder position relative to the bottom switch
  public double getPosition() {
    return homed
        ? (continuousRotations - zeroRotations) * Constants.Hood_Encoder_Up_Sign
        : Double.NaN;
  }

  // Stop hood motor
  public void stop() {
    hoodMotor.stopMotor();
  }

  // Enable or disable automatic aiming
  public void toggleAutomatic() {
    automaticEnabled = !automaticEnabled;
    visionValid = false;
    targetPosition = Double.NaN;
    hoodPID.reset();
    stop();
  }

  // Limelight tY -> hood encoder position
  public double calculatePositionFromTY(double x) {
    return 0.892
        + (-0.271) * x
        + (0.0609) * Math.pow(x, 2)
        + (-5.73e-3) * Math.pow(x, 3)
        + (2.48e-4) * Math.pow(x, 4)
        + (-3.94e-6) * Math.pow(x, 5);
  }

  // Track multiple encoder rotations
  private void updateEncoderTracking() {

    if (!hoodEncoder.isConnected()) {
      rawInitialized = false;
      homed = false;
      automaticEnabled = false;
      stop();
      return;
    }

    double raw = hoodEncoder.get();

    if (!Double.isFinite(raw)) {
      rawInitialized = false;
      homed = false;
      automaticEnabled = false;
      stop();
      return;
    }

    if (!rawInitialized) {
      lastRaw = raw;
      rawInitialized = true;
      return;
    }

    double change = MathUtil.inputModulus(raw - lastRaw, -0.5, 0.5);

    lastRaw = raw;

    if (Math.abs(change) > Constants.Hood_Max_Encoder_Change_Per_Loop) {
      rawInitialized = false;
      homed = false;
      automaticEnabled = false;
      stop();
      return;
    }

    continuousRotations += change;
  }

  // Apply motor power while respecting position limits
  private void setSafeOutput(double upPositiveOutput) {

    if (!Constants.Hood_Controls_Armed || !hoodEncoder.isConnected()) {
      stop();
      return;
    }

    // Downward movement
    if (upPositiveOutput < 0.0) {

      if (bottomPressed() || (homed && getPosition() <= Constants.Hood_Min_Position)) {
        stop();
        return;
      }

      // Upward movement
    } else if (upPositiveOutput > 0.0) {

      if (!homed || getPosition() >= Constants.Hood_Max_Position) {
        stop();
        return;
      }
    }

    hoodMotor.set(
        MathUtil.clamp(upPositiveOutput, -Constants.Hood_Max_Output, Constants.Hood_Max_Output)
            * Constants.Hood_Motor_Up_Sign);
  }

  // Check whether an AprilTag belongs to our alliance's Hub
  private boolean isAllowedTag(int tagId) {

    if (tagId <= 0) {
      return false;
    }

    DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(null);

    if (alliance == null) {
      return false;
    }

    int[] hubTags =
        alliance == DriverStation.Alliance.Red
            ? Constants.Hood_Red_Hub_Tag_IDs
            : Constants.Hood_Blue_Hub_Tag_IDs;

    for (int allowedId : hubTags) {
      if (allowedId == tagId) {
        return true;
      }
    }

    return false;
  }

  // Check Limelight target and heartbeat
  private boolean hasFreshAprilTag() {

    double heartbeat = limelight.getEntry("hb").getDouble(Double.NaN);

    if (Double.isFinite(heartbeat) && heartbeat != previousHeartbeat) {
      previousHeartbeat = heartbeat;
      heartbeatUpdatedAt = Timer.getFPGATimestamp();
    }

    currentTagId = (int) limelight.getEntry("tid").getDouble(-1.0);

    double hasTarget = limelight.getEntry("tv").getDouble(0.0);

    return Timer.getFPGATimestamp() - heartbeatUpdatedAt < Constants.Hood_Limelight_Timeout
        && hasTarget > 0.5
        && isAllowedTag(currentTagId);
  }

  // Manual and automatic hood control
  public void control(int pov) {

    if (!DriverStation.isTeleopEnabled()
        || !Constants.Hood_Controls_Armed
        || !hoodEncoder.isConnected()) {
      visionValid = false;
      stop();
      return;
    }

    // Manual control using D-pad
    if (pov == 0 || pov == 180) {

      visionValid = false;
      targetPosition = Double.NaN;
      hoodPID.reset();

      double power = pov == 0 ? Constants.Hood_Manual_Output : -Constants.Hood_Manual_Output;

      setSafeOutput(power);
      return;
    }

    // Automatic mode must be enabled and hood homed
    if (!automaticEnabled || !homed) {
      visionValid = false;
      stop();
      return;
    }

    // Check AprilTag
    visionValid = hasFreshAprilTag();

    if (!visionValid) {
      targetPosition = Double.NaN;
      hoodPID.reset();
      stop();
      return;
    }

    // Read Limelight tY
    double ty = limelight.getEntry("ty").getDouble(Double.NaN);

    if (!Double.isFinite(ty)) {
      visionValid = false;
      stop();
      return;
    }

    // Calculate requested hood position
    double requested = calculatePositionFromTY(ty);

    // Enforce lower and upper position limits
    if (!Double.isFinite(requested)
        || requested < Constants.Hood_Min_Position
        || requested > Constants.Hood_Max_Position) {
      targetPosition = Double.NaN;
      hoodPID.reset();
      stop();
      return;
    }

    targetPosition = requested;

    double currentPosition = getPosition();

    // Stop at setpoint
    if (Math.abs(targetPosition - currentPosition) <= Constants.Hood_Position_Tolerance) {
      stop();
      return;
    }

    // PID motor movement
    double pidOutput = hoodPID.calculate(currentPosition, targetPosition);

    setSafeOutput(pidOutput);
  }

  @Override
  public void periodic() {

    // Update encoder position
    updateEncoderTracking();

    // Require homing after robot is disabled
    if (DriverStation.isDisabled()) {

      homed = false;
      automaticEnabled = false;
      visionValid = false;
      targetPosition = Double.NaN;
      stop();

    } else if (rawInitialized && bottomPressed()) {

      // Zero the hood at the bottom limit switch
      zeroRotations = continuousRotations;
      homed = true;
      hoodPID.reset();
    }
  }
}
