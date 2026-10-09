package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax Intake_Right = new SparkMax(Constants.Intake_Right, MotorType.kBrushless);
    private final SparkMax Intake_Left = new SparkMax(Constants.Intake_Left, MotorType.kBrushless);
    private final SparkMax Intake_Roller = new SparkMax(Constants.Intake_Roller, MotorType.kBrushless);
    private final DutyCycleEncoder Intake_Encoder = new DutyCycleEncoder(Constants.Intake_Encoder);
    private static final double GEAR_RATIO = 1.0;
    private static final double ROTATIONS_TO_DEGREES = 360.0 / GEAR_RATIO;
    private static final double ROTATIONS_TO_RADIANS = (2.0 * Math.PI) / GEAR_RATIO;
    private final SparkMaxConfig leftconfig = new SparkMaxConfig();
    private final SparkMaxConfig rightconfig = new SparkMaxConfig();

   public IntakeSubsystem() {
    leftconfig.inverted(true);
    rightconfig.inverted(false);

    Intake_Left.configure(
        leftconfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
    );

    Intake_Right.configure(
        rightconfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
    );
}
    
    public void intakeForward() {
       Intake_Roller.set(Constants.IntakeWheel_Velocity_Forward);
    }
    
    public void intakeBackward() {
       Intake_Roller.set(Constants.IntakeWheel_Velocity_Backward); 
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
 
@Override
public void periodic() {
    SmartDashboard.putNumber("Intake Arm Position", Intake_Encoder.get());
}
}