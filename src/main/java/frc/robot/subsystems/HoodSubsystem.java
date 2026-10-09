package frc.robot.subsystems.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    
    private final TalonFX Hood_Motor = new TalonFX(Constants.Hood_Motor);
    private final DutyCycleEncoder Hood_Encoder = new DutyCycleEncoder(Constants.Hood_Encoder);
    private double lastTYRaw = 0.0;
    private double lastTYUsed = 0.0;
    private double lastPolynomialOutput = 0.0;
    private double lastPolynomialClamped = 0.0;
    private double visionSetPoint = 0.0;

    public double computeHoodSetpointFromTY(double ty) {
    double x = Math.abs(ty);

    double setpoint =
        0.892
            + (-0.271) * x
            + (0.0609) * Math.pow(x, 2)
            + (-5.73e-3) * Math.pow(x, 3)
            + (2.48e-4) * Math.pow(x, 4)
            + (-3.94e-6) * Math.pow(x, 5);

    double clamped = MathUtil.clamp(setpoint, 0.0, 3.0);

    lastTYRaw = ty;
    lastTYUsed = x;
    lastPolynomialOutput = setpoint;
    lastPolynomialClamped = clamped;

    visionSetPoint = clamped;
    return clamped;
  }
}
