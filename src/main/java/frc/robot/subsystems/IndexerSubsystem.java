package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

  private final SparkMax Intake1 = new SparkMax(Constants.INDEXER_1_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax Intake2 = new SparkMax(Constants.INDEXER_2_MOTOR_ID, MotorType.kBrushless);

  public void runForward() {
    Intake1.set(Constants.INDEXER_1_SPEED);
    Intake2.set(Constants.INDEXER_2_SPEED);
  }

  public void runReverse() {
    Intake1.set(Constants.INDEXER_1_REVERSE_SPEED);
    Intake2.set(Constants.INDEXER_2_REVERSE_SPEED);
  }

  public void stop() {
    Intake1.stopMotor();
    Intake2.stopMotor();
  }
}
