package frc.robot.subsystems;

public class Constants {

  // Intake Constants
  public static final int Intake_Roller = 12;
  public static final int Intake_Left = 10;
  public static final int Intake_Right = 11;
  public static final int Intake_Encoder = 2;

  // Intake speed (velocity)
  public static final double IntakeWheel_Velocity_Forward = 0.09;
  public static final double IntakeWheel_Velocity_Reverse = -0.09;
  public static final double IntakeWheel_Velocity_Stop = 0.0;

  // Intake encoder positions
  public static final double Intake_Up = 0.64;
  public static final double Intake_Down = 0.21;

  // Intake PID
  public static final double Intake_kP = 1.6;
  public static final double Intake_kI = 0.0;
  public static final double Intake_kD = 0.0;

  /**********************************************************************************************************************/

  // Hood Constants

  // Hardware
  public static final int Hood_Motor = 15;

  public static final int Hood_Encoder = 1;
  public static final int Hood_Zero_Switch = 3;

  // Hood encoder positions
  public static final double POSITION1 = 0.195;
  public static final double POSITION2 = 1.200;
  public static final double POSITION3 = 2.244;

  // Hood position limits
  public static final double Hood_Min_Position = POSITION1;
  public static final double Hood_Max_Position = POSITION3;

  // Hood motor speeds
  public static final double Hood_Manual_Output = 0.06;
  public static final double Hood_Max_Output = 0.10;

  // Hood PID
  public static final double Hood_kP = 0.6;
  public static final double Hood_kI = 0.0;
  public static final double Hood_kD = 0.0;
  public static final double Hood_Position_Tolerance = 0.01;

  // Hood safety
  // Leave false until hardware checks are complete.
  public static final boolean Hood_Controls_Armed = false;

  // Limit switch polarity - must verify
  public static final boolean Hood_Limit_Active_Low = true;

  // Motor direction
  // Positive output should move hood UP
  public static final double Hood_Motor_Up_Sign = 1.0;

  // Encoder direction
  // Positive encoder change should represent UP
  public static final double Hood_Encoder_Up_Sign = 1.0;

  // Encoder rollover protection
  public static final double Hood_Max_Encoder_Change_Per_Loop = 0.20;

  // Limelight
  public static final String Hood_Limelight_Name = "limelight";
  public static final double Hood_Limelight_Timeout = 0.5;

  // Center Hub AprilTag IDs
  // Verify against official field layout
  public static final int[] Hood_Red_Hub_Tag_IDs = {2, 4, 5, 10};
  public static final int[] Hood_Blue_Hub_Tag_IDs = {18, 20, 21, 26};

  /**********************************************************************************************************************/

  // Indexer Constants
  public static final int INDEXER_1_MOTOR_ID = 13;

  public static final int INDEXER_2_MOTOR_ID = 17;

  public static final double INDEXER_1_SPEED = -0.45;
  public static final double INDEXER_2_SPEED = -0.3;

  public static final double INDEXER_1_REVERSE_SPEED = 0.45;
  public static final double INDEXER_2_REVERSE_SPEED = 0.3;
}
