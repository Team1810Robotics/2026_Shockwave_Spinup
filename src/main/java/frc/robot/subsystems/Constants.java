package frc.robot.subsystems;

public class Constants {

//Intake Constants
    public static final int Intake_Roller = 12; 
    public static final int Intake_Left = 10;
    public static final int Intake_Right = 11;
    public static final int Intake_Encoder = 2;
    //Intake speed(velocity)
    public static final double IntakeWheel_Velocity_Forward = 0.09;
    public static final double IntakeWheel_Velocity_Backward = -0.09;
    public static final double IntakeWheel_Velocity_Stop = 0.0;
    //Encoder stuff
    public static final double Intake_Up = 0.64;
    public static final double Intake_Down = 0.21;
    //PID constants
    public static final double Intake_kP = 1.6;
    public static final double Intake_kI = 0.0;
    public static final double Intake_kD = 0.0;

/**********************************************************************************************************************/

//Hood Constants
    public static final int Hood_Motor = 15;
    public static final int Hood_Encoder = 1;
    public static final double Hood_kP = 1.0;
    public static final double Hood_kI = 0.0;
    public static final double Hood_kD = 0.0;   
    public static final double Hood_Velocity = 0.25;

/**********************************************************************************************************************/
//Indexer Constants
   public static final int INDEXER_1_MOTOR_ID = 13;
    public static final int INDEXER_2_MOTOR_ID = 17;
    public static final double INDEXER_1_SPEED = -0.45;
    public static final double INDEXER_2_SPEED = -0.3;
    public static final double INDEXER_1_REVERSE_SPEED = 0.45;
    public static final double INDEXER_2_REVERSE_SPEED = 0.3;
} 
