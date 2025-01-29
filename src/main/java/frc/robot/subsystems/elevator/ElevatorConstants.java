package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorConstants {
    /** current limit on the elevator motor in amps */
    public static int MOTOR_CURRENT_LIMIT = 50;
    public static int LEFT_MOTOR_ID = 0;
    public static int RIGHT_MOTOR_ID = 1;

    public static IdleMode MOTOR_DEFAULT_IDLE_MODE = IdleMode.kBrake;
}
