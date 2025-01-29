package frc.robot.subsystems.elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorIOSparkMax implements ElevatorIO{
    SparkMaxConfig config;
    SparkMax leftMotor;
    SparkMax rightMotor;
    AbsoluteEncoder leftEncoder;
    AbsoluteEncoder rightEncoder;

    public ElevatorIOSparkMax() {
        leftMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

        leftEncoder = leftMotor.getAbsoluteEncoder();
        rightEncoder = rightMotor.getAbsoluteEncoder();

        config = new SparkMaxConfig();
        config
            .smartCurrentLimit(ElevatorConstants.MOTOR_CURRENT_LIMIT)
            .idleMode(ElevatorConstants.MOTOR_DEFAULT_IDLE_MODE);

        //reset safe kResetSafeParameters switches the motor to default paramaters, then adds the changes from the config object
        //persist paramaters saves these changes to the motor memory so it doesn't get cooked during brownouts
        //only use persist in the BEGINNING, not later
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setBrakeMode(boolean brakeEnabled) {
        config.idleMode(brakeEnabled ? IdleMode.kBrake : IdleMode.kCoast);
        //we only want to change one paramater, so DO NOT reset paramaters
        //because this is a temporary change at runtime, do not persist
        leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rightMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
