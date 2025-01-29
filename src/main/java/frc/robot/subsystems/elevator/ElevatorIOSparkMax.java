package frc.robot.subsystems.elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorIOSparkMax implements ElevatorIO{
    SparkMax leftMotor;
    SparkMax rightMotor;
    SparkClosedLoopController leftController;
    SparkClosedLoopController rightController;
    AbsoluteEncoder leftEncoder;
    AbsoluteEncoder rightEncoder;

    //Because sparkmax does not have getters for pid, use these variables to keep track of dynamic pid values
    double kP = ElevatorConstants.kP;
    double kI = ElevatorConstants.kI;
    double kD = ElevatorConstants.kD;

    public ElevatorIOSparkMax() {
        leftMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

        leftController = leftMotor.getClosedLoopController();
        rightController = rightMotor.getClosedLoopController();

        leftEncoder = leftMotor.getAbsoluteEncoder();
        rightEncoder = rightMotor.getAbsoluteEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .smartCurrentLimit(ElevatorConstants.MOTOR_CURRENT_LIMIT)
            .idleMode(ElevatorConstants.MOTOR_DEFAULT_IDLE_MODE);
        config.closedLoop
            .p(ElevatorConstants.kP)
            .i(ElevatorConstants.kI)
            .d(ElevatorConstants.kD);

        //reset safe kResetSafeParameters switches the motor to default paramaters, then adds the changes from the config object
        //persist paramaters saves these changes to the motor memory so it doesn't get cooked during brownouts
        //only use persist in the BEGINNING, not later
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    void updateMotorConfig(SparkMaxConfig config) {
        //we only want to change one paramater, so DO NOT reset paramaters
        //because this is a temporary change at runtime, do not persist
        leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rightMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    @Override
    public void goToSetpoint(double setpoint) {
        leftController.setReference(setpoint, ControlType.kPosition);
        rightController.setReference(setpoint, ControlType.kPosition);
    }
    @Override
    public void setBrakeMode(boolean brakeEnabled) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(brakeEnabled ? IdleMode.kBrake : IdleMode.kCoast);
        updateMotorConfig(config);
    }
    @Override
    public void setP(double kP) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.p(kP);
        updateMotorConfig(config);
    }
    @Override
    public void setI(double kI) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.i(kI);
        updateMotorConfig(config);
    }
    @Override
    public void setD(double kD) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.d(kD);
        updateMotorConfig(config);
    }
    @Override
    public double getP() {
        return kP;
    }
}
