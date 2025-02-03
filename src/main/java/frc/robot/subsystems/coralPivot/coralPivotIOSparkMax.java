package frc.robot.subsystems.coralPivot;

import static frc.robot.Constants.ElectricalLayout.PIVOT_ARM_ID;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Constants.ElectricalLayout;

public class CoralPivotIOSparkMax implements CoralPivotIO {
    // Motor and Encoders
    private CANSparkMax pivotMotor;
    private final ProfiledPIDController pidController;
    private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);
    
    private DutyCycleEncoder absoluteEncoder;
    private RelativeEncoder motorEncoder;

    private double setpoint = 0;

    public CoralPivotIOSparkMax() {
        pivotMotor = new CANSparkMax(PIVOT_ARM_ID, MotorType.kBrushless);
    

        pivotMotor.restoreFactoryDefaults();

        setBrake(true);
        
        pivotMotor.enableVoltageCompensation(12.0);

        pivotMotor.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
       
        pivotMotor.burnFlash();
       
        //wasn't burning the flash to all the motors, this might be the issue

        absoluteEncoder = new DutyCycleEncoder(ElectricalLayout.ABSOLUTE_ENCODER_ID);
        absoluteEncoder.setDistancePerRotation(2 * Constants.PI * coralPivotConstants.POSITION_CONVERSION_FACTOR);
        absoluteEncoder.setDutyCycleRange(1/1024.0, 1023.0/1024.0);
        // absoluteEncoder.reset();
        Logger.recordOutput("Absolute Encoder Starting Position: ", absoluteEncoder.getDistance());
        // make sure the pivot starts at the bottom position every time
        // absoluteEncoder.reset();

        pidController = new ProfiledPIDController(CoralPivotConstants.PIVOT_ARM_PID_REAL[0], CoralPivotConstants.PIVOT_ARM_PID_REAL[1], CoralPivotConstants.PIVOT_ARM_PID_REAL[2],
                new TrapezoidProfile.Constraints(2.45, 2.45));
        
        pidController.setTolerance(CoralPivotConstants.PIVOT_ARM_PID_TOLERANCE, CoralPivotConstants.PIVOT_ARM_PID_VELOCITY_TOLERANCE);

 
        //0 position for absolute encoder is at 0.2585 rad, so subtract that value from everything

        motorEncoder = pivotMotor.getEncoder();
        motorEncoder.setPositionConversionFactor(CoralPivotConstants.POSITION_CONVERSION_FACTOR);
        motorEncoder.setVelocityConversionFactor(CoralPivotConstants.POSITION_CONVERSION_FACTOR / 60.0);
        configurePID();
        configureFeedForward();

    }

    private void configurePID() {
        // pidController.setOutputRange(CoralPivotConstants.PIVOT_ARM_MIN_ANGLE, CoralPivotConstants.PIVOT_ARM_MAX_ANGLE);
        pidController.setP(CoralPivotConstants.PIVOT_ARM_PID_REAL[0]);
        pidController.setI(CoralPivotConstants.PIVOT_ARM_PID_REAL[1]);
        pidController.setD(CoralPivotConstants.PIVOT_ARM_PID_REAL[2]);
    }

    private void configureFeedForward() {
        setkS(CoralPivotConstants.PIVOT_ARM_FEEDFORWARD_REAL[0]);
        setkG(CoralPivotConstants.PIVOT_ARM_FEEDFORWARD_REAL[1]);
        setkV(CoralPivotConstants.PIVOT_ARM_FEEDFORWARD_REAL[2]);
        setkA(CoralPivotConstants.PIVOT_ARM_FEEDFORWARD_REAL[3]);
    }

    /** Updates the set of loggable inputs. */
    @Override
    public void updateInputs(CoralPivotIOInputs inputs) {
        inputs.angleRads = getAngle();
        Logger.recordOutput("CoralPivot/Absolute", absoluteEncoder.getAbsolutePosition());
        Logger.recordOutput("CoralPivot/MotorEncoder", motorEncoder.getPosition());
        inputs.angVelocityRadsPerSec = motorEncoder.getVelocity();
        inputs.appliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.currentAmps = new double[] {pivotMotor.getOutputCurrent()};
        inputs.tempCelsius = new double[] {pivotMotor.getMotorTemperature()};
        inputs.setpointAngleRads = setpoint;
    }

    /** Run open loop at the specified voltage. */
    @Override
    public void setVoltage(double motorVolts) {
        Logger.recordOutput("CoralPivot/AppliedVolts", motorVolts);
        pivotMotor.setVoltage(motorVolts);
    }

    /** Returns the current distance measurement. */
    @Override
    public double getAngle() {
        return -absoluteEncoder.getDistance() + CoralPivotConstants.PIVOT_ARM_OFFSET;
    }

    /** Go to Setpoint */
    @Override
    public void goToSetpoint(double setpoint) {
        pidController.setGoal(setpoint);
        // With the setpoint value we run PID control like normal
        double pidOutput = MathUtil.clamp(pidController.calculate(getAngle()), -3, 3);
        double feedforwardOutput = feedforward.calculate(getAngle(), pidController.getSetpoint().velocity);

        Logger.recordOutput("CoralPivot/FeedforwardOutput", feedforwardOutput);
        Logger.recordOutput("CoralPivot/PIDOutput", pidOutput);

        Logger.recordOutput("CoralPivot/VelocityError", pidController.getVelocityError());

        setVoltage(MathUtil.clamp(pidOutput + feedforwardOutput, -4, 4));
    }

    @Override
    public void holdSetpoint(double setpoint) {
        goToSetpoint(setpoint);
    }

    @Override
    public void setBrake(boolean brake) {
        pivotMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public boolean atSetpoint() {
        return Math.abs(getAngle() - setpoint) < CoralPivotConstants.PIVOT_ARM_PID_TOLERANCE;
    }

    @Override
    public void setP(double p) {
        pidController.setP(p);
    }

    @Override
    public void setI(double i) {
        pidController.setI(i);
    }

    @Override
    public void setD(double d) {
        pidController.setD(d);
    }

    @Override
    public void setFF(double ff) {
        // pidController.setFF(ff);
    }

    @Override
    public void setkS(double kS) {
        feedforward = new ArmFeedforward(kS, feedforward.kg, feedforward.kv, feedforward.ka);
    }

    @Override
    public void setkG(double kG) {
        feedforward = new ArmFeedforward(feedforward.ks, kG, feedforward.kv, feedforward.ka);
    }

    @Override
    public void setkV(double kV) {
        feedforward = new ArmFeedforward(feedforward.ks, feedforward.kg, kV, feedforward.ka);
    }

    @Override
    public void setkA(double kA) {
        feedforward = new ArmFeedforward(feedforward.ks, feedforward.kg, feedforward.kv, kA);
    }

    @Override 
    public double getkS(){
        return feedforward.ks;
    }

    @Override 
    public double getkG(){
        return feedforward.kg;
    }

    @Override 
    public double getkV(){
        return feedforward.kv;
    }

    @Override 
    public double getkA(){
        return feedforward.ka;
    }

    @Override
    public double getP() {
        return pidController.getP();
    }

    @Override
    public double getI() {
        return pidController.getI();
    }

    @Override
    public double getD() {
        return pidController.getD();
    }

    @Override
    public double getFF() {
        // return pidController.getFF();
        return 0;
    }

}