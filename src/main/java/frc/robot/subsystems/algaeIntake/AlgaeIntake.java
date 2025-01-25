package frc.robot.subsystems.algaeIntake;


import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AlgaeIntake extends SubsystemBase {
    private final AlgaeIntakeIO io;
    AlgaeIntakeIOInputsAutoLogged inputs = new AlgaeIntakeIOInputsAutoLogged();
    
    private LoggedNetworkNumber logP;
    private LoggedNetworkNumber logI;
    private LoggedNetworkNumber logD;
    
    public AlgaeIntake (AlgaeIntakeIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this);

        logP = new LoggedNetworkNumber("/SmartDashboard/AlgaeIntake/P", io.getP());
        logI = new LoggedNetworkNumber("/SmartDashboard/AlgaeIntake/I", io.getI());
        logD = new LoggedNetworkNumber("/SmartDashboard/AlgaeIntake/D", io.getD());
    }


    public void periodic() {
        io.updateInputs(inputs);
        // Update PID constants to ensure they are up to date
        if(logP.get() != io.getP()) {
            io.setP(logP.get());
        }
        if(logI.get() != io.getI()) {
            io.setI(logI.get());
        }
        if(logD.get() != io.getD()) {
            io.setD(logD.get());
        }
        Logger.processInputs("AlgaeIntake", inputs);
  

        Logger.recordOutput("AlgaeIntake/AIntakeMotorConnected", inputs.velocityRadsPerSec != 0);
    }

    @AutoLogOutput(key = "AlgaeIntake/Close")
    public boolean isVoltageClose(double setVoltage) {
        double voltageDifference = Math.abs(setVoltage - inputs.appliedVoltage);
        return voltageDifference <= AlgaeIntakeConstants.ALGAE_INTAKE_TOLERANCE;
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
        isVoltageClose(voltage);
    }
    
    public void setBrake(boolean brake) {
        io.setBrake(brake);
    }
    
    /**
     * Uses input from controller to set speed of the flywheel
     * and is used as the default command for the intake
    */
    public Command speedCommand(DoubleSupplier speed) {
        return new FunctionalCommand(
            () -> {},
            () -> io.setSpeed(speed.getAsDouble()),
            (stop) -> io.stop(),
            () -> false,
            this
        );
    }
    // Allows manual command of the flywheel for testing
    public Command manualCommand(DoubleSupplier voltage) {
        return new FunctionalCommand(
            () -> {},
            () -> io.setVoltage(voltage.getAsDouble()),
            (stop) -> io.stop(),
            () -> false,
            this
        );
    }

        // Allows manual command of the flywheel for testing
    public Command manualCommand(double voltage) {
        return manualCommand(() -> voltage);
    }

    public Command stop() {
        return new FunctionalCommand(
            () -> {},
            () -> io.setVoltage(0),
            (stop) -> io.stop(),
            () -> false,
            this
        );
    }
}