package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorInputs {
        public double setPoint = 0.0;
        public double height = 0.0;
        public double velocity = 0.0;
        public double appliedVoltage = 0.0;

        //arrays are used because we have multiple motors
        public double[] motorTemperature = new double[]{};
        public double[] motorCurrent = new double[]{};
    }

    /**
     * This function updates all the loggable inputs inside the ElevatorInputs object.
     * @param inputs an instance of ElevatorInputsAutoLogged that is created in Elevator.java
     */
    public default void updateInputs(ElevatorInputs inputs) {}

    /** 
     * Tells the elevator to start moving towards a setpoint (height)
     * @param setpoint the target height the elevator is trying to go to
     * */ 
    public default void goToSetpoint(double setpoint) {}

    /**
     * Gets the elevator's instantaneous height
     * */
    public default double getHeight() {return 0.0;}

    /**
     * returns true if the elevator is at the setpoint
     */
    public default boolean atSetpoint() {return false;}

    /**
     * Sets the elevator's speed
     * @param velocity speed percentage (0 is nothing, 1 is maximum)
     * */
    public default void setSpeed(double velocity) {}

    /**
     * Sets the brakemode of both motors
     * @param brakeEnabled true = brake, false = coast
     */
    public default void setBrakeMode(boolean brakeEnabled) {}
    
    //PID getters and setters
    public default double getP() {return 0.0;}
    public default double getI() {return 0.0;}
    public default double getD() {return 0.0;}
    public default double getFF() {return 0.0;}

    public default void setP() {}
    public default void setI() {}
    public default void setD() {}
    public default void setFF() {}
}
