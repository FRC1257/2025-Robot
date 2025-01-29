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

    public default double getSetpoint() {return 0.0;}

    /**
     * Gets the elevator's instantaneous height
     * */
    public default double getPosition() {return 0.0;}

    /**
     * returns true if the elevator is at the setpoint
     */
    public default boolean atSetpoint() {return false;}

    /**
     * Sets the elevator's speed
     * @param velocity speed percentage (0 is nothing, 1 is maximum)
     * */
    public default void setSpeed(double speed) {}

    /**
     * Sets the brakemode of both motors
     * @param brakeEnabled true = brake, false = coast
     */
    public default void setBrakeMode(boolean brakeEnabled) {}
    
    /** Only use individual getters and setters for spontaneous changes during runtime. All PID constants are automatically set when the robot starts up*/
    public default double getP() {return 0.0;}
    /** Only use individual getters and setters for spontaneous changes during runtime. All PID constants are automatically set when the robot starts up*/
    public default double getI() {return 0.0;}
    /** Only use individual getters and setters for spontaneous changes during runtime. All PID constants are automatically set when the robot starts up*/
    public default double getD() {return 0.0;}
    /** Only use individual getters and setters for spontaneous changes during runtime. All PID constants are automatically set when the robot starts up*/
    public default double getFF() {return 0.0;}
    
    /** Only use individual getters and setters for spontaneous changes during runtime. All PID constants are automatically set when the robot starts up*/
    public default void setP(double kP) {}
    /** Only use individual getters and setters for spontaneous changes during runtime. All PID constants are automatically set when the robot starts up*/
    public default void setI(double kI) {}
    /** Only use individual getters and setters for spontaneous changes during runtime. All PID constants are automatically set when the robot starts up*/
    public default void setD(double kD) {}
    /** Only use individual getters and setters for spontaneous changes during runtime. All PID constants are automatically set when the robot starts up*/
    public default void setFF(double kFF) {}
}
