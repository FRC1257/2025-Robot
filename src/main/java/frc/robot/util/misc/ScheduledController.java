package frc.robot.util.misc;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import lombok.Getter;
import lombok.Setter;

/*Implements a PID Controller loop with scheduled gain parameters */
public class ScheduledController implements Sendable, AutoCloseable {
    private static int instances;

    @Getter private InterpolatingDoubleTreeMap m_kpMap = new InterpolatingDoubleTreeMap();
    @Getter private InterpolatingDoubleTreeMap m_kiMap = new InterpolatingDoubleTreeMap();
    @Getter private InterpolatingDoubleTreeMap m_kdMap = new InterpolatingDoubleTreeMap();

    /* The zone in which the integral term is not applied */
    @Getter private double m_iZone = Double.POSITIVE_INFINITY;


    /*Fuzzy Logic States */
    @Getter @Setter private double[] m_fuzzyStates;

    /* The period measured in seconds of the loop which calls the controller */
    private final double m_period;

    private double m_maximumIntegral = 1.0;
    private double m_minimumIntegral = -1.0;

    private double m_maximumInput;
    private double m_minimumInput;

    @Getter private boolean m_continuous;

    @Getter private double m_error;
    @Getter private double m_errorDerivative;

    @Getter private double m_prevError;

    @Getter private double m_totalError;

    @Getter @Setter private double m_errorTolerance = 0.05;
    @Getter @Setter private double m_errorDerivativeTolerance = Double.POSITIVE_INFINITY;

    @Getter private double m_setpoint;
    @Getter private double m_measurement;

    @Getter private boolean m_haveMeasurement;
    @Getter private boolean m_haveSetpoint;


    /**
     * Allocates a ScheduledController with the given PID gains and default period of 0.02 seconds.
     * 
     * @param kp proportional gain parameters
     * @param ki integral gain parameters
     * @param kd derivative gain parameters
     * @param fuzzyStates fuzzy logic states for the controller
     * @throws IllegalArgumentException if the lengths of the kp, ki, kd, and fuzzyStates arrays are not equal
     * @throws IllegalArgumentException if any kp &lt; 0
     * @throws IllegalArgumentException if any ki &lt; 0
     * @throws IllegalArgumentException if any kd &lt; 0
     */
    public ScheduledController(double[] kp, double[] ki, double[] kd, double[] fuzzyStates) {
        this(kp, ki, kd, fuzzyStates, 0.02);
    }

    /**
     * Allocates a ScheduledController with the given PID gains and period.
     * 
     * @param kp proportional gain parameters
     * @param ki integral gain parameters
     * @param kd derivative gain parameters
     * @param fuzzyStates fuzzy logic states for the controller
     * @param period the period of the loop which calls the controller
     * @throws IllegalArgumentException if the lengths of the kp, ki, kd, and fuzzyStates arrays are not equal
     * @throws IllegalArgumentException if any kp &lt; 0
     * @throws IllegalArgumentException if any ki &lt; 0
     * @throws IllegalArgumentException if any kd &lt; 0
     * @throws IllegalArgumentException if period &lt;= 0
     */
    @SuppressWarnings("this-escape")
    public ScheduledController(double[] kp, double[] ki, double[] kd, double[] fuzzyStates, double period) {

        if (fuzzyStates.length != kp.length || fuzzyStates.length != ki.length || fuzzyStates.length != kd.length) {
            throw new IllegalArgumentException("All gain arrays must be of equal length");
        }

        m_fuzzyStates = fuzzyStates;

        for (int i = 0; i < m_fuzzyStates.length; i++) {
            if (kp[i] < 0 || ki[i] < 0 || kd[i] < 0) {
                throw new IllegalArgumentException("All gain values must be non-negative");
            }
        }

        if (period <= 0) {
            throw new IllegalArgumentException("Period must be greater than zero");
        }

        m_period = period;



        for(int i = 0; i < m_fuzzyStates.length; i++) {
            m_kpMap.put(m_fuzzyStates[i], kp[i]);
            m_kiMap.put(m_fuzzyStates[i], ki[i]);
            m_kdMap.put(m_fuzzyStates[i], kd[i]);
        }
        

        instances++;
        SendableRegistry.addLW(this, "ScheduledController", instances);

        MathSharedStore.reportUsage(MathUsageId.kController_PIDController2, instances);

        
    }

    @Override
    public void close() throws Exception {
        SendableRegistry.remove(this);
    }

    /**
     * Sets ScheduledController gain parameters
     * 
     * @param kp proportional gain parameters
     * @param ki integral gain parameters
     * @param kd derivative gain parameters
     */
    public void setPID(double[] kp, double[] ki, double[] kd) {
        for(int i = 0; i < m_fuzzyStates.length; i++) {
            m_kpMap.put(m_fuzzyStates[i], kp[i]);
            m_kiMap.put(m_fuzzyStates[i], ki[i]);
            m_kdMap.put(m_fuzzyStates[i], kd[i]);
        }
    }

    /**
     * Sets the IZone range
     * @param iZone the IZone range
     * @throws IllegalArgumentException if iZone &lt; 0
     */
    public void setIZone(double iZone) {
        if (iZone < 0) {
            throw new IllegalArgumentException("IZone must be non-negative");
        }
        m_iZone = iZone;
    }

    /**
     * Sets the setpoint for the ScheduledController
     * 
     * @param setpoint the desired setpoint
     */
    public void setSetpoint(double setpoint) {
        m_setpoint = setpoint;
        m_haveSetpoint = true;

        if (m_continuous) {
            double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
            m_error = MathUtil.inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
        } else {
            m_error = m_setpoint - m_measurement;
        }

        m_errorDerivative = (m_error - m_prevError) / m_period;
    }

    /**
     * Returns true if the error is within tollerance of the setpoint. The error tolerance defaults to 0.05
     * while the error derivative tolerance defaults to {@link Double.POSITIVE_INFINITY}.
     * 
     * @return whether the error is within acceptable bounds
     */
    public boolean atSetpoint() {
        return m_haveMeasurement
            && m_haveSetpoint
            && Math.abs(m_error) < m_errorTolerance
            && Math.abs(m_errorDerivative) < m_errorDerivativeTolerance;
    }

    /**
     * Enables continuous input for the ScheduledController.
     * 
     * @param minimumInput the minimum input value
     * @param maximumInput the maximum input value
     * @throws IllegalArgumentException if minimumInput is greater than or equal to maximumInput
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        if (minimumInput >= maximumInput) {
            throw new IllegalArgumentException("Minimum input must be less than maximum input");
        }
        m_minimumInput = minimumInput;
        m_maximumInput = maximumInput;
        m_continuous = true;
    }

    /**
     * Disables continuous input for the ScheduledController.
     */
    public void disableContinuousInput() {
        m_continuous = false;
    }

    /**
     * Sets the minimum and maximum contributions of the integral term
     * 
     * @param minimum the minimum integral value
     * @param maximum the maximum integral value
     * @throws IllegalArgumentException if minimum is greater than or equal to maximum
     */
    public void setIntegratorRange(double minimum, double maximum) {
        if (minimum >= maximum) {
            throw new IllegalArgumentException("Minimum integral value must be less than maximum integral value");
        }
        m_minimumIntegral = minimum;
        m_maximumIntegral = maximum;
    }

    /**
    * Sets the error which is considered tolerable for use with atSetpoint().
    *
    * @param errorTolerance Error which is tolerable.
    */
    public void setTolerance(double errorTolerance) {
        setTolerance(errorTolerance, Double.POSITIVE_INFINITY);
    }

    /**
    * Sets the error which is considered tolerable for use with atSetpoint().
    *
    * @param errorTolerance Error which is tolerable.
    * @param errorDerivativeTolerance Error derivative which is tolerable.
    */
    public void setTolerance(double errorTolerance, double errorDerivativeTolerance) {
        m_errorTolerance = errorTolerance;
        m_errorDerivativeTolerance = errorDerivativeTolerance;
    }

    /**
     * Returns the next output of the ScheduledController based on the current measurement and setpoint.
     * @param measurement the current measurement
     * @param setpoint the desired setpoint
     * @return the output of the ScheduledController
     */
    public double calculate(double measurment, double setpoint) {
        m_setpoint = setpoint;
        m_haveMeasurement = true;
        return calculate(measurment);
    }

    /**
     * Returns the next output of the ScheduledController based on the current measurement.
     * @param measurement the current measurement
     * @return the output of the ScheduledController
     */
    public double calculate(double measurment){
        m_measurement = measurment;
        m_haveMeasurement = true;
        m_prevError = m_error;

        if (m_continuous) {
            double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
            m_error = MathUtil.inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
        } else {
            m_error = m_setpoint - m_measurement;
        }
      
        m_errorDerivative = (m_error - m_prevError) / m_period;

        if(Math.abs(m_error) > m_iZone) {
            m_totalError = 0;
        } else if (m_kiMap.get(m_measurement) != 0) {
            m_totalError =
                MathUtil.clamp(
                    m_totalError + m_error * m_period,
                    m_minimumIntegral / m_kiMap.get(m_measurement),
                    m_maximumIntegral / m_kiMap.get(m_measurement));
        }

        return m_kpMap.get(m_measurement)* m_error
            + m_kiMap.get(m_measurement)* m_totalError
            + m_kdMap.get(m_measurement) * m_errorDerivative;
        
    }

    /**
     * Resets the ScheduledController to its initial state.
     */
    public void reset() {
        m_error = 0;
        m_errorDerivative = 0;
        m_prevError = 0;
        m_totalError = 0;
        m_haveMeasurement = false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ScheduledController");
        builder.addDoubleProperty("p", () -> getM_kpMap().get(m_measurement), null);
        builder.addDoubleProperty("i", () -> getM_kiMap().get(m_measurement), null);
        builder.addDoubleProperty("d", () -> getM_kdMap().get(m_measurement), null);
        builder.addDoubleProperty("iZone", this::getM_iZone, this::setIZone);
        builder.addDoubleProperty("setpoint", this::getM_setpoint, this::setSetpoint);
        builder.addDoubleProperty("measurement", () -> m_measurement, null);
        builder.addDoubleProperty("error", this::getM_error, null);
        builder.addDoubleProperty("errorDerivative", this::getM_errorDerivative, null);
        builder.addDoubleProperty("prevError", () -> this.m_prevError, null);
        builder.addDoubleProperty("totalError", this::getM_totalError, null);
    }
}
