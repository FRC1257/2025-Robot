package frc.robot.util.misc;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import lombok.Getter;
import lombok.Setter;

/*Implements a PID Controller loop with scheduled gain parameters */
public class ScheduledController implements Sendable, AutoCloseable {
    private static int instances;

    /* Factors for Proportional Control */
    @Getter @Setter private double[] m_kp;

    /* Factors for Integral Control */
    @Getter @Setter private double[] m_ki;

    /* Factors for Derivative Control */
    @Getter @Setter private double[] m_kd;

    /* The zone in which the integral term is not applied */
    @Getter private double m_iZone = Double.POSITIVE_INFINITY;


    /*Fuzzy Logic States */
    @Getter @Setter private double[] m_fuzzyStates;
    @Getter private int m_fuzzyStateIndex = 0;

    /* The period measured in seconds of the loop which calls the controller */
    private final double m_period;

    @Getter @Setter private double m_maximumIntegral = 1.0;
    @Getter @Setter private double m_minimumIntegral = -1.0;

    @Getter @Setter private double m_maximumInput;

    @Getter @Setter private double m_minimumInput;

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
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;

        for (double k : m_kp) {
            if (k < 0) {
                throw new IllegalArgumentException("Proportional gain must be non-negative");
            }
        }

        for (double k : m_ki) {
            if (k < 0) {
                throw new IllegalArgumentException("Integral gain must be non-negative");
            }
        }

        for (double k : m_kd) {
            if (k < 0) {
                throw new IllegalArgumentException("Derivative gain must be non-negative");
            }
        }

        m_fuzzyStates = fuzzyStates;

        if (m_fuzzyStates.length != m_kp.length || m_fuzzyStates.length != m_ki.length || m_fuzzyStates.length != m_kd.length) {
            throw new IllegalArgumentException("All gain arrays must be of equal length");
        }

        m_period = period;

        if (m_period <= 0) {
            throw new IllegalArgumentException("Period must be greater than zero");
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
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;
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
     * Sets the FuzzyStateIndex based on the current measured position in comparison to the fuzzy states.
     * @param measured the current measured position
     */
    public void setFuzzyStateIndex(double measured) {
        int currentfuzzyStateIndex = 0;
        for (int i = 0; i < m_fuzzyStates.length; i++) {
            if (measured > m_fuzzyStates[i]) {
                currentfuzzyStateIndex = i;
                break;
            }
        }
        m_fuzzyStateIndex = currentfuzzyStateIndex;
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
        setFuzzyStateIndex(measurment);

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
        } else if (m_ki[m_fuzzyStateIndex] != 0) {
            m_totalError =
                MathUtil.clamp(
                    m_totalError + m_error * m_period,
                    m_minimumIntegral / m_ki[m_fuzzyStateIndex],
                    m_maximumIntegral / m_ki[m_fuzzyStateIndex]);
        }

        double percentage = (m_measurement - m_fuzzyStates[m_fuzzyStateIndex]) 
                            / (m_fuzzyStates[m_fuzzyStateIndex + 1] - m_fuzzyStates[m_fuzzyStateIndex]);

        return (m_kp[m_fuzzyStateIndex + 1]*percentage + (1 - percentage)*m_kp[m_fuzzyStateIndex]) * m_error
            + (m_ki[m_fuzzyStateIndex + 1]*percentage + (1 - percentage)*m_ki[m_fuzzyStateIndex]) * m_totalError
            + (m_kd[m_fuzzyStateIndex + 1]*percentage + (1 - percentage)*m_kd[m_fuzzyStateIndex] )* m_errorDerivative;
        
    }

    public void reset() {
        m_error = 0;
        m_errorDerivative = 0;
        m_prevError = 0;
        m_totalError = 0;
        m_haveMeasurement = false;
        m_fuzzyStateIndex = 0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ScheduledController");
        builder.addDoubleArrayProperty("p", this::getM_kp, this::setM_kp);
        builder.addDoubleArrayProperty("i", this::getM_ki, this::setM_ki);
        builder.addDoubleArrayProperty("d", this::getM_kd, this::setM_kd);
        builder.addDoubleProperty("iZone", this::getM_iZone, this::setIZone);
        builder.addDoubleProperty("setpoint", this::getM_setpoint, this::setSetpoint);
        builder.addDoubleProperty("measurement", () -> m_measurement, null);
        builder.addDoubleProperty("error", this::getM_error, null);
        builder.addDoubleProperty("errorDerivative", this::getM_errorDerivative, null);
        builder.addDoubleProperty("prevError", () -> this.m_prevError, null);
        builder.addDoubleProperty("totalError", this::getM_totalError, null);
    }
}
