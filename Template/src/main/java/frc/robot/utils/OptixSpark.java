package frc.robot.utils;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Lightweight wrapper around SparkBase/SparkMax/SparkFlex providing
 * fluent configuration, telemetry, and simple open/closed-loop helpers
 * with optional position wrapping and deadband behavior.
 */
public final class OptixSpark {
    private final SparkBase motor;
    private final SparkClosedLoopController ctrl;
    private SparkBaseConfig cfg;

    // Position wrapping and deadband (local logic + mirrored in controller config)
    private boolean wrapEnabled = false;
    private double wrapMin = 0.0;
    private double wrapMax = 0.0;
    private double positionDeadband = 0.0;

    /**
     * Creates an OptixSpark around an existing SparkBase.
     * 
     * @param motor underlying Spark device
     * @throws IllegalArgumentException if the motor type is unsupported
     */
    private OptixSpark(SparkBase motor) {
        this.motor = motor;
        this.ctrl = motor.getClosedLoopController();
        if (motor instanceof SparkMax) {
            this.cfg = new SparkMaxConfig();
        } else if (motor instanceof SparkFlex) {
            this.cfg = new SparkFlexConfig();
        } else {
            throw new IllegalArgumentException("Unsupported motor type" + motor.getClass().getName());
        }
    }

    /**
     * Factory for a SparkMax (brushless).
     * 
     * @param id CAN device ID
     * @return new OptixSpark wrapping a SparkMax
     */
    public static OptixSpark ofSparkMax(int id) {
        return new OptixSpark(new SparkMax(id, MotorType.kBrushless));
    }

    /**
     * Factory for a SparkFlex (brushless).
     * 
     * @param id CAN device ID
     * @return new OptixSpark wrapping a SparkFlex
     */
    public static OptixSpark ofSparkFlex(int id) {
        return new OptixSpark(new SparkFlex(id, MotorType.kBrushless));
    }

    // Accessors

    /**
     * @return underlying Spark device
     */
    public SparkBase getSpark() {
        return motor;
    }

    /**
     * @return closed-loop controller for the motor
     */
    public SparkClosedLoopController getController() {
        return ctrl;
    }

    /**
     * @return absolute encoder attached to the motor
     */
    public SparkAbsoluteEncoder getAbsoluteEncoder() {
        return motor.getAbsoluteEncoder();
    }

    /**
     * @return relative encoder of the motor
     */
    public RelativeEncoder getEncoder() {
        return motor.getEncoder();
    }

    /**
     * @return current mutable configuration
     */
    public SparkBaseConfig getConfig() {
        return cfg;
    }

    // Telemetry

    /**
     * Gets the current position, applying wrapping if enabled.
     * 
     * @return position in configured units
     */
    public double getPosition() {
        double p = motor.getEncoder().getPosition();
        return wrapEnabled ? wrap(p) : p;
    }

    /**
     * @return velocity in configured units per second
     */
    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    /**
     * @return applied output voltage (bus voltage times applied output)
     */
    public double getAppliedVolts() {
        return motor.getBusVoltage() * motor.getAppliedOutput();
    }

    /**
     * @return bus voltage supplied to the motor controller
     */
    public double getBusVolts() {
        return motor.getBusVoltage();
    }

    /**
     * @return output current in amps
     */
    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    /**
     * @return motor temperature in Celsius
     */
    public double getTemperature() {
        return motor.getMotorTemperature();
    }

    // Zeroing

    /**
     * Sets the encoder's current position.
     * 
     * @param position new position in configured units
     */
    public void setPosition(double position) {
        motor.getEncoder().setPosition(position);
    }

    // Open-loop control

    /**
     * Sets open-loop duty cycle [-1, 1].
     * 
     * @param dutyCycle normalized output
     */
    public void setDutyCycle(double dutyCycle) {
        motor.set(dutyCycle);
    }

    /**
     * Sets open-loop voltage.
     * 
     * @param volts desired output voltage
     */
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    // Velocity closed-loop

    /**
     * Requests velocity control using slot 0 and zero feedforward.
     * 
     * @param velocitySetpoint target velocity
     */
    public void requestVelocity(double velocitySetpoint) {
        requestVelocity(velocitySetpoint, 0.0, ClosedLoopSlot.kSlot0);
    }

    /**
     * Requests velocity control on slot 0 with feedforward.
     * 
     * @param velocitySetpoint target velocity
     * @param feedforward      arbitrary feedforward (volts)
     */
    public void requestVelocity(double velocitySetpoint, double feedforward) {
        requestVelocity(velocitySetpoint, feedforward, ClosedLoopSlot.kSlot0);
    }

    /**
     * Requests velocity control on a specific slot.
     * 
     * @param velocitySetpoint target velocity
     * @param feedforward      arbitrary feedforward (volts)
     * @param slot             closed-loop slot to use
     */
    public void requestVelocity(double velocitySetpoint, double feedforward, ClosedLoopSlot slot) {
        ctrl.setReference(velocitySetpoint, ControlType.kVelocity, slot, feedforward);
    }

    // Position closed-loop with deadband-to-slot3 behavior by default

    /**
     * Requests position control using slot 0 normally, slot 3 if within deadband.
     * 
     * @param positionSetpoint target position (wrapping applied if enabled)
     */
    public void requestPosition(double positionSetpoint) {
        requestPosition(positionSetpoint, 0.0);
    }

    /**
     * Requests position control; uses slot 3 if target is within deadband.
     * 
     * @param positionSetpoint target position (wrapping applied if enabled)
     * @param feedforward      arbitrary feedforward (volts)
     */
    public void requestPosition(double positionSetpoint, double feedforward) {
        double sp = wrapEnabled ? wrap(positionSetpoint) : positionSetpoint;
        ClosedLoopSlot slot = inDeadband(getPosition(), sp) ? ClosedLoopSlot.kSlot3 : ClosedLoopSlot.kSlot0;
        ctrl.setReference(sp, ControlType.kPosition, slot, feedforward);
    }

    /**
     * Requests position control on an explicit slot.
     * Deadband logic is not applied.
     * 
     * @param positionSetpoint target position (wrapping applied if enabled)
     * @param feedforward      arbitrary feedforward (volts)
     * @param slot             closed-loop slot to use
     */
    public void requestPosition(double positionSetpoint, double feedforward, ClosedLoopSlot slot) {
        double sp = wrapEnabled ? wrap(positionSetpoint) : positionSetpoint;
        ctrl.setReference(sp, ControlType.kPosition, slot, feedforward);
    }

    // Configuration helpers (fluent)

    // Units

    /**
     * Sets the position conversion factor for the relative encoder.
     * 
     * @param factor units per encoder rotation
     * @return this for chaining
     */
    public OptixSpark setPositionConversionFactor(double factor) {
        cfg.apply(new EncoderConfig().positionConversionFactor(factor));
        return this;
    }

    /**
     * Sets the velocity conversion factor for the relative encoder.
     * 
     * @param factor units per encoder RPM
     * @return this for chaining
     */
    public OptixSpark setVelocityConversionFactor(double factor) {
        cfg.apply(new EncoderConfig().velocityConversionFactor(factor));
        return this;
    }

    // Current limit

    /**
     * Configures smart current limits.
     * 
     * @param stallLimit stall current limit (A)
     * @param freeLimit  free current limit (A)
     * @return this for chaining
     */
    public OptixSpark setSmartCurrentLimit(int stallLimit, int freeLimit) {
        cfg.smartCurrentLimit(stallLimit, freeLimit);
        return this;
    }

    // PID per slot

    /**
     * Sets PID gains for a given slot.
     * 
     * @param slot closed-loop slot
     * @param p    proportional gain
     * @param i    integral gain
     * @param d    derivative gain
     * @return this for chaining
     */
    public OptixSpark setPID(ClosedLoopSlot slot, double p, double i, double d) {
        cfg.apply(new ClosedLoopConfig().pid(p, i, d, slot));
        return this;
    }

    /**
     * Selects the feedback sensor used for closed-loop control.
     * 
     * @param sensor feedback sensor
     * @return this for chaining
     */
    public OptixSpark useFeedbackSensor(FeedbackSensor sensor) {
        cfg.apply(new ClosedLoopConfig().feedbackSensor(sensor));
        return this;
    }

    /**
     * Enables or disables position wrapping and sets the input range.
     * If min equals max, wrapping is disabled.
     * 
     * @param min minimum of wrap range
     * @param max maximum of wrap range
     * @return this for chaining
     */
    public OptixSpark setPositionWrapping(double min, double max) {
        if (min == max) {
            this.wrapEnabled = false;
            cfg.apply(new ClosedLoopConfig().positionWrappingEnabled(false));
            return this;
        }
        double a = Math.min(min, max);
        double b = Math.max(min, max);
        this.wrapEnabled = true;
        this.wrapMin = a;
        this.wrapMax = b;

        cfg.apply(
                new ClosedLoopConfig()
                        .positionWrappingEnabled(true)
                        .positionWrappingInputRange(a, b));
        return this;
    }

    /**
     * Sets the deadband width for position requests.
     * If error is within deadband, slot 3 is used.
     * 
     * @param deadband absolute error threshold in position units
     * @return this for chaining
     */
    public OptixSpark setPositionDeadband(double deadband) {
        this.positionDeadband = Math.max(0.0, deadband);
        return this;
    }

    // Inversion and idle mode

    /**
     * Sets motor inversion.
     * 
     * @param inverted true to invert motor output
     * @return this for chaining
     */
    public OptixSpark setInverted(boolean inverted) {
        cfg.inverted(inverted);
        return this;
    }

    /**
     * Sets absolute encoder inversion.
     * 
     * @param inverted true to invert absolute encoder
     * @return this for chaining
     */
    public OptixSpark setAbsoluteEncoderInverted(boolean inverted) {
        cfg.absoluteEncoder.inverted(inverted);
        return this;
    }

    /**
     * Sets idle mode (brake/coast).
     * 
     * @param mode idle mode
     * @return this for chaining
     */
    public OptixSpark setIdleMode(IdleMode mode) {
        cfg.idleMode(mode);
        return this;
    }

    /**
     * Configures this controller to follow another.
     * 
     * @param leader leader OptixSpark
     * @return this for chaining
     */
    public OptixSpark follow(OptixSpark leader) {
        cfg.follow(leader.getSpark());
        return this;
    }

    /**
     * Applies the accumulated configuration to hardware, safely resetting
     * parameters and persisting them to flash.
     */
    public void apply() {
        motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ---- Internal helpers ----

    /**
     * Checks if the current-to-setpoint error is within the deadband.
     * 
     * @param current  current position
     * @param setpoint target position
     * @return true if within deadband, false otherwise
     */
    private boolean inDeadband(double current, double setpoint) {
        double err = positionError(current, setpoint);
        return Math.abs(err) <= positionDeadband;
    }

    /**
     * Computes signed position error, using the shortest path when wrapping.
     * 
     * @param current  current position
     * @param setpoint target position
     * @return signed error
     */
    private double positionError(double current, double setpoint) {
        if (!wrapEnabled)
            return setpoint - current;

        double errorBound = (wrapMax - wrapMin) / 2.0;
        return wrap(setpoint - current, -errorBound, errorBound);
    }

    /**
     * Wraps a value into the configured range [wrapMin, wrapMax).
     * 
     * @param value input position
     * @return wrapped position
     */
    private double wrap(double value) {
        return wrap(value, wrapMin, wrapMax);
    }

    /**
     * Wraps a value into the configured range [wrapMin, wrapMax).
     * 
     * @param value input position
     * @param min   minimum of wrap range
     * @param max   maximum of wrap range
     * @return wrapped position
     */
    private double wrap(double value, double min, double max) {
        double wrapped = value;

        double modulus = max - min;

        // Wrap input if it's above the maximum input
        double numMax = ((value - min) / modulus);
        wrapped -= numMax * modulus;

        // Wrap input if it's below the minimum input
        double numMin = ((value - min) / modulus);
        wrapped -= numMin * modulus;

        return wrapped;
    }
}