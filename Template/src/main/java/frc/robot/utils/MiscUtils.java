package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import java.io.ObjectInputFilter.Config;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.RobotType;

/**
 * @author FRC 3749
 * @author Neel Adem
 */
public class MiscUtils {
    // cleaner utility class to help make switching between real and sim configs
    // easier
    public static class ControlConfig {
        public double kP;
        public double kI;
        public double kD;
        public double kG;
        public double kS;
        public double kV;
        public double kA;

        public ControlConfig kP(double val) {
            kP = val;
            return this;
        }

        public ControlConfig kI(double val) {
            kI = val;
            return this;
        }

        public ControlConfig kD(double val) {
            kD = val;
            return this;
        }

        public ControlConfig kG(double val) {
            kG = val;
            return this;
        }

        public ControlConfig kS(double val) {
            kS = val;
            return this;
        }

        public ControlConfig kV(double val) {
            kV = val;
            return this;
        }

        public ControlConfig kA(double val) {
            kA = val;
            return this;
        }
    }

    public static class ControlConfigBuilder {
        ControlConfig realConfig = new ControlConfig();
        ControlConfig simConfig = new ControlConfig();

        public ControlConfig real() {
            return realConfig;
        }
        public ControlConfig sim() {
            return simConfig;
        }

        public ControlConfig get() {
            if (MiscUtils.getRobotType() == RobotType.REAL) {
                return realConfig;
            } else {
                return simConfig;
            }
        }
    }

    /***
     * @param <T>   any numeric type (e.g., Double, Integer). Will automatically be
     *              inferred.
     * @param input the value to flip
     * @return the flipped value
     */
    public static <T extends Number> double fieldFlipY(T input) {
        double value = input.doubleValue();
        double height = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getFieldWidth();

        System.out.println("FIELD FLIP Y IS USING 2025 FIELD DIMENSIONS. UPDATE IF NEEDED.");

        return height - value;
    }

    /***
     * Clamps the input voltage to the nominal bus voltage defined in RobotConfig.
     * 
     * @param <T>   any numeric type (e.g., Double, Integer). Will automatically be
     *              inferred.
     * @param input the voltage to clamp
     * @return the clamped voltage
     */
    public static <T extends Number> double voltageClamp(T input) {
        double value = input.doubleValue();

        if (Math.abs(value) > RobotConfig.GENERAL.NOMINAL_BUS_VOLTAGE) {
            return Math.copySign(RobotConfig.GENERAL.NOMINAL_BUS_VOLTAGE, value);
        }

        return value;
    }

    /***
     * @param <T>    any numeric type (e.g., Double, Integer). Will automatically be
     *               inferred.
     * @param margin how close the values need to be to return true. Use a positive
     *               number
     * @param a      the first number
     * @param b      the second number
     * @return true if it is within the margin, false if not
     */
    public static <T extends Number> boolean withinMargin(T margin, T a, T b) {
        double marginDouble = margin.doubleValue();
        double aDouble = a.doubleValue();
        double bDouble = b.doubleValue();

        return aDouble + marginDouble >= bDouble && aDouble - marginDouble <= bDouble;
    }

    public static boolean isStopped(LinearVelocity velocity) {
        return withinMargin(RobotConfig.ACCURACY.DEFAULT_MOVEMENT_TOLERANCE.in(MetersPerSecond),
                velocity.in(MetersPerSecond),
                0.0);
    }

    /**
     * @param <T>      any numeric type (e.g., Double, Integer). Will automatically
     *                 be inferred.
     * @param velocity
     * @return whether or not the velocity is below the default min speed, which we
     *         consider to be
     *         stopped
     */
    public static <T extends Number> boolean isStopped(T velocity) {
        return isStopped(velocity, RobotConfig.ACCURACY.DEFAULT_MOVEMENT_TOLERANCE.in(MetersPerSecond));
    }

    /**
     * @param <T>      any numeric type (e.g., Double, Integer). Will automatically
     *                 be inferred.
     * @param velocity
     * @param minSpeed
     * @return whether or not the velocity is below the minimum speed to be
     *         considered stopped
     */
    public static <T extends Number> boolean isStopped(T velocity, T minSpeed) {
        return withinMargin(minSpeed, velocity, 0.0);
    }

    /**
     * @param <T>         any numeric type (e.g., Double, Integer). Will
     *                    automatically be inferred.
     * @param measurement
     * @param deadband
     * @return whether or not the velocity is below the minimum speed to be
     *         considered stopped
     */
    public static <T extends Number> double signedDeadband(T measurement, T deadband) {
        double measurementDouble = measurement.doubleValue();
        double deadbandDouble = deadband.doubleValue();

        if (Math.abs(measurementDouble) < deadbandDouble) {
            return 0.0;
        }
        return measurementDouble;
    }

    public static RobotType getRobotType() {
        if (Robot.isReal()) {
            return RobotType.REAL;
        } else if (Robot.isSimulation()) {
            return RobotType.SIM;
        } else {
            throw new IllegalStateException("[MiscUtils.java] Unknown Robot Type. Something has gone HORRIBLY wrong.");
        }
    }

    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Red);
    }

    public static boolean isSimulation() {
        return getRobotType() == RobotType.SIM;
    }

    public static boolean isReal() {
        return getRobotType() == RobotType.REAL;
    }

    public static boolean isReplay() {
        return RobotConfig.REPLAY_MODE && isSimulation();
    }

    public static boolean isBlueAlliance() {
        return getAlliance() == Alliance.Blue;
    }

    public static boolean isRedAlliance() {
        return getAlliance() == Alliance.Red;
    }
}
