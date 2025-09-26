package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Methods that are helpful throughout the code base
 * 
 * @author Noah Simon
 */
public class UtilityFunctions {

    public static boolean isRedAlliance() {
        boolean isRed = false;

        if (DriverStation.getAlliance().isEmpty()) {
            return isRed;
        } else {
            isRed = DriverStation.getAlliance().get() == Alliance.Red;
        }
        return isRed;
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

        if (aDouble + marginDouble >= bDouble && aDouble - marginDouble <= bDouble) {
            return true;
        }
        return false;
    }

    /**
     * @param <T>      any numeric type (e.g., Double, Integer). Will automatically
     *                 be inferred.
     * @param velocity
     * @return whether or not the velocity is below 0.01, which we consider to be
     *         stopped
     */
    public static <T extends Number> boolean isStopped(T velocity) {
        return withinMargin(0.01, velocity.doubleValue(), 0.0);
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
        } else {
            return measurementDouble;
        }
    }
}
