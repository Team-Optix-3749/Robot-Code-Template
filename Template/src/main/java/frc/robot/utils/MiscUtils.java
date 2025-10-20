package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.config.RobotConfig.RobotType;

/**
 * Methods that are helpful throughout the code base
 * 
 * @author Noah Simon
 */
public class MiscUtils {

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance()
                .map(alliance -> alliance == Alliance.Red)
                .orElse(false);
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
        }
        return measurementDouble;
    }

    public static RobotType getRobotType() {
        if (Robot.isReal()) {
            return RobotType.REAL;
        } else if (Robot.isSimulation()) {
            return RobotType.SIM;
        } else {
            return RobotType.REPLAY;
        }
    }
}
