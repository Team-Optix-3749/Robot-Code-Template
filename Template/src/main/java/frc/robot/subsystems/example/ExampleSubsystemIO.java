package frc.robot.subsystems.example;

/**
 *              Example subsystsem IO interfaces
 * @author Noah Simon
 */
public interface ExampleSubsystemIO {

  public static class SubsystemData {
    public double positionUnits = 0;
    public double velocityUnits = 0;
    public double accelerationUnits = 0;
    public double inputVolts = 0;
    public double appliedVolts = 0;
    public double currentAmps = 0;
    public double tempCelcius = 0;

  }

  /* Updates the set of loggable inputs. */
  public default void updateData(SubsystemData data) {
  };

  /* Run the motor at the specified voltage. */
  public default void setVoltage(double volts) {
  };

  /* Enable or disable brake mode on the motor. */
  public default void setBrakeMode(boolean enable) {

  };

}
