package frc.robot.subsystems.example;
import frc.robot.utils.ShuffleData;

public interface SubsystemIO {

    public static class SubsystemData {
        public ShuffleData<Double> positionUnitsLog = new ShuffleData<Double>("Subsystem", "position units", 0.0);
        public ShuffleData<Double> velocityUnitsLog = new ShuffleData<Double>("Subsystem", "velocity units", 0.0);
        public ShuffleData<Double> inputVoltsLog = new ShuffleData<Double>("Subsystem", "input volts", 0.0);
        public ShuffleData<Double> appliedVoltsLog = new ShuffleData<Double>("Subsystem", "applied volts", 0.0);
        public ShuffleData<Double> currentAmpsLog = new ShuffleData<Double>("Subsystem", "current amps", 0.0);
        public ShuffleData<Double> tempCelciusLog = new ShuffleData<Double>("Subsystem", "temp celcius", 0.0);

      }
    
      /* Updates the set of loggable inputs. */
      public default void updateData(SubsystemData data) { };
    
      /* Run the motor at the specified voltage. */
      public default void setVoltage(double volts) { };
    

      /* Enable or disable brake mode on the  motor. */
      public default void setBrakeMode(boolean enable){ 

      };
    
} 