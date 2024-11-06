package frc.robot.subsystems.example;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface SubsystemIO {

    public static class SubsystemData {
        public double position = 0.0;
        public double velocity = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelcius = 0.0;
    
      }
    
      /* Updates the set of loggable inputs. */
      public default void updateData(SubsystemData data) {
      }
    
      /* Run the motor at the specified voltage. */
      public default void setDriveVoltage(double volts) {
      }
    

      /* Enable or disable brake mode on the  motor. */
      public default void setBrakeMode(boolean enable) {
      }
    
      /* Log Data */
      public default void logData(SubsystemData data) {
        SmartDashboard.putNumber("position", data.position);
        SmartDashboard.putNumber("velocity", data.velocity);
        SmartDashboard.putNumber("applied volts", data.appliedVolts);
        SmartDashboard.putNumber("current amps", data.currentAmps);
        SmartDashboard.putNumber("temp celcius", data.tempCelcius);


      }
} 