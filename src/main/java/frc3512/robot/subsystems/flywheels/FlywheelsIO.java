package frc3512.robot.subsystems.flywheels;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface FlywheelsIO {
  public static class FlywheelsIOInputs implements LoggableInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};

    @Override
    public void toLog(LogTable table) {
      table.put("PositionRad", positionRad);
      table.put("VelocityRadPerSec", velocityRadPerSec);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
    }

    @Override
    public void fromLog(LogTable table) {
      positionRad = table.getDouble("PositionRad", positionRad);
      velocityRadPerSec = table.getDouble("VelocityRadPerSec", velocityRadPerSec);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelsIOInputs inputs) {}

  /** Run motor at the specified voltage. */
  public default void setVoltage(double volts) {}
}
