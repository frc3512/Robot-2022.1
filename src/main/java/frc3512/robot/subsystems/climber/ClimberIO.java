package frc3512.robot.subsystems.climber;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ClimberIO {
  public static class ClimberIOInputs implements LoggableInputs {
    public boolean extended = false;
    public double leftSensorValue = 0.0;
    public double rightSensorValue = 0.0;

    public double appliedVoltsLeft = 0.0;
    public double appliedVoltsRight = 0.0;

    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};

    @Override
    public void toLog(LogTable table) {
      table.put("Extended", extended);
      table.put("LeftSensorValue", leftSensorValue);
      table.put("RightSensorValue", rightSensorValue);
      table.put("AppliedVoltsLeft", appliedVoltsLeft);
      table.put("AppliedVoltsRight", appliedVoltsRight);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
    }

    @Override
    public void fromLog(LogTable table) {
      extended = table.getBoolean("Extended", extended);
      leftSensorValue = table.getDouble("LeftSensorValue", leftSensorValue);
      rightSensorValue = table.getDouble("RightSensorValue", rightSensorValue);
      appliedVoltsLeft = table.getDouble("AppliedVoltsLeft", appliedVoltsLeft);
      appliedVoltsRight = table.getDouble("AppliedVoltsRight", appliedVoltsRight);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Sets the speed of the left motor as a percent. */
  public default void setLeftClimberPercent(double percent) {}

  /** Sets the speed of the right motor as a percent. */
  public default void setRightClimberPercent(double percent) {}

  /** Set solenoid state. */
  public default void setExtended(boolean extended) {}
}
