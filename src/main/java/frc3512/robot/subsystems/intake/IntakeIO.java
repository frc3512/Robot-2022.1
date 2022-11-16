package frc3512.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeIO {
  public static class IntakeIOInputs implements LoggableInputs {
    public boolean extended = false;
    public boolean upperTriggered = false;
    public boolean lowerTriggered = false;

    public double appliedVoltsCM = 0.0;
    public double appliedVoltsIM = 0.0;
    public double appliedVoltsMAM = 0.0;

    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};

    @Override
    public void toLog(LogTable table) {
      table.put("Extended", extended);
      table.put("UpperTriggered", upperTriggered);
      table.put("LowerTriggered", lowerTriggered);

      table.put("AppliedVoltsCM", appliedVoltsCM);
      table.put("AppliedVoltsIM", appliedVoltsIM);
      table.put("AppliedVoltsMAM", appliedVoltsMAM);

      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
    }

    @Override
    public void fromLog(LogTable table) {
      extended = table.getBoolean("Extended", extended);
      upperTriggered = table.getBoolean("UpperTriggered", upperTriggered);
      lowerTriggered = table.getBoolean("LowerTriggered", lowerTriggered);

      appliedVoltsCM = table.getDouble("AppliedVoltsCM", appliedVoltsCM);
      appliedVoltsIM = table.getDouble("AppliedVoltsIM", appliedVoltsIM);
      appliedVoltsMAM = table.getDouble("AppliedVoltsMAM", appliedVoltsMAM);

      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Sets the speed of the conveyor motor as a percent. */
  public default void setConveyorMotorPercent(double percent) {}

  /** Sets the speed of the intake motor as a percent. */
  public default void setIntakeMotorPercent(double percent) {}

  /** Sets the speed of the mini arm motor as a percent. */
  public default void setMiniArmMotorPercent(double percent) {}

  /** Set solenoid state. */
  public default void setExtended(boolean extended) {}
}
