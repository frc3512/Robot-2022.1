package frc3512.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs {
    public boolean haveTargets = false;
    public double area = 0.0;
    public double yaw = 0.0;
    public double pitch = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("HaveTargets", haveTargets);
      table.put("Area", area);
      table.put("Yaw", yaw);
      table.put("Pitch", pitch);
    }

    @Override
    public void fromLog(LogTable table) {
      haveTargets = table.getBoolean("HaveTargets", haveTargets);
      area = table.getDouble("Area", area);
      yaw = table.getDouble("Yaw", yaw);
      pitch = table.getDouble("Pitch", pitch);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  /** Update the vision values */
  public default void updateVision() {}
}
