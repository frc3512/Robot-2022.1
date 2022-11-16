package frc3512.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.robot.subsystems.vision.VisionIO.VisionIOInputs;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

public class Vision extends SubsystemBase {

  private final VisionIO io;
  private final VisionIOInputs inputs = new VisionIOInputs();

  private static double targetHeightMeters = 2.606;
  private static double cameraHeightMeters = 0.76835;
  private static double cameraPitchDegrees = 39.6;
  private static double yawOffsetDegrees = 3.5;

  /** Subsystem class for the vision camera. */
  public Vision(VisionIO io) {
    this.io = io;
  }

  public boolean hasTargets() {
    return inputs.haveTargets;
  }

  public double getArea() {
    return inputs.area;
  }

  public double getPitch() {
    return inputs.pitch;
  }

  public double getYaw() {
    return inputs.yaw;
  }

  public double getOffsetedYaw() {
    var adjustedYaw = 0.0;
    if (getYaw() < 0.0) {
      adjustedYaw = getYaw() - yawOffsetDegrees;
    } else if (getYaw() > 0.0) {
      adjustedYaw = getYaw() + yawOffsetDegrees;
    } else {
      adjustedYaw = getYaw();
    }
    return adjustedYaw;
  }

  public double getRange() {
    if (!inputs.haveTargets) {
      return 0;
    }
    var range =
        PhotonUtils.calculateDistanceToTargetMeters(
            cameraHeightMeters,
            targetHeightMeters,
            Units.degreesToRadians(cameraPitchDegrees),
            Units.degreesToRadians(getPitch()));
    return range;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updateVision();
    Logger.getInstance().processInputs("Vision", inputs);
    Logger.getInstance().recordOutput("VisionRange", getRange());
  }
}
