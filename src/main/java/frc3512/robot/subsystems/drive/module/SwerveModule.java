package frc3512.robot.subsystems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc3512.lib.math.OnboardModuleState;
import frc3512.robot.Constants;
import frc3512.robot.subsystems.drive.module.ModuleIO.ModuleIOInputs;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  public int moduleNumber;
  private double lastAngle;

  private final ModuleIO moduleIO;
  private final ModuleIOInputs moduleInputs = new ModuleIOInputs();

  /**
   * Creates a new swerve module with NEO motors and a CTRE CANCoder.
   *
   * @param moduleNumber - Number of the module (0-3)
   * @param moduleIO - Module IO for the appropriate module
   */
  public SwerveModule(int moduleNumber, ModuleIO moduleIO) {
    this.moduleNumber = moduleNumber;
    this.moduleIO = moduleIO;

    lastAngle = getState().angle.getDegrees();
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState =
        OnboardModuleState.optimize(
            desiredState,
            getState().angle); // Custom optimize command, since default WPILib optimize assumes
    // continuous controller which REV and CTRE are not

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      moduleIO.setDrivePercent(percentOutput);
    } else {
      moduleIO.setDrivePID(desiredState.speedMetersPerSecond);
    }

    double angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle
                .getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
    // Jittering.
    moduleIO.setTurnPID(angle);
    lastAngle = angle;
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(moduleInputs.turnAbsolutePositionDegree);
  }

  public void stop() {
    moduleIO.stop();
  }

  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - moduleInputs.turnAbsolutePositionOffset;
    moduleIO.setTurnEncoder(absolutePosition);
  }

  public SwerveModuleState getState() {
    double velocity = moduleInputs.driveVelocityMeterPerSec;
    Rotation2d angle = Rotation2d.fromDegrees(moduleInputs.turnPositionDegree);
    return new SwerveModuleState(velocity, angle);
  }

  public void updateModule() {
    moduleIO.updateInputs(moduleInputs);
    Logger.getInstance().processInputs("Swerve/Mod " + moduleNumber, moduleInputs);
  }
}
