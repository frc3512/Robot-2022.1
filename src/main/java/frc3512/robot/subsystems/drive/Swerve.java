package frc3512.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.robot.Constants;
import frc3512.robot.subsystems.drive.gyro.GyroIO;
import frc3512.robot.subsystems.drive.gyro.GyroIO.GyroIOInputs;
import frc3512.robot.subsystems.drive.module.ModuleIO;
import frc3512.robot.subsystems.drive.module.SwerveModule;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final GyroIO gyroIO;
  private final GyroIOInputs gyroInputs = new GyroIOInputs();

  private final ModuleIO[] moduleIOs = new ModuleIO[4]; // FL, FR, BL, BR

  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;

  public PIDController controller = new PIDController(0.1, 0.0, 0.0);

  /** Subsystem class for the swerve drive. */
  public Swerve(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    moduleIOs[0] = flModuleIO;
    moduleIOs[1] = frModuleIO;
    moduleIOs[2] = blModuleIO;
    moduleIOs[3] = brModuleIO;

    zeroGyro();
    controller.setTolerance(4.5);

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, flModuleIO),
          new SwerveModule(1, frModuleIO),
          new SwerveModule(2, blModuleIO),
          new SwerveModule(3, brModuleIO)
        };
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void stop() {
    for (SwerveModule mod : mSwerveMods) {
      mod.stop();
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(pose, getYaw());
  }

  public void resetModuleZeros() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void zeroGyro() {
    gyroIO.resetGyro();
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyroInputs.positionDegree)
        : Rotation2d.fromDegrees(gyroInputs.positionDegree);
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getStates());
    gyroIO.updateInputs(gyroInputs);
    Logger.getInstance().processInputs("Swerve/Gyro", gyroInputs);
    for (SwerveModule mod : mSwerveMods) {
      mod.updateModule();
    }
    Logger.getInstance()
        .recordOutput(
            "SwerveOdometry",
            new double[] {
              getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees()
            });
  }
}
