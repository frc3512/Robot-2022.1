package frc3512.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc3512.robot.Constants;
import frc3512.robot.commands.driving.TurnInPlace;
import frc3512.robot.commands.intake.AutonIntakeCargo;
import frc3512.robot.commands.intake.DeployIntake;
import frc3512.robot.commands.intake.StopIntake;
import frc3512.robot.commands.shooting.AutoVisionShot;
import frc3512.robot.subsystems.drive.Swerve;
import frc3512.robot.subsystems.flywheels.BackFlywheel;
import frc3512.robot.subsystems.flywheels.FrontFlywheel;
import frc3512.robot.subsystems.intake.Intake;
import frc3512.robot.subsystems.vision.Vision;
import java.util.List;

public class ShootThree extends SequentialCommandGroup {
  public ShootThree(
      Swerve swerve,
      FrontFlywheel frontFlywheel,
      BackFlywheel backFlywheel,
      Intake intake,
      Vision vision) {
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.maxSpeedMetersPerSecond,
                Constants.AutoConstants.maxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);
    TrajectoryConfig config2 =
        new TrajectoryConfig(
                Constants.AutoConstants.maxSpeedMetersPerSecond,
                Constants.AutoConstants.maxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics)
            .setReversed(true);
    Trajectory driveBack =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.0, 4.474, Rotation2d.fromDegrees(0.0)),
            List.of(),
            new Pose2d(0.6, 4.474, Rotation2d.fromDegrees(0.0)),
            config);

    Trajectory getBall =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.5, 0.0, Rotation2d.fromDegrees(0.0)),
            List.of(),
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
            config2);

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.pThetaController,
            0,
            0,
            Constants.AutoConstants.thetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            driveBack,
            swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.pXController, 0, 0),
            new PIDController(Constants.AutoConstants.pYController, 0, 0),
            thetaController,
            () -> Rotation2d.fromDegrees(0.0),
            swerve::setModuleStates,
            swerve);

    SwerveControllerCommand getBallCommand =
        new SwerveControllerCommand(
            getBall,
            swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.pXController, 0, 0),
            new PIDController(Constants.AutoConstants.pYController, 0, 0),
            thetaController,
            () -> Rotation2d.fromDegrees(0.0),
            swerve::setModuleStates,
            swerve);

    addCommands(
        new DeployIntake(intake),
        new InstantCommand(() -> swerve.resetOdometry(driveBack.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> swerve.stop()),
        new AutoVisionShot(swerve, frontFlywheel, backFlywheel, intake, vision),
        new TurnInPlace(swerve, 160.0),
        new AutonIntakeCargo(intake),
        new InstantCommand(() -> swerve.resetOdometry(getBall.getInitialPose())),
        getBallCommand,
        new InstantCommand(() -> swerve.stop()),
        new WaitCommand(0.5),
        new StopIntake(intake),
        new DeployIntake(intake),
        new TurnInPlace(swerve, 0.0),
        new AutoVisionShot(swerve, frontFlywheel, backFlywheel, intake, vision));
  }
}
