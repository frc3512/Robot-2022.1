package frc3512.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc3512.robot.commands.driving.VisionAim;
import frc3512.robot.commands.flywheels.SpinUpVision;
import frc3512.robot.commands.flywheels.StopShooters;
import frc3512.robot.commands.intake.RunConveyorsVision;
import frc3512.robot.subsystems.drive.Swerve;
import frc3512.robot.subsystems.flywheels.BackFlywheel;
import frc3512.robot.subsystems.flywheels.FrontFlywheel;
import frc3512.robot.subsystems.intake.Intake;
import frc3512.robot.subsystems.vision.Vision;

public class AutoVisionShot extends SequentialCommandGroup {
  public AutoVisionShot(
      Swerve swerve,
      FrontFlywheel frontFlywheel,
      BackFlywheel backFlywheel,
      Intake intake,
      Vision vision) {
    addCommands(
        new ParallelCommandGroup(
            new VisionAim(swerve, vision), new SpinUpVision(frontFlywheel, backFlywheel)),
        new WaitCommand(0.5),
        new RunConveyorsVision(vision, intake),
        new StopShooters(frontFlywheel, backFlywheel));
  }
}
