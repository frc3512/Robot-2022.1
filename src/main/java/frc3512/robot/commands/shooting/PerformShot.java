package frc3512.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc3512.robot.commands.flywheels.StopShooters;
import frc3512.robot.commands.intake.RunConveyors;
import frc3512.robot.subsystems.flywheels.BackFlywheel;
import frc3512.robot.subsystems.flywheels.FrontFlywheel;
import frc3512.robot.subsystems.intake.Intake;

public class PerformShot extends SequentialCommandGroup {
  public PerformShot(FrontFlywheel frontFlywheel, BackFlywheel backFlywheel, Intake intake) {
    addCommands(new RunConveyors(intake), new StopShooters(frontFlywheel, backFlywheel));
  }
}
