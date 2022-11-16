package frc3512.robot.commands.flywheels;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.flywheels.BackFlywheel;
import frc3512.robot.subsystems.flywheels.FrontFlywheel;

public class StopShooters extends CommandBase {

  private final FrontFlywheel m_frontFlywheel;
  private final BackFlywheel m_backFlywheel;

  public StopShooters(FrontFlywheel frontFlywheel, BackFlywheel backFlywheel) {
    this.m_frontFlywheel = frontFlywheel;
    this.m_backFlywheel = backFlywheel;
    addRequirements(m_frontFlywheel, m_backFlywheel);
  }

  @Override
  public void initialize() {
    m_frontFlywheel.setGoalFromRange(false);
    m_backFlywheel.setGoalFromRange(false);
    m_frontFlywheel.stop();
    m_backFlywheel.stop();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
