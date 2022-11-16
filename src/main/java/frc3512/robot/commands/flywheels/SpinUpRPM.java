package frc3512.robot.commands.flywheels;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.flywheels.BackFlywheel;
import frc3512.robot.subsystems.flywheels.FrontFlywheel;

public class SpinUpRPM extends CommandBase {

  private final FrontFlywheel m_frontFlywheel;
  private final BackFlywheel m_backFlywheel;
  private final double m_frontRPM, m_backRPM;

  public SpinUpRPM(
      FrontFlywheel frontFlywheel, BackFlywheel backFlywheel, double frontRPM, double backRPM) {
    this.m_frontFlywheel = frontFlywheel;
    this.m_backFlywheel = backFlywheel;
    this.m_frontRPM = frontRPM;
    this.m_backRPM = backRPM;
    addRequirements(m_frontFlywheel, m_backFlywheel);
  }

  @Override
  public void initialize() {
    m_frontFlywheel.reset();
    m_backFlywheel.reset();
    m_frontFlywheel.setGoal(m_frontRPM);
    m_backFlywheel.setGoal(m_backRPM);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_frontFlywheel.atGoal() && m_backFlywheel.atGoal();
  }
}
