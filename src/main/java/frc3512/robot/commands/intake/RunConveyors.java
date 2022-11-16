package frc3512.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.intake.Intake;

public class RunConveyors extends CommandBase {
  private final Intake m_intake;
  private final Timer m_timer = new Timer();

  public RunConveyors(Intake intake) {
    this.m_intake = intake;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    m_intake.setConveyor(true);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_intake.stopConveyor();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(1.5);
  }
}
