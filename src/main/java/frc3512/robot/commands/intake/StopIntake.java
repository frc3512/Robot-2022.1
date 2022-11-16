package frc3512.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.intake.Intake;

public class StopIntake extends CommandBase {
  private final Intake m_intake;

  public StopIntake(Intake intake) {
    this.m_intake = intake;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.stopConveyor();
    m_intake.stopIntake();
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
