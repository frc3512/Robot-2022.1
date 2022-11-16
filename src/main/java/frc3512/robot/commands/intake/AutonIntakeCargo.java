package frc3512.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.intake.Intake;

public class AutonIntakeCargo extends CommandBase {
  Intake m_intake;

  public AutonIntakeCargo(Intake intake) {
    this.m_intake = intake;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intake.runIntake();
    if (!m_intake.isUpperSensorBlocked() && m_intake.isLowerSensorBlocked()) {
      m_intake.setConveyor(false);
    } else {
      m_intake.stopConveyor();
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
