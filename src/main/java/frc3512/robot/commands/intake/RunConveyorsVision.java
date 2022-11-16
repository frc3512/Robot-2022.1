package frc3512.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.intake.Intake;
import frc3512.robot.subsystems.vision.Vision;

public class RunConveyorsVision extends CommandBase {
  private final Vision m_vision;
  private final Intake m_intake;
  private final Timer m_timer = new Timer();

  public RunConveyorsVision(Vision vision, Intake intake) {
    this.m_vision = vision;
    this.m_intake = intake;
    addRequirements(m_vision, m_intake);
  }

  @Override
  public void initialize() {
    if (m_vision.hasTargets()) {
      m_timer.reset();
      m_timer.start();
    }
  }

  @Override
  public void execute() {
    if (m_vision.hasTargets()) {
      m_intake.setConveyor(true);
    }
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
