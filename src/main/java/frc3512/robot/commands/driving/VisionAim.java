package frc3512.robot.commands.driving;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.drive.Swerve;
import frc3512.robot.subsystems.vision.Vision;

public class VisionAim extends CommandBase {
  private final Swerve s_Swerve;
  private final Vision m_vision;
  private final Timer timer = new Timer();

  public VisionAim(Swerve swerve, Vision vision) {
    this.s_Swerve = swerve;
    this.m_vision = vision;

    addRequirements(s_Swerve, vision);
  }

  @Override
  public void initialize() {
    timer.reset();
  }

  @Override
  public void execute() {
    if (m_vision.hasTargets()) {
      timer.start();
      double rotationSpeed = -s_Swerve.controller.calculate(m_vision.getOffsetedYaw(), 0);
      s_Swerve.drive(new Translation2d(), rotationSpeed, true, true);
    } else {
      s_Swerve.drive(new Translation2d(), 0.0, true, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    s_Swerve.drive(new Translation2d(), 0.0, true, true);
  }

  @Override
  public boolean isFinished() {
    return s_Swerve.controller.atSetpoint() || timer.hasElapsed(1.0);
  }
}
