package frc3512.robot.commands.driving;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.drive.Swerve;

public class TurnInPlace extends CommandBase {
  private final Swerve s_Swerve;
  private final PIDController m_controller;
  private final double headingGoal;

  public TurnInPlace(Swerve swerve, double goal) {
    this.s_Swerve = swerve;
    this.headingGoal = goal;
    m_controller = new PIDController(0.1, 0.0, 0.0);
    m_controller.setTolerance(4.5);
    m_controller.enableContinuousInput(0.0, 360.0);
    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() {
    m_controller.reset();
  }

  @Override
  public void execute() {
    double rotationSpeed = -m_controller.calculate(s_Swerve.getYaw().getDegrees(), headingGoal);
    s_Swerve.drive(new Translation2d(), rotationSpeed, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    s_Swerve.drive(new Translation2d(), 0.0, true, true);
  }

  @Override
  public boolean isFinished() {
    return m_controller.atSetpoint();
  }
}
