package frc3512.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputs inputs = new ClimberIOInputs();
  public final int kSwitchConstant = 3000;
  boolean m_ignoreLimits = false;

  /** Subsystem class for the climber */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  public void setClimbers(double leftSpeed, double rightSpeed) {
    if (m_ignoreLimits) {
      io.setLeftClimberPercent(leftSpeed);
      io.setRightClimberPercent(rightSpeed);
    } else {
      if (!hasLeftPassedTopLimit() || leftSpeed > 0) {
        io.setLeftClimberPercent(leftSpeed);
      } else {
        io.setLeftClimberPercent(0.0);
      }

      if (!hasRightPassedTopLimit() || rightSpeed > 0) {
        io.setRightClimberPercent(rightSpeed);
      } else {
        io.setRightClimberPercent(0.0);
      }
    }
  }

  public void deployClimbers() {
    io.setExtended(true);
  }

  public void stowClimbers() {
    io.setExtended(false);
  }

  public boolean areDeployed() {
    return inputs.extended;
  }

  public void overrideLimits() {
    m_ignoreLimits = true;
  }

  public boolean hasRightPassedTopLimit() {
    return (inputs.rightSensorValue < kSwitchConstant);
  }

  public boolean hasLeftPassedTopLimit() {
    return (inputs.leftSensorValue < kSwitchConstant);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Climber", inputs);
    Logger.getInstance().recordOutput("ClimberLeftSwitchPassed", hasLeftPassedTopLimit());
    Logger.getInstance().recordOutput("ClimberRightSwitchPassed", hasRightPassedTopLimit());
  }
}
