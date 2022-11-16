package frc3512.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();
  Timer conveyorTimer;

  /** Subsystem class for the intake. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void deploy() {
    io.setExtended(true);
  }

  public void stow() {
    io.setExtended(false);
  }

  public boolean isDeployed() {
    return inputs.extended;
  }

  public void runIntake() {
    io.setIntakeMotorPercent(0.8);
    io.setMiniArmMotorPercent(-0.8);
  }

  public void runOuttake() {
    io.setIntakeMotorPercent(-0.8);
    io.setMiniArmMotorPercent(0.8);
  }

  public void stopIntake() {
    io.setIntakeMotorPercent(0.0);
    io.setMiniArmMotorPercent(0.0);
  }

  public void setConveyor(boolean ignoreSensors) {
    if (ignoreSensors) {
      io.setConveyorMotorPercent(0.45);
    } else {
      io.setConveyorMotorPercent(0.8);
    }
  }

  public void setConveyorOuttake() {
    io.setConveyorMotorPercent(-0.8);
  }

  public void stopConveyor() {
    io.setConveyorMotorPercent(0.0);
  }

  public boolean isConveyorRunning() {
    return inputs.appliedVoltsCM > 0.0;
  }

  public boolean isUpperSensorBlocked() {
    return !inputs.upperTriggered;
  }

  public boolean isLowerSensorBlocked() {
    return !inputs.lowerTriggered;
  }

  public void resetTimer() {
    conveyorTimer.reset();
    conveyorTimer.start();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);
  }
}
