package frc3512.robot.subsystems.flywheels;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.robot.Constants;
import frc3512.robot.subsystems.flywheels.FlywheelsIO.FlywheelsIOInputs;
import frc3512.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class FrontFlywheel extends SubsystemBase {
  boolean m_atGoal;
  double m_angle;

  boolean m_useRange = false;
  boolean m_isEnabled = true;

  private final FlywheelsIO io;
  private final FlywheelsIOInputs inputs = new FlywheelsIOInputs();

  private final Vision m_vision;

  private final LinearSystem<N1, N1, N1> m_flywheelPlant =
      LinearSystemId.identifyVelocitySystem(
          Constants.Flywheels.frontkV, Constants.Flywheels.frontkA);

  private final KalmanFilter<N1, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N1(), Nat.N1(), m_flywheelPlant, VecBuilder.fill(100.0), VecBuilder.fill(2.5), 0.020);
  private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
      new LinearQuadraticRegulator<>(
          m_flywheelPlant, VecBuilder.fill(25.0), VecBuilder.fill(12.0), 0.020);

  private final LinearSystemLoop<N1, N1, N1> m_loop =
      new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);

  InterpolatingTreeMap<Double, Double> m_table = new InterpolatingTreeMap<Double, Double>();

  /** Subsystem class for the front flywheel. */
  public FrontFlywheel(FlywheelsIO io, Vision vision) {
    this.io = io;
    this.m_vision = vision;

    m_table.put(72.0, 291.2);
    m_table.put(96.0, 291.2);
    m_table.put(120.0, 291.2);
    m_table.put(144.0, 291.2);
    m_table.put(168.0, 291.2);

    reset();
    stop();
  }

  public void reset() {
    m_loop.reset(VecBuilder.fill(getAngle()));
  }

  public void enable() {
    m_isEnabled = true;
  }

  public void disable() {
    m_isEnabled = false;
  }

  public void setGoal(double speed) {
    m_angle = speed;
  }

  public void setGoalFromRange(boolean useRange) {
    m_useRange = useRange;
  }

  public void controllerPeriodic() {

    if (m_isEnabled) {
      if (m_angle > 0.0) {
        m_loop.setNextR(VecBuilder.fill(m_angle));
      } else {
        m_loop.setNextR(VecBuilder.fill(0.0));
      }

      m_loop.correct(VecBuilder.fill(getAngle()));
      m_loop.predict(0.020);

      double nextVoltage = m_loop.getU(0);
      setVoltage(nextVoltage);
      updateAtGoal(m_loop.getError(0));

      if (m_useRange && m_vision.hasTargets()) {
        setGoal(m_table.get(m_vision.getRange()));
      }
    }
  }

  private void updateAtGoal(double error) {
    if (m_atGoal && error > Constants.Flywheels.recoveryThreshold) {
      m_atGoal = false;
    } else if (!m_atGoal && error < Constants.Flywheels.shotThreshold) {
      m_atGoal = true;
    }
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void stop() {
    setGoal(0.0);
  }

  public double getGoal() {
    return m_loop.getNextR(0);
  }

  public boolean atGoal() {
    return m_atGoal;
  }

  public double getAngle() {
    return inputs.velocityRadPerSec;
  }

  public boolean isEnabled() {
    return m_isEnabled;
  }

  @Override
  public void periodic() {
    controllerPeriodic();
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("FrontFlywheel", inputs);
    Logger.getInstance().recordOutput("FrontFlywheelSetpoint", getGoal());
    Logger.getInstance().recordOutput("FrontFlywheelAtGoal", atGoal());
    Logger.getInstance().recordOutput("FrontFlywheelUseRange", m_useRange);
  }
}
