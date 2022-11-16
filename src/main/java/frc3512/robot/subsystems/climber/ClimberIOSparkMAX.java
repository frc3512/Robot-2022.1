package frc3512.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.robot.Constants;

public class ClimberIOSparkMAX implements ClimberIO {

  CANSparkMax m_leftGrbx;
  CANSparkMax m_rightGrbx;

  AnalogInput m_leftClimberSwitch;
  AnalogInput m_rightClimberSwitch;

  Solenoid m_solenoid;

  public ClimberIOSparkMAX() {
    m_leftGrbx = new CANSparkMax(Constants.Climber.leftClimberID, MotorType.kBrushless);
    m_rightGrbx = new CANSparkMax(Constants.Climber.rightClimberID, MotorType.kBrushless);
    m_leftClimberSwitch = new AnalogInput(Constants.Climber.leftMagneticSwitch);
    m_rightClimberSwitch = new AnalogInput(Constants.Climber.rightMagenticSwitch);
    m_solenoid =
        new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Climber.climberSolenoidChannel);

    CANSparkMaxUtil.setCANSparkMaxBusUsage(m_leftGrbx, Usage.kPositionOnly);
    m_leftGrbx.setSmartCurrentLimit(40);
    m_leftGrbx.enableVoltageCompensation(Constants.General.voltageComp);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(m_rightGrbx, Usage.kPositionOnly);
    m_rightGrbx.setSmartCurrentLimit(40);
    m_rightGrbx.enableVoltageCompensation(Constants.General.voltageComp);

    m_leftGrbx.burnFlash();
    m_rightGrbx.burnFlash();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.extended = m_solenoid.get();
    inputs.leftSensorValue = m_leftClimberSwitch.getValue();
    inputs.rightSensorValue = m_rightClimberSwitch.getValue();
    inputs.appliedVoltsLeft = m_leftGrbx.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.appliedVoltsRight = m_rightGrbx.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps =
        new double[] {m_leftGrbx.getOutputCurrent(), m_rightGrbx.getOutputCurrent()};
    inputs.tempCelcius =
        new double[] {m_leftGrbx.getMotorTemperature(), m_rightGrbx.getMotorTemperature()};
  }

  @Override
  public void setLeftClimberPercent(double percent) {
    m_leftGrbx.set(percent);
  }

  @Override
  public void setRightClimberPercent(double percent) {
    m_rightGrbx.set(percent);
  }

  @Override
  public void setExtended(boolean extended) {
    m_solenoid.set(extended);
  }
}
