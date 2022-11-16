package frc3512.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.robot.Constants;

public class IntakeIOSparkMAX implements IntakeIO {
  CANSparkMax m_conveyorMotor;
  CANSparkMax m_intakeMotor;
  CANSparkMax m_miniArmMotor;

  DigitalInput m_upperSensor;
  DigitalInput m_lowerSensor;

  Solenoid m_fourbar;

  public IntakeIOSparkMAX() {
    m_conveyorMotor = new CANSparkMax(Constants.Intake.conveyorMotorID, MotorType.kBrushless);
    m_intakeMotor = new CANSparkMax(Constants.Intake.armMotorID, MotorType.kBrushless);
    m_miniArmMotor = new CANSparkMax(Constants.Intake.miniArmMotorID, MotorType.kBrushless);

    m_upperSensor = new DigitalInput(Constants.Intake.upperSensorChannel);
    m_lowerSensor = new DigitalInput(Constants.Intake.lowerSensorChannel);

    m_fourbar = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Intake.armChannel);

    CANSparkMaxUtil.setCANSparkMaxBusUsage(m_miniArmMotor, Usage.kMinimal);
    m_miniArmMotor.setSmartCurrentLimit(80);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(m_intakeMotor, Usage.kMinimal);
    m_intakeMotor.setSmartCurrentLimit(80);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(m_conveyorMotor, Usage.kMinimal);
    m_conveyorMotor.setSmartCurrentLimit(80);

    m_miniArmMotor.enableVoltageCompensation(Constants.General.voltageComp);
    m_intakeMotor.enableVoltageCompensation(Constants.General.voltageComp);
    m_conveyorMotor.enableVoltageCompensation(Constants.General.voltageComp);

    m_miniArmMotor.burnFlash();
    m_intakeMotor.burnFlash();
    m_conveyorMotor.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.extended = m_fourbar.get();
    inputs.upperTriggered = m_upperSensor.get();
    inputs.lowerTriggered = m_lowerSensor.get();

    inputs.appliedVoltsCM =
        m_conveyorMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.appliedVoltsIM = m_intakeMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.appliedVoltsMAM =
        m_miniArmMotor.getAppliedOutput() * RobotController.getBatteryVoltage();

    inputs.currentAmps =
        new double[] {
          m_conveyorMotor.getOutputCurrent(),
          m_intakeMotor.getOutputCurrent(),
          m_miniArmMotor.getOutputCurrent()
        };
    inputs.tempCelcius =
        new double[] {
          m_conveyorMotor.getMotorTemperature(),
          m_intakeMotor.getMotorTemperature(),
          m_miniArmMotor.getMotorTemperature()
        };
  }

  @Override
  public void setConveyorMotorPercent(double percent) {
    m_conveyorMotor.set(percent);
  }

  @Override
  public void setIntakeMotorPercent(double percent) {
    m_intakeMotor.set(percent);
  }

  @Override
  public void setMiniArmMotorPercent(double percent) {
    m_miniArmMotor.set(percent);
  }

  @Override
  public void setExtended(boolean extended) {
    m_fourbar.set(extended);
  }
}
