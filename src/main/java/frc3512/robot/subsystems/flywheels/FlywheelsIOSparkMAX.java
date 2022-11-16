package frc3512.robot.subsystems.flywheels;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.robot.Constants;
import frc3512.robot.Constants.Flywheels.FlywheelPose;

public class FlywheelsIOSparkMAX implements FlywheelsIO {
  CANSparkMax m_grbx;
  Encoder m_encoder;

  public FlywheelsIOSparkMAX(FlywheelPose pose) {
    switch (pose) {
      case front:
        m_grbx = new CANSparkMax(Constants.Flywheels.frontMotorID, MotorType.kBrushless);
        m_encoder =
            new Encoder(
                Constants.Flywheels.frontEncoderA,
                Constants.Flywheels.frontEncoderB,
                true,
                EncodingType.k1X);
        m_grbx.restoreFactoryDefaults();
        m_grbx.setSmartCurrentLimit(40);
        m_grbx.set(0.0);
        m_grbx.setInverted(true);
        m_grbx.setIdleMode(IdleMode.kCoast);
        m_grbx.burnFlash();

        m_encoder.setDistancePerPulse(Constants.Flywheels.distancePerPulse);
        m_encoder.setSamplesToAverage(5);

        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_grbx, Usage.kMinimal);
        break;
      case back:
        m_grbx = new CANSparkMax(Constants.Flywheels.backMotorID, MotorType.kBrushless);
        m_encoder =
            new Encoder(
                Constants.Flywheels.backEncoderA,
                Constants.Flywheels.backEncoderB,
                false,
                EncodingType.k1X);
        m_grbx.restoreFactoryDefaults();
        m_grbx.setSmartCurrentLimit(40);
        m_grbx.set(0.0);
        m_grbx.setInverted(true);
        m_grbx.setIdleMode(IdleMode.kCoast);
        m_grbx.burnFlash();

        m_encoder.setDistancePerPulse(Constants.Flywheels.distancePerPulse);
        m_encoder.setSamplesToAverage(5);

        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_grbx, Usage.kMinimal);
        break;
      default:
        break;
    }
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    inputs.positionRad = m_encoder.getDistance();
    inputs.velocityRadPerSec = m_encoder.getRate();
    inputs.appliedVolts = m_grbx.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = new double[] {m_grbx.getOutputCurrent()};
    inputs.tempCelcius = new double[] {m_grbx.getMotorTemperature()};
  }

  @Override
  public void setVoltage(double volts) {
    m_grbx.setVoltage(volts);
  }
}
