package frc3512.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc3512.lib.config.SwerveModuleConstants;

/** Constants for the robot project */
public final class Constants {

  /** Robot mode enum. */
  public static enum RobotMode {
    ROBOT_2022_REAL,
    ROBOT_2022_SIM
  }

  /** Running mode enum. */
  public static enum RunningMode {
    REAL,
    REPLAY,
    SIM
  }

  /** General robot constants */
  public static final class General {
    /** The mode you want to run the robot in. Change it to reflect what you want to run it in. */
    private static final RobotMode robot = RobotMode.ROBOT_2022_REAL;

    // Joystick axis deadband for the climbers
    public static double climberDeadband = 0.05;

    // Joystick axis deadband for the swerve drive
    public static final double swerveDeadband = 0.1;

    // The period at which feedback controllers run
    public static double controllerPeriodic = 0.020;

    // Value to voltage compensate the motors for
    public static final double voltageComp = 12.0;

    /** Get the robot your code is running. */
    public static RobotMode getRobot() {
      if (RobotBase.isReal()) {
        if (robot == RobotMode.ROBOT_2022_SIM) {
          return RobotMode.ROBOT_2022_REAL;
        } else {
          return robot;
        }
      } else {
        return robot;
      }
    }

    /** Get the mode of what you're running the robot in. */
    public static RunningMode getMode() {
      switch (getRobot()) {
        case ROBOT_2022_REAL:
          return RobotBase.isReal() ? RunningMode.REAL : RunningMode.REPLAY;
        case ROBOT_2022_SIM:
          return RunningMode.SIM;
        default:
          return RunningMode.REAL;
      }
    }
  }

  /** Constants revolving around joysticks */
  public static final class Joysticks {
    // Xbox Controller port
    public static final int xboxControllerPort = 0;

    // Appendage xbox controller port
    public static final int appendageControllerPort = 1;
  }

  public static final class Swerve {
    /* Gyro Constants */
    public static final int pigeonID = 6;
    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.73);
    public static final double wheelBase = Units.inchesToMeters(21.73);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (5.14 / 1.0); // 5.14:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.005;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionVelocityFactor =
        ((wheelDiameter * Math.PI) / driveGearRatio) / 60.0;
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double angleConversionFactor = 360.0 / 12.8;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 20;
      public static final int angleMotorID = 10;
      public static final int canCoderID = 1;
      public static final double angleOffset = 55.45;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 21;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 2;
      public static final double angleOffset = 197.402;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 12;
      public static final int angleMotorID = 22;
      public static final int canCoderID = 3;
      public static final double angleOffset = 327.568;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 13;
      public static final int angleMotorID = 23;
      public static final int canCoderID = 4;
      public static final double angleOffset = 154.336;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  /* Auton constants */
  public static final class AutoConstants {
    public static final double maxSpeedMetersPerSecond = 3;
    public static final double maxAccelerationMetersPerSecondSquared = 3;
    public static final double maxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double maxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double pXController = 1.0;
    public static final double pYController = 1.0;
    public static final double pThetaController = 1.0;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints thetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            maxAngularSpeedRadiansPerSecond, maxAngularSpeedRadiansPerSecondSquared);
  }

  /** Constants for the intake subsystem */
  public static final class Intake {
    /// Arm motor CAN ID
    public static final int armMotorID = 50;

    /// Mini arm motor CAN ID
    public static final int miniArmMotorID = 52;

    /// Arm solenoid channel
    public static final int armChannel = 4;

    /// Conveyor motor CAN ID
    public static final int conveyorMotorID = 51;

    /// Lower proximity sensor digial input channel
    public static final int lowerSensorChannel = 3;

    /// Upper proximity sensor digital input channel
    public static final int upperSensorChannel = 2;
  }

  /** Constants for the climber subsystem */
  public static final class Climber {
    public static final int leftClimberID = 40;
    public static final int rightClimberID = 41;

    public static final int leftMagneticSwitch = 0;
    public static final int rightMagenticSwitch = 2;

    public static final int climberSolenoidChannel = 5;
  }

  /** Constants for the flywheel subsystems */
  public static final class Flywheels {

    // Flywheel pose enum
    public static enum FlywheelPose {
      front,
      back
    }

    // Gear ratio for both flywheels
    private static double gearRatio = 1.0 / 1.0;

    // Angle per encoder pulse
    public static double distancePerPulse = (Math.PI * 2.0) * gearRatio / 2048.0;

    // Motor IDs
    public static final int frontMotorID = 30;
    public static final int backMotorID = 31;

    // Front Encoder Inputs
    public static final int frontEncoderA = 4;
    public static final int frontEncoderB = 5;

    // Back Encoder Inputs
    public static final int backEncoderA = 0;
    public static final int backEncoderB = 1;

    // Front SysID gains
    public static final double frontkS = 0.15418;
    public static final double frontkV = 0.020604;
    public static final double frontkA = 0.0027615;

    // Back SysID gains
    public static final double backkS = 0.18295;
    public static final double backkV = 0.020009;
    public static final double backkA = 0.0016076;

    // Controller Thresholds
    public static final double shotThreshold = 18.0;
    public static final double recoveryThreshold = 18.0;

    // 6 Foot Shot
    public static final double shoot6FootFront = 291.2;
    public static final double shoot6FootBack = 175.47;

    // 8 Foot Shot
    public static final double shoot8FootFront = 291.2;
    public static final double shoot8FootBack = 230.0;

    // 10 Foot Shot
    public static final double shoot10FootFront = 291.2;
    public static final double shoot10FootBack = 270.0;
  }
}
