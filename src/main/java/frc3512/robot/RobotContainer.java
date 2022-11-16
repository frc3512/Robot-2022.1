package frc3512.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc3512.robot.Constants.Flywheels.FlywheelPose;
import frc3512.robot.autos.ShootThree;
import frc3512.robot.autos.ShootTwo;
import frc3512.robot.commands.climbers.DeployClimbers;
import frc3512.robot.commands.climbers.RunClimbers;
import frc3512.robot.commands.driving.TeleopSwerve;
import frc3512.robot.commands.flywheels.SpinUpRPM;
import frc3512.robot.commands.flywheels.StopShooters;
import frc3512.robot.commands.intake.DeployIntake;
import frc3512.robot.commands.intake.IntakeCargo;
import frc3512.robot.commands.intake.OuttakeCargo;
import frc3512.robot.commands.shooting.AutoVisionShot;
import frc3512.robot.commands.shooting.PerformShot;
import frc3512.robot.subsystems.climber.Climber;
import frc3512.robot.subsystems.climber.ClimberIOSparkMAX;
import frc3512.robot.subsystems.drive.Swerve;
import frc3512.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc3512.robot.subsystems.drive.module.ModuleIOSparkMAX;
import frc3512.robot.subsystems.flywheels.BackFlywheel;
import frc3512.robot.subsystems.flywheels.FlywheelsIOSparkMAX;
import frc3512.robot.subsystems.flywheels.FrontFlywheel;
import frc3512.robot.subsystems.intake.Intake;
import frc3512.robot.subsystems.intake.IntakeIOSparkMAX;
import frc3512.robot.subsystems.vision.Vision;
import frc3512.robot.subsystems.vision.VisionIOPhotonVision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Auton Chooser
  private final SendableChooser<Command> m_autonChooser = new SendableChooser<Command>();

  // Robot subsystems
  private Climber m_climber;
  private Intake m_intake;
  private Swerve m_swerve;
  private Vision m_vision;
  private FrontFlywheel m_frontFlywheel;
  private BackFlywheel m_backFlywheel;

  // Xbox controllers
  private final Joystick driver = new Joystick(Constants.Joysticks.xboxControllerPort);
  private final XboxController appendageStick =
      new XboxController(Constants.Joysticks.appendageControllerPort);

  // Drive Controls
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Driver Buttons
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton robotCentric =
      new JoystickButton(driver, XboxController.Button.kRightStick.value);

  // Climber buttons
  private final JoystickButton m_deployClimbersButton =
      new JoystickButton(appendageStick, XboxController.Button.kLeftBumper.value);
  private final JoystickButton m_overrideLimitsButton =
      new JoystickButton(appendageStick, XboxController.Button.kB.value);

  // Intake buttons
  private final Trigger m_deployIntakeButton =
      new JoystickButton(appendageStick, XboxController.Button.kRightBumper.value);
  private final JoystickButton m_intakeButton =
      new JoystickButton(appendageStick, XboxController.Button.kA.value);
  private final JoystickButton m_outtakeButton =
      new JoystickButton(appendageStick, XboxController.Button.kX.value);

  // Shooting buttons
  private final JoystickButton m_8footShot =
      new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton m_visionShot =
      new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton m_killShooters =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton m_performShot =
      new JoystickButton(driver, XboxController.Button.kY.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.General.getMode() != Constants.RunningMode.REPLAY) {
      switch (Constants.General.getRobot()) {
        case ROBOT_2022_REAL:
          m_swerve =
              new Swerve(
                  new GyroIOPigeon2(),
                  new ModuleIOSparkMAX(0),
                  new ModuleIOSparkMAX(1),
                  new ModuleIOSparkMAX(2),
                  new ModuleIOSparkMAX(3));
          m_intake = new Intake(new IntakeIOSparkMAX());
          m_climber = new Climber(new ClimberIOSparkMAX());
          m_vision = new Vision(new VisionIOPhotonVision());
          m_frontFlywheel =
              new FrontFlywheel(new FlywheelsIOSparkMAX(FlywheelPose.front), m_vision);
          m_backFlywheel = new BackFlywheel(new FlywheelsIOSparkMAX(FlywheelPose.back), m_vision);
          break;
        case ROBOT_2022_SIM:
          break;
        default:
          break;
      }
    }

    configureButtonBindings();
    configureAxisActions();
    registerAutons();
  }

  /** Enable the flywheel controllers. */
  public void enableFlywheels() {
    m_frontFlywheel.enable();
    m_backFlywheel.enable();
  }

  /** Disable the flywheel controllers. */
  public void disableFlywheels() {
    m_frontFlywheel.disable();
    m_backFlywheel.disable();
  }

  /** Actions we do in disabled mode. */
  public void disabledActions() {
    m_swerve.resetModuleZeros();
  }

  /** Used for defining button actions. */
  private void configureButtonBindings() {

    // Climber buttons
    m_deployClimbersButton.whenActive(new DeployClimbers(m_climber));
    m_overrideLimitsButton.whenPressed(new InstantCommand(m_climber::overrideLimits, m_climber));

    // Intake buttons
    m_deployIntakeButton.whenActive(new DeployIntake(m_intake));
    m_intakeButton.whenHeld(new IntakeCargo(m_intake, false));
    m_outtakeButton.whenHeld(new OuttakeCargo(m_intake));

    /* Driver Buttons */
    zeroGyro.whenPressed(new InstantCommand(() -> m_swerve.zeroGyro()));

    // Shooting buttons
    m_8footShot.whenPressed(
        new SpinUpRPM(
            m_frontFlywheel,
            m_backFlywheel,
            Constants.Flywheels.shoot8FootFront,
            Constants.Flywheels.shoot8FootBack));
    m_visionShot.whenPressed(
        new AutoVisionShot(m_swerve, m_frontFlywheel, m_backFlywheel, m_intake, m_vision));
    m_performShot.whenPressed(new PerformShot(m_frontFlywheel, m_backFlywheel, m_intake));
    m_killShooters.whenPressed(new StopShooters(m_frontFlywheel, m_backFlywheel));
  }

  /** Used for joystick/xbox axis actions. */
  private void configureAxisActions() {
    m_swerve.setDefaultCommand(
        new TeleopSwerve(
            m_swerve,
            () -> driver.getRawAxis(translationAxis),
            () -> driver.getRawAxis(strafeAxis),
            () -> driver.getRawAxis(rotationAxis),
            () -> robotCentric.get()));

    m_climber.setDefaultCommand(
        new RunClimbers(
            m_climber,
            () ->
                MathUtil.applyDeadband(
                        -appendageStick.getRightY(), Constants.General.climberDeadband)
                    * 0.8,
            () ->
                MathUtil.applyDeadband(
                        -appendageStick.getLeftY(), Constants.General.climberDeadband)
                    * 0.78));
  }

  /** Register the autonomous modes to the chooser for the drivers to select. */
  public void registerAutons() {

    // Register autons.
    m_autonChooser.setDefaultOption("No-op", new InstantCommand());
    m_autonChooser.addOption(
        "Shoot Two", new ShootTwo(m_swerve, m_frontFlywheel, m_backFlywheel, m_intake, m_vision));
    m_autonChooser.addOption(
        "Shoot Three",
        new ShootThree(m_swerve, m_frontFlywheel, m_backFlywheel, m_intake, m_vision));

    // Push the chooser to the dashboard.
    SmartDashboard.putData("Auton Chooser", m_autonChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Get the selected auton from the chooser.
    return m_autonChooser.getSelected();
  }
}
