// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Intake.ChangeIsExtended;
import frc.robot.commands.Intake.Eject;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.ShootInHub;
import frc.robot.commands.Shooter.ChangeTestingBackingRatio;
import frc.robot.commands.Shooter.ChangeTestingSpeed;
import frc.robot.commands.Shooter.ShootAtSpeed;
import frc.robot.commands.Shooter.ShootAtTestingSpeed;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIO;
import frc.robot.subsystems.Indexer.IndexerIOSim;
import frc.robot.subsystems.Indexer.IndexerIOSparkFlex;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeIOSparkFlex;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.Shooter.ShooterIOSparkFlex;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive m_drive;
  private final Vision m_vision;
  private final Intake m_intake;
  private final Shooter m_shooter;
  private final Indexer m_indexer;

  // Controller
  private final CommandGenericHID m_driverController =
      new CommandGenericHID(OIConstants.kDriverControllerPort);

  private final CommandGenericHID m_operatorController =
      new CommandGenericHID(OIConstants.kOperatorControllerPort);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        m_intake = new Intake(new IntakeIOSparkFlex());
        m_shooter = new Shooter(new ShooterIOSparkFlex());
        m_vision = new Vision(m_drive::addVisionMeasurement);
        m_indexer = new Indexer(new IndexerIOSparkFlex());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        m_intake = new Intake(new IntakeIOSim());
        m_shooter = new Shooter(new ShooterIOSim());
        m_vision = new Vision(m_drive::addVisionMeasurement); // TODO: update vision for sim
        m_indexer = new Indexer(new IndexerIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        m_drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        m_intake = new Intake(new IntakeIO() {});
        m_shooter = new Shooter(new ShooterIO() {});
        m_vision = new Vision(m_drive::addVisionMeasurement);
        m_indexer = new Indexer(new IndexerIO() {});
        break;
    }

    // Named Comands for PathPlanner autos
    // ex: NamedCommands.registerCommand("InitilizeTurretLeft", new TurretInitilize(m_turret,
    // false));
    NamedCommands.registerCommand("IntakeIn", new IntakeIn(m_intake));
    NamedCommands.registerCommand("PivotIn", new ChangeIsExtended(m_intake, false));
    NamedCommands.registerCommand("PivotOut", new ChangeIsExtended(m_intake, true));
    NamedCommands.registerCommand("ShootInHub", new ShootInHub(m_indexer, m_shooter));
    NamedCommands.registerCommand("SpinUpShooter", new ShootAtSpeed(m_shooter));

    NamedCommands.registerCommand(
        "FixAngle",
        DriveCommands.joystickDriveAtAngle(
            m_drive,
            () -> 0,
            () -> 0,
            () ->
                Rotation2d.fromDegrees(SmartDashboard.getNumber("Robot Angle to Target Hub", 0))));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData("PivotOut", new ChangeIsExtended(m_intake, true));
    SmartDashboard.putData("PivotIn", new ChangeIsExtended(m_intake, false));
    SmartDashboard.putData("IntakeIn", new IntakeIn(m_intake));
    SmartDashboard.putData("Eject", new Eject(m_intake));
    SmartDashboard.putData("ShooterTestPointUP", new ChangeTestingSpeed(m_shooter, 100));
    SmartDashboard.putData("ShooterTestPointDOWN", new ChangeTestingSpeed(m_shooter, -100));
    SmartDashboard.putData("ShooterTestPointUPSmall", new ChangeTestingSpeed(m_shooter, 10));
    SmartDashboard.putData("ShooterTestPointDOWNSmall", new ChangeTestingSpeed(m_shooter, -10));
    SmartDashboard.putData("ShooterBackinRatioUP", new ChangeTestingBackingRatio(m_shooter, .1));
    SmartDashboard.putData("ShooterBackinRatioDOWN", new ChangeTestingBackingRatio(m_shooter, -.1));
    SmartDashboard.putData(
        "ShooterBackinRatioUPSmall", new ChangeTestingBackingRatio(m_shooter, .01));
    SmartDashboard.putData(
        "ShooterBackinRatioDOWNSmall", new ChangeTestingBackingRatio(m_shooter, -.01));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    m_drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_drive,
            () -> -m_driverController.getRawAxis(1),
            () -> -m_driverController.getRawAxis(0),
            () -> -m_driverController.getRawAxis(4)));

    // Lock to 0° when Y button is held
    final Trigger JoystickDriveAtZero = m_driverController.button(4);
    JoystickDriveAtZero.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            m_drive,
            () -> -m_driverController.getRawAxis(1),
            () -> -m_driverController.getRawAxis(0),
            () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    final Trigger XPattern = m_driverController.button(3);
    XPattern.onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));

    // Reset gyro to 0° when B button is pressed
    final Trigger ResetHeading = m_driverController.button(2);
    ResetHeading.onTrue(
        Commands.runOnce(
                () ->
                    m_drive.setPose(
                        new Pose2d(m_drive.getPose().getTranslation(), Rotation2d.kZero)),
                m_drive)
            .ignoringDisable(true));

    // Lock to 45° when left trigger is held, for use when crossing the trench
    final Trigger DriveCrossTrench = m_driverController.axisGreaterThan(2, .1);
    DriveCrossTrench.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            m_drive,
            () -> -m_driverController.getRawAxis(1),
            () -> -m_driverController.getRawAxis(0),
            () -> Rotation2d.fromDegrees(45)));

    final Trigger ShootInHub = m_driverController.axisGreaterThan(3, .1);
    ShootInHub.whileTrue(new ShootInHub(m_indexer, m_shooter));
    ShootInHub.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            m_drive,
            () -> -m_driverController.getRawAxis(1) * Constants.kDriveShootingRatio,
            () -> -m_driverController.getRawAxis(0) * Constants.kDriveShootingRatio,
            () ->
                Rotation2d.fromDegrees(SmartDashboard.getNumber("Robot Angle to Target Hub", 0))));

    /*
    final Trigger AirMail = m_driverController.axisGreaterThan(5, .3);
    AirMail.whileTrue(new AirMail(m_indexer, m_shooter));
    AirMail.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            m_drive,
            () -> -m_driverController.getRawAxis(1) * Constants.kDriveShootingRatio,
            () -> -m_driverController.getRawAxis(0) * Constants.kDriveShootingRatio,
            () ->
                Rotation2d.fromDegrees(
                    SmartDashboard.getNumber("Robot Angle to Target Air Mail", 0))));
                    */

    final Trigger ShootAtTestingSpeed = m_driverController.button(1);
    ShootAtTestingSpeed.whileTrue(new ShootAtTestingSpeed(m_shooter));

    final Trigger IncreaseTestingPoint = m_driverController.povUp();
    IncreaseTestingPoint.whileTrue(new ChangeTestingSpeed(m_shooter, 100));

    final Trigger DecreaseTestingSpeed = m_driverController.povDown();
    DecreaseTestingSpeed.whileTrue(new ChangeTestingSpeed(m_shooter, -100));

    final Trigger IncreaseTestingSpeedSmall = m_driverController.povRight();
    IncreaseTestingSpeedSmall.whileTrue(new ChangeTestingSpeed(m_shooter, 10));

    final Trigger DecreaseTestingSpeedSmall = m_driverController.povLeft();
    DecreaseTestingSpeedSmall.whileTrue(new ChangeTestingSpeed(m_shooter, -10));

    final Trigger IncreaseBackingRatio = m_driverController.button(6);
    IncreaseBackingRatio.whileTrue(new ChangeTestingBackingRatio(m_shooter, .1));

    final Trigger DecreaseBackingRatio = m_driverController.button(5);
    DecreaseBackingRatio.whileTrue(new ChangeTestingBackingRatio(m_shooter, -.1));

    final Trigger IncreaseBackingRatioSmall = m_driverController.button(8);
    IncreaseBackingRatioSmall.whileTrue(new ChangeTestingBackingRatio(m_shooter, .01));

    final Trigger DecreaseBackingRatioSmall = m_driverController.button(7);
    DecreaseBackingRatioSmall.whileTrue(new ChangeTestingBackingRatio(m_shooter, -.01));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
