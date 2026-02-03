// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Drive drive;
  public Feeder shooterFeeder = new Feeder();
  public Shooter shooter = new Shooter();
  public Intake intake = new Intake();
  public Spinner spinner = new Spinner();
  public Climber climber = new Climber();

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureSmartDashboardButtons();
    configureDefaultCommands();
    configureDriverController();
  }

  private void configureSmartDashboardButtons() {
    SmartDashboard.putData("RunShooterFeeder", shooterFeeder.runFeeder());
    SmartDashboard.putData("StopShooterFeeder", shooterFeeder.stopFeeder());
    SmartDashboard.putData("RunShooter", shooter.runShooter());
    SmartDashboard.putData("StopShooter", shooter.stopShooter());
    SmartDashboard.putData("RunIntake", intake.runIndexer());
    SmartDashboard.putData("StopIntake", intake.stopIntake());
    SmartDashboard.putData("RunSpinner", spinner.runSpinner());
    SmartDashboard.putData("StopSpinner", spinner.stopSpinner());
    SmartDashboard.putData("RunHopper", shooterFeeder.runFeeder().alongWith(spinner.runSpinner()));
    SmartDashboard.putData(
        "StopHopper", shooterFeeder.stopFeeder().alongWith(spinner.stopSpinner()));
    SmartDashboard.putData("RunFooter", shooterFeeder.runFeeder().alongWith(shooter.runShooter()));
    SmartDashboard.putData(
        "StopFooter", shooterFeeder.stopFeeder().alongWith(shooter.stopShooter()));
    SmartDashboard.putData(
        "Shooter/SysId/Quasistatic Forward", shooter.sysIdQuasistatic(Direction.kForward));
    SmartDashboard.putData(
        "Shooter/SysId/Quasistatic Reverse", shooter.sysIdQuasistatic(Direction.kReverse));
    SmartDashboard.putData(
        "Shooter/SysId/Dynamic Forward", shooter.sysIdDynamic(Direction.kForward));
    SmartDashboard.putData(
        "Shooter/SysId/Dynamic Reverse", shooter.sysIdDynamic(Direction.kReverse));
  }

  private void configureDefaultCommands() {
    spinner.setDefaultCommand(spinner.stopSpinner());
    intake.setDefaultCommand(intake.stopIntake());
    shooterFeeder.setDefaultCommand(shooterFeeder.stopFeeder());
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    // SmartDashboard.putData("RunShooterVoltage", shooter.runShooterVoltage());
  }

  public void disabledInit() {
    shooter.resetBPS();
  }

  private void configureDriverController() {
    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
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
