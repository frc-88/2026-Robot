// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.BatteryFuelGauge;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //   private final Drive drive;
  //   public Feeder feeder = new Feeder();
  //   public Shooter shooter = new Shooter();
  //   public Intake intake = new Intake();
  //   public Spinner spinner = new Spinner();
  public BatteryFuelGauge batteryFuelGauge = new BatteryFuelGauge();

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // switch (Constants.currentMode) {
    //   case REAL:
    //     // Real robot, instantiate hardware IO implementations
    //     // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
    //     // a CANcoder
    //     drive =
    //         new Drive(
    //             new GyroIOPigeon2(),
    //             new ModuleIOTalonFX(TunerConstants.FrontLeft),
    //             new ModuleIOTalonFX(TunerConstants.FrontRight),
    //             new ModuleIOTalonFX(TunerConstants.BackLeft),
    //             new ModuleIOTalonFX(TunerConstants.BackRight));
    //     break;

    //   case SIM:
    //     // Sim robot, instantiate physics sim IO implementations
    //     drive =
    //         new Drive(
    //             new GyroIO() {},
    //             new ModuleIOSim(TunerConstants.FrontLeft),
    //             new ModuleIOSim(TunerConstants.FrontRight),
    //             new ModuleIOSim(TunerConstants.BackLeft),
    //             new ModuleIOSim(TunerConstants.BackRight));
    //     break;

    //   default:
    //     // Replayed robot, disable IO implementations
    //     drive =
    //         new Drive(
    //             new GyroIO() {},
    //             new ModuleIO() {},
    //             new ModuleIO() {},
    //             new ModuleIO() {},
    //             new ModuleIO() {});
    //     break;
    // }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureSmartDashboardButtons();
    configureDefaultCommands();
    configureDriverController();
  }

  private void configureSmartDashboardButtons() {
    // SmartDashboard.putData("RunFooter", shooter.runShooter().alongWith(feeder.runFeeder()));
    // SmartDashboard.putData("StopFooter", shooter.stopShooter().alongWith(feeder.stopFeeder()));
    // SmartDashboard.putData("RunShooterFeeder", feeder.runFeeder());
    // SmartDashboard.putData("StopShooterFeeder", feeder.stopFeeder());
    // SmartDashboard.putData("RunShooter", shooter.runShooter());
    // SmartDashboard.putData("StopShooter", shooter.stopShooter());
    // SmartDashboard.putData("RunIntake", intake.runIndexer());
    // SmartDashboard.putData("StopIntake", intake.stopIntake());
    // SmartDashboard.putData("RunSpinner", spinner.runSpinner());
    // SmartDashboard.putData("StopSpinner", spinner.stopSpinner());
    // SmartDashboard.putData("RunHopper", feeder.runFeeder().alongWith(spinner.runSpinner()));
    // SmartDashboard.putData("StopHopper", feeder.stopFeeder().alongWith(spinner.stopSpinner()));
    // SmartDashboard.putData("RunFooter", feeder.runFeeder().alongWith(shooter.runShooter()));
    // SmartDashboard.putData("StopFooter", feeder.stopFeeder().alongWith(shooter.stopShooter()));
    // SmartDashboard.putData(
    //     "Shooter/SysId/Quasistatic Forward", shooter.sysIdQuasistatic(Direction.kForward));
    // SmartDashboard.putData(
    //     "Shooter/SysId/Quasistatic Reverse", shooter.sysIdQuasistatic(Direction.kReverse));
    // SmartDashboard.putData(
    //     "Shooter/SysId/Dynamic Forward", shooter.sysIdDynamic(Direction.kForward));
    // SmartDashboard.putData(
    //     "Shooter/SysId/Dynamic Reverse", shooter.sysIdDynamic(Direction.kReverse));
    // SmartDashboard.putData(
    //     "Feeder/SysId/Quasistatic Forward", feeder.sysIdQuasistatic(Direction.kForward));
    // SmartDashboard.putData(
    //     "Feeder/SysId/Quasistatic Reverse", feeder.sysIdQuasistatic(Direction.kReverse));
    // SmartDashboard.putData("Feeder/SysId/Dynamic Forward",
    // feeder.sysIdDynamic(Direction.kForward));
    // SmartDashboard.putData("Feeder/SysId/Dynamic Reverse",
    // feeder.sysIdDynamic(Direction.kReverse));
  }

  private void configureDefaultCommands() {
    // spinner.setDefaultCommand(spinner.stopSpinner());
    // intake.setDefaultCommand(intake.stopIntake());
    // feeder.setDefaultCommand(feeder.stopFeeder());
    // shooter.setDefaultCommand(shooter.stopShooter());
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> -controller.getLeftY(),
    //         () -> -controller.getLeftX(),
    //         () -> -controller.getRightX()));
    // SmartDashboard.putData("RunShooterVoltage", shooter.runShooterVoltage());
  }

  public void disabledInit() {
    // shooter.resetBPS();
  }

  private void configureDriverController() {
    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> Rotation2d.kZero));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
    //                 drive)
    //             .ignoringDisable(true));
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
