// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Simulation;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Turret;
// import frc.robot.subsystems.Turret;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Batman;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.TrajectorySolver;
import frc.robot.util.Util;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Drive drive;
  private final Turret turret;
  public Feeder feeder = new Feeder();
  public Shooter shooter;
  public Intake intake = new Intake();
  public Spinner spinner = new Spinner();
  public TrajectorySolver trajectorySolver;
  public Batman batman = new Batman();
  public Hood hood;
  public final Vision vision;
  private final Simulation simulation;
  public Climber climber = new Climber();

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private CommandGenericHID buttons = new CommandGenericHID(1);
  //   private LoggedDashboardChooser autoChooser;

  // private Joystick joystick0 = new Joystick(0);
  // private Joystick joystick1 = new Joystick(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    GyroIO gyro;

    // TODO Disable diagnostic server if in COMP mode?
    if (!Util.logif()) {
      Unmanaged.setPhoenixDiagnosticsStartTime(-1);
    }
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        gyro = new GyroIOPigeon2();
        drive =
            new Drive(
                gyro,
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation));
        simulation = null;
        // turret = new Turret(gyro);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        gyro = new GyroIO() {};
        drive =
            new Drive(
                gyro,
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        // turret = new Turret(gyro);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation));
        simulation = new Simulation(drive::getPose, drive::getChassisSpeedsFieldRelative);
        break;

      default:
        // Replayed robot, disable IO implementations
        gyro = new GyroIO() {};
        drive =
            new Drive(
                gyro, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation));
        simulation = null;
        // turret = new Turret(gyro);
        break;
    }

    trajectorySolver =
        new TrajectorySolver(
            () -> batman.isConnected() ? batman.getPose2d() : drive.getPose(),
            drive::getChassisSpeedsFieldRelative);
    turret = new Turret(drive::getYaw, drive::getRate, trajectorySolver::getYaw);
    hood = new Hood(trajectorySolver::getAngle);
    shooter = new Shooter(trajectorySolver::getShootSpeed);

    NamedCommands.registerCommand("Start Intake", intake.forceDeploy());
    NamedCommands.registerCommand("Stop Intake", intake.forceRetract());
    NamedCommands.registerCommand("Start Shooter", shoot());
    NamedCommands.registerCommand("Stop Shooter", stopShoot());
    NamedCommands.registerCommand("Calibrate Hood", hood.calibrate());
    NamedCommands.registerCommand("Reset Batman", resetBatman());

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
    configureButtonBox();
  }

  private void configureSmartDashboardButtons() {
    if (Util.logif()) {
      SmartDashboard.putData("RunFooter", shooter.runShooter().alongWith(feeder.runFeeder()));
      SmartDashboard.putData(
          "CalibrateTurret", turret.calibrateEncodersFactory().ignoringDisable(true));
      SmartDashboard.putData("SetTurretTargeting", turret.setPositionTargeting());
      SmartDashboard.putData("StopFooter", shooter.stopShooter().alongWith(feeder.stopFeeder()));
      SmartDashboard.putData("RunFeeder", feeder.runFeeder());
      SmartDashboard.putData("StopFeeder", feeder.stopFeeder());
      SmartDashboard.putData("RunShooter", shooter.runShooter());
      SmartDashboard.putData("StopShooter", shooter.stopShooter());
      // SmartDashboard.putData("RunShooterVoltage", shooter.runShooterVoltage());
      SmartDashboard.putData("RunIntake", intake.runIntake());
      SmartDashboard.putData("StopIntake", intake.stopIntake());
      SmartDashboard.putData("RunSpinner", spinner.runSpinner());
      SmartDashboard.putData("StopSpinner", spinner.stopSpinner());
      SmartDashboard.putData("RunHopper", feeder.runFeeder().alongWith(spinner.runSpinner()));
      SmartDashboard.putData("StopHopper", feeder.stopFeeder().alongWith(spinner.stopSpinner()));
      SmartDashboard.putData("RunFooter", feeder.runFeeder().alongWith(shooter.runShooter()));
      SmartDashboard.putData("StopFooter", feeder.stopFeeder().alongWith(shooter.stopShooter()));
      //   SmartDashboard.putData(
      //       "Shooter/SysId/Quasistatic Forward", shooter.sysIdQuasistatic(Direction.kForward));
      //   SmartDashboard.putData(
      //       "Shooter/SysId/Quasistatic Reverse", shooter.sysIdQuasistatic(Direction.kReverse));
      //   SmartDashboard.putData(
      //       "Shooter/SysId/Dynamic Forward", shooter.sysIdDynamic(Direction.kForward));
      //   SmartDashboard.putData(
      //       "Shooter/SysId/Dynamic Reverse", shooter.sysIdDynamic(Direction.kReverse));
      //   SmartDashboard.putData(
      //       "Feeder/SysId/Quasistatic Forward", feeder.sysIdQuasistatic(Direction.kForward));
      //   SmartDashboard.putData(
      //       "Feeder/SysId/Quasistatic Reverse", feeder.sysIdQuasistatic(Direction.kReverse));
      //   SmartDashboard.putData(
      //       "Feeder/SysId/Dynamic Forward", feeder.sysIdDynamic(Direction.kForward));
      //   SmartDashboard.putData(
      //       "Feeder/SysId/Dynamic Reverse", feeder.sysIdDynamic(Direction.kReverse));
      SmartDashboard.putData("Drive/RotateAroundTurretCenter", driveRotateAroundTurretCenter());
      SmartDashboard.putData("Drive/RotateAroundRobotCenter", driveRotateAroundRobotCenter());
      SmartDashboard.putData("Prepclimb", prepClimber());
    }
    SmartDashboard.putData( // DO NOT FLIP IF RED
        "Batman/SetPose", resetBatman());
  }

  private void configureDefaultCommands() {
    spinner.setDefaultCommand(spinner.stopSpinner());
    intake.setDefaultCommand(intake.stopIntake());
    feeder.setDefaultCommand(feeder.stopFeeder());
    shooter.setDefaultCommand(shooter.stopShooter());
    hood.setDefaultCommand(hood.setPositionTargeting());
    turret.setDefaultCommand(turret.setPositionToZero());
    drive.setDefaultCommand(driveRotateAroundTurretCenter());
    climber.setDefaultCommand(climber.stopall());
  }

  public void disabledInit() {
    shooter.resetBPS();
  }

  private void configureDriverController() {
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller
        .x()
        .onTrue(
            intake
                .stopIntake()
                .alongWith(driveRotateAroundTurretCenter())
                .alongWith(shooter.stopShooter())
                .alongWith(spinner.stopSpinner())
                .alongWith(feeder.stopFeeder()));

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
    controller
        .rightTrigger()
        .onTrue(shoot())
        .onFalse( // TODO: Replace with stop shoot when okay
            driveRotateAroundTurretCenter()
                .alongWith(shooter.stopShooter())
                .alongWith(spinner.stopSpinner())
                .alongWith(feeder.stopFeeder())
                .alongWith(hood.setNotShooting()));
    controller.leftTrigger().onTrue(intake.runIntake()).onFalse(intake.stopIntake());
  }

  public void configureButtonBox() {
    buttons.button(1).onTrue(prepClimber());
    buttons.button(2).onTrue(climber.leftFlip());
    // buttons.button(3).onTrue(climber.rightFlip());
    buttons.button(4).onTrue(climber.gotoL1());
    buttons.button(5).onTrue(climber.gotoStow());
    buttons.button(6).onTrue(intake.forceDeploy());
    buttons.button(7).onTrue(intake.forceRetract());
    buttons.button(8).onTrue(driveRotateAroundRobotCenter());
    buttons.button(9).onTrue(driveRotateAroundTurretCenter());
    buttons.button(10).onTrue(resetBatman());
  }

  public Command prepClimber() {
    return climber.calibrate().withTimeout(0.5).andThen(climber.gotoGrip());
  }

  public Command resetBatman() {
    return batman.resetQuestPose(() -> new Pose3d(drive.getPose())).ignoringDisable(true);
  }

  public Command driveRotateAroundRobotCenter() {
    return DriveCommands.joystickDrive(
        drive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX());
  }

  public Command driveRotateAroundTurretCenter() {
    return DriveCommands.rotateAroundTurret(
        drive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX());
  }

  public Command shoot() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(shooter.runShooter(), turret.setPositionTargeting())
            .until(() -> turret.onTarget() && shooter.atShooterSpeed()),
        new ParallelCommandGroup(
            spinner.runSpinner(),
            feeder.runFeeder(),
            shooter.runShooter(),
            turret.setPositionTargeting(),
            hood.setIsShooting()));
  }

  public Command stopShoot() { // temporarily just for autos. Copied from shoot controller command
    return shooter
        .stopShooter()
        .alongWith(spinner.stopSpinner())
        .alongWith(feeder.stopFeeder())
        .alongWith(hood.setNotShooting());
  }

  // /**
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
  //  *
  //  * @return the command to run in autonomous
  //  */
  public Command getAutonomousCommand() {
    return autoChooser.get();
    // return null;
  }
}
