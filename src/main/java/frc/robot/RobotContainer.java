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
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.HotTub;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Simulation;
import frc.robot.subsystems.Turret;
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
  private final Feeder feeder;
  private final Shooter shooter;
  private final Intake intake = new Intake();
  private final HotTub hotTub;
  private final TrajectorySolver trajectorySolver;
  private final Batman batman = new Batman();
  private final Hood hood;
  private final Vision vision;
  private final Simulation simulation;
  private final Climber climber = new Climber();

  private final CommandXboxController controller = new CommandXboxController(0);
  private CommandGenericHID buttons = new CommandGenericHID(1);

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
        break;
    }

    trajectorySolver =
        new TrajectorySolver(
            () -> (batman.isConnected() ? batman.getPose2d() : drive.getPose()),
            drive::getChassisSpeedsFieldRelative);
    turret =
        new Turret(
            () -> (batman.isConnected() ? batman.getPose2d() : drive.getPose()),
            drive::getRate,
            trajectorySolver::getTurretTarget);
    feeder = new Feeder(turret::onTarget);
    hotTub = new HotTub(turret::onTarget);
    hood = new Hood(trajectorySolver::getAngle);
    shooter = new Shooter(trajectorySolver::getShootSpeed);

    NamedCommands.registerCommand("Start Intake", intake.deployIntake());
    NamedCommands.registerCommand("Stop Intake", intake.retractIntake());
    NamedCommands.registerCommand("Start Shooter", shoot());
    NamedCommands.registerCommand("Stop Shooter", stopShoot());
    NamedCommands.registerCommand("Calibrate Hood", hood.calibrate());
    NamedCommands.registerCommand("Reset Batman", resetBatman());

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureSmartDashboardButtons();
    configureDefaultCommands();
    configureDriverController();
    configureButtonBox();
  }

  private void configureSmartDashboardButtons() {
    SmartDashboard.putData(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    SmartDashboard.putData(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));

    if (Util.logif()) {
      SmartDashboard.putData("RunFooter", shooter.runShooter().alongWith(feeder.runFeeder()));
      SmartDashboard.putData("StopFooter", shooter.stopShooter().alongWith(feeder.stopFeeder()));
      SmartDashboard.putData("RunShooter", shooter.runShooter());
      SmartDashboard.putData("StopShooter", shooter.stopShooter());
      SmartDashboard.putData("RunIntake", intake.runIntake());
      SmartDashboard.putData("StopIntake", intake.stopIntake());
      SmartDashboard.putData("RunHopper", feeder.runFeeder().alongWith(hotTub.runSpinner()));
      SmartDashboard.putData("StopHopper", feeder.stopFeeder().alongWith(hotTub.stopSpinner()));
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
    SmartDashboard.putData("Batman/SetPose", resetBatman());
  }

  private void configureDefaultCommands() {
    hotTub.setDefaultCommand(hotTub.stopSpinner());
    intake.setDefaultCommand(intake.stopIntake()); // TODO calibrate first?
    feeder.setDefaultCommand(feeder.stopFeeder());
    shooter.setDefaultCommand(shooter.stopShooter());
    hood.setDefaultCommand(hood.setPositionTargeting());
    turret.setDefaultCommand(turret.aim());
    drive.setDefaultCommand(driveRotateAroundTurretCenter());
    climber.setDefaultCommand(climber.stopall()); // TODO calibration
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
                .alongWith(hotTub.stopSpinner())
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
    controller.rightTrigger().onTrue(shoot()).onFalse(stopShoot());

    controller.leftTrigger().onTrue(intake.runIntake()).onFalse(intake.stopIntake());
  }

  public void configureButtonBox() {
    buttons.button(1).onTrue(prepClimber());
    buttons.button(2).onTrue(climber.leftFlip());
    // buttons.button(3).onTrue(climber.rightFlip());
    buttons.button(4).onTrue(climber.gotoL1());
    buttons.button(5).onTrue(climber.gotoStow());
    buttons.button(6).onTrue(intake.deployIntake());
    buttons.button(7).onTrue(intake.retractIntake());
    buttons.button(8).onTrue(driveRotateAroundRobotCenter());
    buttons.button(9).onTrue(driveRotateAroundTurretCenter());
    buttons.button(10).onTrue(resetBatman());
  }

  public Command prepClimber() {
    return climber.calibrate().withTimeout(0.5).andThen(climber.gotoGrip());
  }

  public Command resetBatman() { // DO NOT FLIP IF RED
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
    return new ParallelCommandGroup(
        hotTub.runSpinner(), feeder.runFeeder(), shooter.runShooter(), hood.setIsShooting());
  }

  public Command stopShoot() {
    return shooter
        .stopShooter()
        .alongWith(hotTub.stopSpinner())
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
  }
}
