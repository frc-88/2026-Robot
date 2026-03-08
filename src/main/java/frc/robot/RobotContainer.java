// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private boolean shooting = false;

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
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));
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
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));
        simulation = null;
        break;
    }

    trajectorySolver =
        new TrajectorySolver(
            () -> (batman.isConnected() ? batman.getPose2d() : drive.getPose()),
            drive::getChassisSpeedsFieldRelative);
    turret = new Turret(drive::getRate, trajectorySolver::getTurretTarget);
    feeder = new Feeder(turret::onTarget);
    hotTub = new HotTub(turret::onTarget);
    hood = new Hood(trajectorySolver::getAngle);
    shooter = new Shooter(trajectorySolver::getShootSpeed);

    NamedCommands.registerCommand("Deploy Intake", intake.deployIntake()); // intake.forceDeploy());
    NamedCommands.registerCommand(
        "Retract Intake", intake.retractIntake()); // intake.forceRetract());

    NamedCommands.registerCommand("Start Shooter", shoot()); // shoot());
    NamedCommands.registerCommand("Stop Shooter", stopShoot());

    NamedCommands.registerCommand(
        "Climb Grab Right",
        climber.goToGrip()); // TODO: Test climber commands and change them if needed
    NamedCommands.registerCommand("Climb Chin Strap Grip", climber.goToChinStrap());
    NamedCommands.registerCommand("Climb Grip", climber.goToGrip()); // climber.gotoGrip());
    NamedCommands.registerCommand("Climb L1", new WaitCommand(0.1)); // climber.gotoL1());

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
    intake.setDefaultCommand(intake.deployJustIntake()); // TODO calibrate first. THIS
    feeder.setDefaultCommand(feeder.stopFeeder());
    shooter.setDefaultCommand(shooter.stopShooter());
    hood.setDefaultCommand(hood.setPositionTargeting());
    turret.setDefaultCommand(turret.aim());
    drive.setDefaultCommand(driveRebuiltTwo());
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
    controller.leftBumper().toggleOnTrue(intake.deployJustIntake());

    controller.leftTrigger().onTrue(intake.deployIntake()).onFalse(intake.stopIntake());
    controller.rightBumper().toggleOnTrue(intake.doTheThing());
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
    return climber.calibrate().withTimeout(0.5).andThen(climber.goToGrip());
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

  public Command driveRebuilt() {
    return DriveCommands.rebuiltDrive(
        drive,
        () ->
            shooting ? MathUtil.clamp(-controller.getLeftY(), -0.75, 0.75) : -controller.getLeftY(),
        () ->
            shooting ? MathUtil.clamp(-controller.getLeftX(), -0.75, 0.75) : -controller.getLeftX(),
        () ->
            shooting
                ? MathUtil.clamp(-controller.getRightX(), -0.75, 0.75)
                : -controller.getRightX(),
        this::turretRotSupplier);
  }

  public Command driveRebuiltTwo() {
    return DriveCommands.rebuiltDriveTwo(
        drive,
        () ->
            shooting ? MathUtil.clamp(-controller.getLeftY(), -0.75, 0.75) : -controller.getLeftY(),
        () ->
            shooting ? MathUtil.clamp(-controller.getLeftX(), -0.75, 0.75) : -controller.getLeftX(),
        this::angleSupplier,
        this::turretRotSupplier);
  }

  private double angleSupplier() {
    Translation2d rotationControl =
        DriveCommands.getLinearVelocityFromJoysticks(
            -controller.getRightY(), -controller.getRightX());
    return rotationControl.getNorm() < 0.1
        ? drive.getRotation().getRadians()
        : rotationControl.getAngle().getRadians();
  }

  private boolean turretRotSupplier() {
    return shooting
        && Util.flipIfRed(drive.getPose()).getTranslation().getX() < Constants.HUB_POSITION.getX();
  }

  public Command shoot() {
    return new ParallelCommandGroup(
        setShooting(true),
        hotTub.runSpinner(),
        feeder.runFeeder(),
        shooter.runShooter(),
        hood.setIsShooting());
  }

  private Command setShooting(boolean shoot) {
    return new InstantCommand(() -> shooting = shoot);
  }

  public Command stopShoot() {
    return shooter
        .stopShooter()
        .alongWith(setShooting(false))
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
