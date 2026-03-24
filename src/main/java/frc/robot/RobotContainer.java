// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.camera2Name;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
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
import frc.robot.util.AutoStartPositions;
import frc.robot.util.TrajectorySolver;
import frc.robot.util.Util;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
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
  private final Intake intake;
  private final HotTub hotTub;
  private final TrajectorySolver trajectorySolver;
  private final Batman batman = new Batman();
  private final Hood hood;
  private final Vision vision;
  private final Simulation simulation;
  // private final Climber climber = new Climber();

  private final AutoStartPositions autoStartPositions = new AutoStartPositions();

  private final CommandXboxController controller = new CommandXboxController(0);
  private CommandGenericHID buttons = new CommandGenericHID(1);

  public final LoggedDashboardChooser<Command> autoChooser;
  private boolean shooting = false;
  private boolean shouldUseQuest = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    GyroIO gyro;

    // TODO Disable diagnostic server if in COMP mode?
    // if (!Util.logif()) {
    //   Unmanaged.setPhoenixDiagnosticsStartTime(-1);
    // }
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
                new ModuleIOTalonFX(TunerConstants.BackRight),
                this::getPoseBatman);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation),
                new VisionIOLimelight(camera2Name, drive::getRotation));
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
                new ModuleIOSim(TunerConstants.BackRight),
                this::getPoseBatman);
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
                gyro,
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                this::getPoseBatman);
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
            () -> ((batman.shouldUse() && shouldUseQuest) ? batman.getPose2d() : drive.getPose()),
            drive::getChassisSpeedsFieldRelative);
    turret =
        new Turret(
            () -> drive.getChassisSpeedsFieldRelative().getRotation().getDegrees(),
            trajectorySolver::getTurretTarget,
            trajectorySolver::getDistanceToProjectedTarget,
            trajectorySolver::getIsTargetingHub);
    feeder = new Feeder(this::onTarget);
    hotTub = new HotTub(this::onTarget);
    hood = new Hood(trajectorySolver::getAngle);
    shooter = new Shooter(trajectorySolver::getShootSpeed);
    intake = new Intake(drive::getSpeed);

    NamedCommands.registerCommand("Intake Out", intake.deployIntake());
    NamedCommands.registerCommand("Intake In", intake.retractIntake());
    NamedCommands.registerCommand("Intake The Thing", intake.doTheThing());

    NamedCommands.registerCommand("Shoot", shoot());
    NamedCommands.registerCommand("Don't Shoot", stopShoot());

    NamedCommands.registerCommand("Calibrate Hood", hood.calibrate());
    NamedCommands.registerCommand("Reset Batman", resetBatman());
    NamedCommands.registerCommand("Start Targeting", turret.startTargeting());

    // NamedCommands.registerCommand("Auto Prep", new WaitCommand(0.1));

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
      SmartDashboard.putData("AntiJam", antiJam());
      SmartDashboard.putData("Drive/RotateAroundTurretCenter", driveRotateAroundTurretCenter());
      SmartDashboard.putData("Drive/RotateAroundRobotCenter", driveRotateAroundRobotCenter());

      SmartDashboard.putData("Drive/TrenchAlign", driveTrench());

      // SmartDashboard.putData("Prepclimb", prepClimber());
    }
    // SmartDashboard.putData("Batman/SetPose", resetBatman());
  }

  private void configureDefaultCommands() {
    hotTub.setDefaultCommand(hotTub.stopSpinner());
    intake.setDefaultCommand(intake.deployJustIntake(/*() -> climber.getLiftPosition() > 2.0*/ ));
    feeder.setDefaultCommand(feeder.stopFeeder());
    shooter.setDefaultCommand(shooter.stopShooter());
    hood.setDefaultCommand(hood.setPositionTargeting());
    turret.setDefaultCommand(turret.aim());
    drive.setDefaultCommand(driveRebuilt());
    // climber.setDefaultCommand(climber.stopall());
  }

  public void disabledInit() {
    shooter.resetBPS();
  }

  public boolean onTarget() {
    return turret
        .onTarget(); // && (time until next shift start - TOF - 1.5 sec for processing < 0.0)
  }

  private void configureDriverController() {
    // controller
    //     .a()
    //     .toggleOnTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> 0.0,
    //             () -> -controller.getLeftX(),
    //             () -> Rotation2d.fromDegrees(-90.0)));
    // controller.a().toggleOnTrue(driveRebuilt());

    // Switch to X pattern when X button is pressed
    controller
        .x()
        .onTrue(
            driveAtAngle(() -> Rotation2d.fromDegrees(90.0))
                .until(() -> MathUtil.applyDeadband(-controller.getRightX(), 0.1) != 0.0));

    controller
        .a()
        .onTrue(
            driveAtAngle(() -> Rotation2d.fromDegrees(180.0))
                .until(() -> MathUtil.applyDeadband(-controller.getRightX(), 0.1) != 0.0));

    controller
        .b()
        .onTrue(
            driveAtAngle(() -> Rotation2d.fromDegrees(-90.0))
                .until(() -> MathUtil.applyDeadband(-controller.getRightX(), 0.1) != 0.0));

    controller
        .y()
        .onTrue(
            driveAtAngle(() -> Rotation2d.fromDegrees(0.0))
                .until(() -> MathUtil.applyDeadband(-controller.getRightX(), 0.1) != 0.0));

    // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
    //                 drive)
    //            .ignoringDisable(true));
    controller.rightTrigger().onTrue(shoot()).onFalse(stopShoot());
    controller.leftBumper().whileTrue(driveTrench());

    controller.leftTrigger().whileTrue(intake.deployIntake());
    controller.rightBumper().whileTrue(intake.retractIntake());
  }

  public void configureButtonBox() {
    // buttons.button(1).whileTrue(prepClimber());
    // buttons.button(2).onTrue(L1AndFlip());
    // buttons.button(4).onTrue(L1());
    // buttons.button(5).onTrue(climber.gotoStow());
    buttons.button(6).onTrue(intake.deployIntake());
    buttons.button(7).onTrue(intake.retractIntake());
    buttons.button(10).onTrue(resetBatman());
    buttons.button(3).onTrue(turret.syncCommand());
    buttons.button(8).whileTrue(intake.doTheThing());
    buttons.button(9).whileTrue(antiJam());
    // buttons.button(11).whileTrue(getOffTower());
    buttons
        .button(12)
        .toggleOnTrue(
            new InstantCommand(
                    () -> {
                      shouldUseQuest = !shouldUseQuest;
                      if (shouldUseQuest) {
                        batman.resetPose(new Pose3d(drive.getPose()));
                      }
                    })
                .ignoringDisable(true));
  }

  public void periodic() {

    // if (climber.getLiftPosition() > 20) {
    //   intake.intakeIn();
    // }

    String autoName = autoChooser.get().getName();

    Logger.recordOutput("ShouldUseQuest", shouldUseQuest);
    Logger.recordOutput("AutoName", autoName);

    Pose2d targetStartingPose = autoStartPositions.getStartingPose(autoName);
    Pose2d currentRobotPose = Util.flipIfRed(drive.getPose());

    boolean isPoseSafe = false;
    double poseDistance =
        targetStartingPose.getTranslation().getDistance(currentRobotPose.getTranslation());
    if (poseDistance
        < 0.5) { // Compare if the distance between the current and target pose is within 0.5 meters
      isPoseSafe = true;
    }

    SmartDashboard.putBoolean("Starting Position/Starting Position is Safe", isPoseSafe);
    SmartDashboard.putNumber("Starting Position/Distance to Starting Target", poseDistance);
  }

  public Pose2d getPoseBatman() {
    return batman.shouldUse() ? batman.getPose2d() : drive.getPose();
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
            shooting ? MathUtil.clamp(-controller.getLeftY(), -0.65, 0.65) : -controller.getLeftY(),
        () ->
            shooting ? MathUtil.clamp(-controller.getLeftX(), -0.65, 0.65) : -controller.getLeftX(),
        () ->
            shooting
                ? MathUtil.clamp(-controller.getRightX(), -0.75, 0.75)
                : -controller.getRightX(),
        this::turretRotSupplier);
  }

  public Command driveTrench() {
    return DriveCommands.trenchDrive(
        drive,
        () ->
            shooting ? MathUtil.clamp(-controller.getLeftY(), -0.5, 0.5) : -controller.getLeftY());
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

  public Command driveAtAngle(Supplier<Rotation2d> angle) {
    return DriveCommands.joystickDriveAtAngle(
        drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), angle);
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
        shooter.runShooter(),
        hotTub.runSpinner(),
        feeder.runFeeder(),
        hood.setIsShooting(),
        intake.setShooting());
  }

  public Command setShooting(boolean shoot) {
    return new InstantCommand(() -> shooting = shoot);
  }

  public void stopShooting() {
    shooting = false;
  }

  public Command stopShoot() {
    return new ParallelCommandGroup(
        setShooting(false),
        shooter.stopShooter(),
        hotTub.stopSpinner(),
        feeder.stopFeeder(),
        hood.setNotShooting(),
        intake.setNotShooting());
  }

  public Command antiJam() {
    return shooter
        .stopShooter()
        .alongWith(setShooting(false))
        .alongWith(hotTub.antiJamSpinner())
        .alongWith(feeder.antiJamFeeder().alongWith(intake.antiJamIntake()));
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
