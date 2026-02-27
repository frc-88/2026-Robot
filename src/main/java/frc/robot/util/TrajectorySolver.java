package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class TrajectorySolver extends SubsystemBase {
  private Translation2d robotToTurret = Constants.robotToTurret;
  private Translation2d targetPosition = Constants.HUB_POSITION;

  Supplier<Pose2d> drivePoseSupplier;
  Supplier<Pose2d> velocityPoseSupplier;

  public Translation2d robotPosition = Translation2d.kZero; // m
  public Rotation2d robotYaw = Rotation2d.kZero; // rad
  public Translation2d robotVelocity = Translation2d.kZero; // m/s
  public double robotRotationalVelocity = 0.0; // rad/s
  public Translation2d targetVelocity = Translation2d.kZero;

  private Translation2d turretToCurrentTarget;
  private Translation2d turretToTargetRelativeVelocity;
  private Translation2d turretPosition;

  private boolean hasPreviousTimeOfFlightGuess = false;
  private double timeOfFlight = 0.0; // seconds
  private Translation2d turretToProjectedTarget = Translation2d.kZero; // m; distanceToTarget
  private double turretToProjectedTargetDistance = 0.0;
  private int numberOfIterations = 5;

  public double hoodAngle;
  public double shootSpeed;

  public TrajectorySolver(Supplier<Pose2d> drivePose, Supplier<Pose2d> velocityPose) {
    drivePoseSupplier = drivePose;
    velocityPoseSupplier = velocityPose;
  }

  @AutoLogOutput(key = "Trajectory/HoodAngle")
  public double getAngle() {
    return hoodAngle;
  }

  @AutoLogOutput(key = "Trajectory/ShooterSpeed")
  public double getShootSpeed() {
    return shootSpeed;
  }

  @AutoLogOutput(key = "Trajectory/Yaw")
  public double getYaw() {
    return turretToProjectedTarget.getAngle().getDegrees() - 180.0;
  }

  @Override
  public void periodic() {
    robotPosition = drivePoseSupplier.get().getTranslation();
    robotVelocity = velocityPoseSupplier.get().getTranslation();
    if (Constants.currentMode == Mode.REAL) {
      robotVelocity =
          robotVelocity.rotateBy(Rotation2d.fromDegrees(-90.0)); // why do we have to do this??
    }
    robotYaw = drivePoseSupplier.get().getRotation();
    robotRotationalVelocity = velocityPoseSupplier.get().getRotation().getRadians();

    turretPosition = robotPosition.plus(robotToTurret.rotateBy(robotYaw));

    targetPosition = findTargetPosition(); // no velocity set

    turretToCurrentTarget = targetPosition.minus(turretPosition);
    turretToTargetRelativeVelocity =
        robotVelocity
            .plus(
                robotToTurret
                    .rotateBy(robotYaw.plus(Rotation2d.fromRadians(Math.PI / 2)))
                    .times(robotRotationalVelocity))
            .minus(targetVelocity);

    Logger.recordOutput("Trajectory/RobotPosition", drivePoseSupplier.get());
    Logger.recordOutput("Trajectory/TurretPosition", new Pose2d(turretPosition, Rotation2d.kZero));
    Logger.recordOutput(
        "Trajectory/TurretToTargetRelativeVelocity",
        new Pose2d(turretToTargetRelativeVelocity, Rotation2d.kZero));

    if (turretToTargetRelativeVelocity.getNorm() > (1.0 / 25.0)) {
      newton();
    } else {
      turretToProjectedTarget = turretToCurrentTarget;

      hasPreviousTimeOfFlightGuess = false;
      hoodAngle = lookupAngle(turretToCurrentTarget.getNorm());
      shootSpeed = lookupSpeed(turretToCurrentTarget.getNorm());
    }
    Logger.recordOutput("Trajectory/Distance", turretToProjectedTarget.getNorm());
    // Logger.recordOutput("Trajectory/Yaw", turretToProjectedTarget.getAngle().getDegrees());
    Logger.recordOutput(
        "Trajectory/ProjectedHub",
        new Pose2d(turretToProjectedTarget.plus(turretPosition), Rotation2d.kZero));
  }

  public void newton() {
    if (!hasPreviousTimeOfFlightGuess) {
      timeOfFlight = lookupTime(turretToCurrentTarget.getNorm());
    } // otherwise use last value
    int i;
    for (i = 0; i < numberOfIterations; i++) {
      turretToProjectedTarget =
          turretToCurrentTarget.minus(turretToTargetRelativeVelocity.times(timeOfFlight));
      turretToProjectedTargetDistance = turretToProjectedTarget.getNorm();
      timeOfFlight =
          timeOfFlight
              - (timeOfFlight - lookupTime(turretToProjectedTargetDistance))
                  / (1.0
                      - (lookupTimePrime(turretToProjectedTargetDistance))
                          * (turretToProjectedTarget.dot(
                                  turretToTargetRelativeVelocity.unaryMinus())
                              / turretToProjectedTargetDistance));
    }
    if (timeOfFlight > 5) {
      System.out.println("Newton Solution Diverged. TOF: " + timeOfFlight);
      timeOfFlight = lookupTime(turretToCurrentTarget.getNorm());
    }
    turretToProjectedTarget =
        turretToCurrentTarget.minus(turretToTargetRelativeVelocity.times(timeOfFlight));
    turretToProjectedTargetDistance = turretToProjectedTarget.getNorm();
    hoodAngle = lookupAngle(turretToProjectedTargetDistance);
    shootSpeed = lookupSpeed(turretToProjectedTargetDistance);
  }

  public Translation2d findTargetPosition() {
    Translation2d target;
    Translation2d turret = Util.flipIfRed(turretPosition);

    if (turret.getX() > Units.inchesToMeters(181.56)) {
      if (turret.getY() > Units.inchesToMeters(158.32)) {
        Logger.recordOutput("Trajectory/TargetSelection", "LEFT_SHUTTLE_TARGET_POSITION");
        target = Constants.LEFT_SHUTTLE_TARGET_POSITION;
      } else {
        Logger.recordOutput("Trajectory/TargetSelection", "RIGHT_SHUTTLE_TARGET_POSITION");
        target = Constants.RIGHT_SHUTTLE_TARGET_POSITION;
      }
    } else {
      Logger.recordOutput("Trajectory/TargetSelection", "HUB_POSITION");
      target = Constants.HUB_POSITION;
    }

    return Util.flipIfRed(target);
  }

  public double lookupTime(double distance) {
    return 0.921749 - 0.0138832 * distance + 0.009373 * (Math.pow(distance, 2.0));
    // - 0.0249606 * (Math.pow(distance, 3.0));
  }

  public double lookupTimePrime(double distance) {
    return -0.0138832 + 0.0188 * distance;
    // - 0.0748818 * (Math.pow(distance, 2.0));
  }

  public double lookupAngle(double distance) {
    if (Constants.currentMode == Mode.SIM) {
      return 91.33289 - 11.95018 * distance + 0.880906 * (Math.pow(distance, 2.0));
    } else { // real
      return 6.45 + 4.56 * distance;
      // - 1.11 * (Math.pow(distance, 3.0));
    }
  }

  public double lookupSpeed(double distance) {
    if (Constants.currentMode == Mode.SIM) {
      return 5.3731 + 0.356504 * (distance) + 0.0279446 * (Math.pow(distance, 2.0));

    } else { // real
      return 25.4 + 3.94 * (distance);
    }
    // + 0.0279446 * (Math.pow(distance, 2.0));
    // - 0.0514 * (Math.pow(distance, 3.0));
  }
}
