package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import java.util.function.Supplier;
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
  public Translation2d targetVelocity = Translation2d.kZero; // for hub: 0

  public DoublePreferenceConstant angle = new DoublePreferenceConstant("Traj/angle", 0.0);
  public DoublePreferenceConstant speed = new DoublePreferenceConstant("Traj/speed", 0.0);

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

  public double getAngle() {
    return hoodAngle;
    // return angle.getValue(); // for testing
  }

  public double getShootSpeed() {
    return shootSpeed;
    // return speed.getValue(); // for testing
  }

  public double getYaw() {
    return turretToProjectedTarget.getAngle().getDegrees();
  }

  @Override
  public void periodic() {
    robotPosition = drivePoseSupplier.get().getTranslation();
    robotVelocity = velocityPoseSupplier.get().getTranslation();
    robotYaw = drivePoseSupplier.get().getRotation();
    robotRotationalVelocity = velocityPoseSupplier.get().getRotation().getRadians();

    turretPosition = robotPosition.plus(robotToTurret.rotateBy(robotYaw));

    targetPosition = findTargetPosition(); // no velocity set

    Logger.recordOutput("Trajectory/TurretPosition", new Pose2d(turretPosition, Rotation2d.kZero));

    turretToCurrentTarget = targetPosition.minus(turretPosition);
    turretToTargetRelativeVelocity =
        robotVelocity
            .plus(
                robotToTurret
                    .rotateBy(robotYaw.plus(Rotation2d.fromRadians(Math.PI / 2)))
                    .times(robotRotationalVelocity))
            .minus(targetVelocity);
    Logger.recordOutput(
        "Trajectory/TurretToTargetRelativeVelocity", turretToTargetRelativeVelocity);

    if (turretToTargetRelativeVelocity.getNorm() > (1.0 / 25.0)) {
      newton();
    } else {
      turretToProjectedTarget = turretToCurrentTarget;
      Logger.recordOutput("Trajectory/Distance", turretToProjectedTarget.getNorm());
      Logger.recordOutput(
          "Trajectory/ProjectedHub",
          new Pose2d(
              turretToProjectedTarget.plus(robotPosition).plus(robotToTurret.rotateBy(robotYaw)),
              Rotation2d.kZero));
      hasPreviousTimeOfFlightGuess = false;
      hoodAngle = lookupAngle(turretToCurrentTarget.getNorm());
      shootSpeed = lookupSpeed(turretToCurrentTarget.getNorm());
    }
    Logger.recordOutput("Trajectory/Yaw", turretToProjectedTarget.getAngle().getDegrees());
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
      timeOfFlight = lookupTime(turretToCurrentTarget.getNorm());
      System.out.println("Newton Solution Diverged. TOF: " + timeOfFlight);
    }
    turretToProjectedTarget =
        turretToCurrentTarget.minus(turretToTargetRelativeVelocity.times(timeOfFlight));
    turretToProjectedTargetDistance = turretToProjectedTarget.getNorm();
    hoodAngle = lookupAngle(turretToProjectedTargetDistance);
    shootSpeed = lookupSpeed(turretToProjectedTargetDistance);
    Logger.recordOutput(
        "Trajectory/ProjectedHub",
        new Pose2d(
            turretToProjectedTarget.plus(robotPosition.plus(robotToTurret.rotateBy(robotYaw))),
            Rotation2d.kZero));
  }

  public Translation2d findTargetPosition() {
    if (robotPosition.getX() > Units.inchesToMeters(181.56)) {
      if (robotPosition.getY() > Units.inchesToMeters(158.32)) {
        return Constants.LEFT_SHUTTLE_TARGET_POSITION;
      } else {
        return Constants.RIGHT_SHUTTLE_TARGET_POSITION;
      }
    } else {
      return Constants.HUB_POSITION;
    }
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
    return 84.15886 - 16.18452 * Math.log(distance);
    // - 1.11 * (Math.pow(distance, 3.0));
  }

  public double lookupSpeed(double distance) {
    if (Constants.currentMode == Mode.SIM) {
      return 5.3731 + 0.356504 * (distance) + 0.0279446 * (Math.pow(distance, 2.0));

    } else { // real
      return 27.71457 + 2.96448 * (distance);
    }
    // + 0.0279446 * (Math.pow(distance, 2.0));
    // - 0.0514 * (Math.pow(distance, 3.0));
  }
}
