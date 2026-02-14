package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TrajectorySolver extends SubsystemBase {
  public Translation2d robotToTurret = new Translation2d(0.3, -3.0 / 4.0 * Math.PI); // m
  public Translation2d targetPosition = new Translation2d(2.0, 2.0);
  // TO GO IN CONSTANTS ^^

  public Translation2d robotPosition = new Translation2d(0.0, 0.0); // m
  public Rotation2d robotYaw = new Rotation2d(Math.PI * (1.0 / 3.0)); // rad
  public Translation2d robotVelocity = new Translation2d(-0.2, -0.2); // m/s
  public double robotRotationalVelocity = 0.0; // rad/s
  public Translation2d targetVelocity = new Translation2d(0, 0); // for hub: 0

  public DoublePreferenceConstant angle = new DoublePreferenceConstant("Traj/angle", 0.0);
  public DoublePreferenceConstant speed = new DoublePreferenceConstant("Traj/speed", 0.0);

  private Translation2d turretToTargetDistance;
  private Translation2d turretToTargetRelativeVelocity;

  private boolean hasPreviousTimeOfFlightGuess = false;
  private double timeOfFlight = 0.0; // seconds
  private Translation2d turretToProjectedTarget = Translation2d.kZero; // m; distanceToTarget
  private double turretToProjectedTargetDistance;
  private int numberOfIterations = 5;

  public double hoodAngle;
  public double shootSpeed;
  Supplier<Pose2d> drivePose1;
  Supplier<Translation2d> vel1;

  public TrajectorySolver(Supplier<Pose2d> drivePose, Supplier<Translation2d> vel) {
    drivePose1 = drivePose;
    vel1 = vel;
    robotYaw = Rotation2d.kZero;
    robotToTurret = Translation2d.kZero;
    targetPosition = Constants.HUB_POSE;
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

  public void periodic() {
    robotPosition = drivePose1.get().getTranslation();
    robotVelocity = vel1.get();

    turretToTargetDistance =
        targetPosition.minus(robotPosition).minus(robotToTurret.rotateBy(robotYaw));
    turretToTargetRelativeVelocity =
        robotVelocity
            .plus(
                robotToTurret
                    .rotateBy(robotYaw.plus(Rotation2d.fromRadians(Math.PI / 2)))
                    .times(robotRotationalVelocity))
            .minus(targetVelocity);
    if (turretToTargetRelativeVelocity.getNorm() > (1.0 / 25.0)) {
      newton();
    } else {
      turretToProjectedTarget = turretToTargetDistance;
      Logger.recordOutput("Field/distance", turretToProjectedTarget.getNorm());
      hasPreviousTimeOfFlightGuess = false;
      hoodAngle = lookupAngle(turretToTargetDistance.getNorm());
      shootSpeed = lookupSpeed(turretToTargetDistance.getNorm());
    }
  }

  public void newton() {
    if (!hasPreviousTimeOfFlightGuess) {
      timeOfFlight = lookupTime(turretToTargetDistance.getNorm());
    } // otherwise use last value
    int i;
    for (i = 0; i < numberOfIterations; i++) {
      turretToProjectedTarget =
          turretToTargetDistance.minus(turretToTargetRelativeVelocity.times(timeOfFlight));
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
      timeOfFlight = lookupTime(turretToTargetDistance.getNorm());
      System.out.println("Newton Solution Diverged");
    }
    turretToProjectedTarget =
        turretToTargetDistance.minus(turretToTargetRelativeVelocity.times(timeOfFlight));
    turretToProjectedTargetDistance = turretToProjectedTarget.getNorm();
    hoodAngle = lookupAngle(turretToProjectedTargetDistance);
    shootSpeed = lookupSpeed(turretToProjectedTargetDistance);
    System.out.println("BALL");
    Logger.recordOutput(
        "Field/ProjectedHub",
        new Pose2d(turretToProjectedTarget.plus(robotPosition), Rotation2d.kZero));
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
    return 91.33289 - 11.95018 * distance + 0.880906 * (Math.pow(distance, 2.0));
    // - 1.11 * (Math.pow(distance, 3.0));
  }

  public double lookupSpeed(double distance) {
    return 5.3731 + 0.356504 * (distance) + 0.0279446 * (Math.pow(distance, 2.0));
    // - 0.0514 * (Math.pow(distance, 3.0));
  }
}
