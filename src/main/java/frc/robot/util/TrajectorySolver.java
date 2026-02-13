package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TrajectorySolver {
  public Translation2d robotToTurret = new Translation2d(0.3, -3.0 / 4.0 * Math.PI); // m
  public Translation2d goalPosition = new Translation2d(2.0, 2.0);
  public static Rotation2d quarterRotation = Rotation2d.fromRadians(Math.PI / 2.0);

  // TO GO IN CONSTANTS ^^

  public Translation2d robotPosition = new Translation2d(1.0, 1.0); // m
  public Rotation2d robotYaw = new Rotation2d(Math.PI * (1.0 / 3.0)); // rad
  public Translation2d robotVelocity = new Translation2d(); // m/s
  public double robotRotationalVelocity = 0.1; // rad/s

  private boolean hasPreviousTimeOfFlightGuess = false;
  private double timeOfFlight = 0.0; // seconds
  private Translation2d turretToProjectedTarget; // m; distanceToTarget
  private double turretToProjectedTargetDistance;
  private int numberOfIterations = 5;

  public double hoodAngle;
  public double shootSpeed;

  public void periodic() {
    if (robotVelocity.getNorm() > (1.0 / 25.0)) {
      newton();
    } else {
      hasPreviousTimeOfFlightGuess = false;
      double staticDistance =
          goalPosition.minus(robotPosition).minus(robotToTurret.rotateBy(robotYaw)).getNorm();
      lookupAngle(staticDistance);
      lookupSpeed(staticDistance);
    }
  }

  public void newton() {
    Translation2d turretDistanceToTarget =
        goalPosition.minus(robotPosition).minus(robotToTurret.rotateBy(robotYaw));
    Translation2d turretRelativeVelocityToTarget =
        robotVelocity.plus(
            robotToTurret
                .rotateBy(robotYaw.plus(Rotation2d.fromRadians(Math.PI / 2)))
                .times(robotRotationalVelocity));
    if (!hasPreviousTimeOfFlightGuess) {
      timeOfFlight = lookupTime(turretDistanceToTarget.getNorm());
    } // otherwise use last value
    int i;
    for (i = 0; i < numberOfIterations; i++) {
      turretToProjectedTarget =
          turretDistanceToTarget.minus(turretRelativeVelocityToTarget.times(timeOfFlight));
      turretToProjectedTargetDistance = turretToProjectedTarget.getNorm();
      timeOfFlight =
          timeOfFlight
              - (timeOfFlight - lookupTime(turretToProjectedTargetDistance))
                  / (1.0
                      - (lookupTimePrime(turretToProjectedTargetDistance))
                          * (turretToProjectedTarget.dot(
                                  turretRelativeVelocityToTarget.unaryMinus())
                              / turretToProjectedTargetDistance));
    }
    turretToProjectedTarget =
        turretDistanceToTarget.minus(turretRelativeVelocityToTarget.times(timeOfFlight));
    turretToProjectedTargetDistance = turretToProjectedTarget.getNorm();
    hoodAngle = lookupAngle(turretToProjectedTargetDistance);
    shootSpeed = lookupSpeed(turretToProjectedTargetDistance);
  }

  public double lookupTime(double distance) {
    return Math.sqrt(distance); // temp
  }

  public double lookupTimePrime(double distance) {
    return 1.0 / (2.0 * Math.sqrt(distance)); // temp
  }

  public double lookupAngle(double distance) {
    return distance * 25.0; // very temp
  }

  public double lookupSpeed(double distance) {
    return distance * 30.0; // temp
  }
}
