package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TrajectorySolver {
  public Translation2d robotToTurret = new Translation2d(0.3, -3 / 4 * Math.PI); // m
  public Translation2d goalPosition = new Translation2d(2, 2);
  public static Rotation2d quarterRotation = Rotation2d.fromRadians(Math.PI / 2);

  // TO GO IN CONSTANTS ^^

  public Translation2d robotPosition = new Translation2d(1, 1); // m
  public Rotation2d robotYaw = new Rotation2d(Math.PI * (1 / 3)); // rad
  public Translation2d robotVelocity = new Translation2d(); // m/s
  public double robotRotationalVelocity = 0.1; // rad/s

  public boolean hasPreviousTimeOfFlightGuess = false;
  public double timeOfFlight = 0; // seconds
  public Translation2d turretToTarget; // m; distanceToTarget
  public int numberOfIterations = 5;

  public double hoodAngle;
  public double shootSpeed;

  public void periodic() {
    if (robotVelocity.getNorm() > (1 / 25)) {
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
    } //otherwise use last value
    int i;
    for (i = 0; i < numberOfIterations; i++) {
      Translation2d turretToProjectedTarget =
          turretDistanceToTarget.minus(turretRelativeVelocityToTarget.times(timeOfFlight));
      double projectedTargetDistance = turretToProjectedTarget.getNorm();
      timeOfFlight =
          timeOfFlight
              - (timeOfFlight - lookupTime(projectedTargetDistance))
                  / (1
                      - (lookupTimePrime(projectedTargetDistance))
                          * (turretToProjectedTarget.dot(turretRelativeVelocityToTarget.unaryMinus())
                              / projectedTargetDistance));
    }
    turretToTarget = turretDistanceToTarget.minus(turretRelativeVelocityToTarget.times(timeOfFlight));
    double turretToProjectedTargetDistance = turretToTarget.getNorm();
    hoodAngle = lookupAngle(turretToProjectedTargetDistance);
    shootSpeed = lookupSpeed(turretToProjectedTargetDistance);
  }

  public double lookupTime(double distance) {
    return Math.sqrt(distance); // temp
  }

  public double lookupTimePrime(double distance) {
    return 1 / (2 * Math.sqrt(distance)); // temp
  }

  public double lookupAngle(double distance) {
    return distance * 25; // very temp
  }

  public double lookupSpeed(double distance) {
    return distance * 30; // temp
  }
}
