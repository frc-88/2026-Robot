package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrajectorySolver extends SubsystemBase {
  public Translation2d robotToTurret = new Translation2d(0.3, -3.0 / 4.0 * Math.PI); // m
  public Translation2d targetPosition = new Translation2d(2.0, 2.0);
  public static Rotation2d quarterRotation = Rotation2d.fromRadians(Math.PI / 2.0);

  // TO GO IN CONSTANTS ^^

  private DoublePreferenceConstant robotPoseX = new Double DoublePreferenceConstant("TrajSolv/RobotPoseX", 0.0);
  private DoublePreferenceConstant robotPoseY = new Double DoublePreferenceConstant("TrajSolv/RobotPoseY", 0.0);
  private DoublePreferenceConstant robotPoseYaw = new Double DoublePreferenceConstant("TrajSolv/RobotPoseYaw", 0.0);
  private DoublePreferenceConstant robotVelocityX = new Double DoublePreferenceConstant("TrajSolv/RobotVelX", 0.0);
  private DoublePreferenceConstant robotVelocityY = new Double DoublePreferenceConstant("TrajSolv/RobotVelY", 0.0);
  private DoublePreferenceConstant robotVelocityRot = new Double DoublePreferenceConstant("TrajSolv/RobotVelRot", 0.0);

  public Translation2d robotPosition = new Translation2d(0.0, 0.0); // m
  public Rotation2d robotYaw = new Rotation2d(Math.PI * (1.0 / 3.0)); // rad
  public Translation2d robotVelocity = new Translation2d(-0.2,-0.2); // m/s
  public double robotRotationalVelocity = 0.0; // rad/s
  public Translation2d targetVelocity = new Translation2d(0,0); // for hub: 0

  private Translation2d turretToTargetDistance;
  private Translation2d turretToTargetRelativeVelocity;

  private boolean hasPreviousTimeOfFlightGuess = false;
  private double timeOfFlight = 0.0; // seconds
  private Translation2d turretToProjectedTarget; // m; distanceToTarget
  private double turretToProjectedTargetDistance;
  private int numberOfIterations = 5;

  public double hoodAngle;
  public double shootSpeed;

  public TrajectorySolver() {
    
  }

  public void setInputs() {
    robotPosition = new Translation2d(robotPoseX.getValue(), robotPoseY.getValue()); // m
    robotYaw = new Rotation2d.fromDegrees(robotPoseYaw.getValue()); // rad
    robotVelocity = new Translation2d(robotVelocityX.getValue(), robotPoseY.getValue()); // m/s
    robotRotationalVelocity = robotVelocityRot.getValue(); // rad/s
    targetVelocity = new Translation2d(0,0); // for hub: 0
  }

  public void periodic() {
    turretToTargetDistance =
        targetPosition.minus(robotPosition).minus(robotToTurret.rotateBy(robotYaw));
    turretToTargetRelativeVelocity =
        robotVelocity.plus(
            robotToTurret
                .rotateBy(robotYaw.plus(Rotation2d.fromRadians(Math.PI / 2)))
                .times(robotRotationalVelocity))
                .minus(targetVelocity);
    if (turretToTargetRelativeVelocity.getNorm() > (1.0 / 25.0)) {
      newton();
    } else {
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
