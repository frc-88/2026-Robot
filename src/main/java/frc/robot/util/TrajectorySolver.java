package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class TrajectorySolver extends SubsystemBase {
  private Translation2d robotToTurret = Constants.ROBOT_TO_TURRET;
  private Translation2d targetPosition = Constants.HUB_POSITION;

  Supplier<Pose2d> drivePoseSupplier;
  Supplier<Pose2d> velocityPoseSupplier;

  private Translation2d robotPosition = Translation2d.kZero; // m
  private Rotation2d robotYaw = Rotation2d.kZero; // rad
  public Translation2d robotVelocity = Translation2d.kZero; // m/s
  private Translation2d lastRobotVelocity = Translation2d.kZero;
  // public Timer accelerationTimer = new Timer();
  // public double lastTime = 0.0;
  private double robotRotationalVelocity = 0.0; // rad/s
  private Translation2d robotAcceleration = Translation2d.kZero; // m/s/s
  private Translation2d targetVelocity = Translation2d.kZero;

  private Translation2d turretToCurrentTarget;
  private Translation2d turretVelocity;
  private Translation2d turretPosition;
  private Translation2d target = Constants.HUB_POSITION;

  private boolean hasPreviousTimeOfFlightGuess = false;
  private double timeOfFlight = 0.0; // seconds
  private Translation2d turretToProjectedTarget = Translation2d.kZero; // m; distanceToTarget
  private double turretToProjectedTargetDistance = 0.0;
  private int numberOfIterations = 5;

  public DoublePreferenceConstant lagCompensation =
      new DoublePreferenceConstant("Trajectory/LagCompensation", 0.01);

  private double hoodAngle;
  private double shootSpeed;

  private boolean isTargetingHub = true;
  private double lastTargetRadians = 0.0;
  private BooleanSupplier isPreAimingSupplier;

  public TrajectorySolver(
      Supplier<Pose2d> drivePose, Supplier<Pose2d> velocityPose, BooleanSupplier isPreAiming) {
    drivePoseSupplier = drivePose;
    velocityPoseSupplier = velocityPose;
    isPreAimingSupplier = isPreAiming;
  }

  @AutoLogOutput(key = "Trajectory/HoodAngle")
  public double getAngle() {
    return hoodAngle;
  }

  @AutoLogOutput(key = "Trajectory/ShooterSpeed")
  public double getShootSpeed() {
    return shootSpeed;
  }

  public double getDistanceToProjectedTarget() {
    return turretToProjectedTargetDistance;
  }

  @AutoLogOutput(key = "Trajectory/TurretTarget")
  public double getTurretTarget() {
    double targetRadians =
        MathUtil.angleModulus(
            turretToProjectedTarget
                .getAngle()
                .minus(Rotation2d.k180deg)
                .minus(Rotation2d.fromDegrees(robotYaw.getDegrees()))
                .getRadians());
    double delta = MathUtil.angleModulus(targetRadians - lastTargetRadians);
    double targetDegrees = Units.radiansToDegrees(lastTargetRadians + delta);
    if (targetDegrees >= 250.0) {
      targetDegrees -= 360.0;
    } else if (targetDegrees <= -250.0) {
      targetDegrees += 360.0;
    }
    lastTargetRadians = Units.degreesToRadians(targetDegrees);
    return targetDegrees;
  }

  @AutoLogOutput
  public boolean getIsTargetingHub() {
    return isTargetingHub;
  }

  public double getSimTarget() {
    return turretToProjectedTarget.getAngle().getDegrees();
  }

  public double getTimeOfFlight() {
    return timeOfFlight;
  }

  @Override
  public void periodic() {
    robotPosition = drivePoseSupplier.get().getTranslation();
    robotVelocity = velocityPoseSupplier.get().getTranslation();
    robotYaw = drivePoseSupplier.get().getRotation();
    robotRotationalVelocity = velocityPoseSupplier.get().getRotation().getRadians();

    boolean cancelX = false;
    boolean cancelY = false;

    if ((robotPosition.getX() > Constants.FIELD_LENGTH - Constants.FIELD_MARGIN
            && robotVelocity.getX() > 0.0)
        || (robotPosition.getX() < Constants.FIELD_MARGIN && robotVelocity.getX() < 0.0)) {
      robotVelocity = new Translation2d(0.0, robotVelocity.getY());
      cancelX = true;
    }

    if ((robotPosition.getY() > Constants.FIELD_WIDTH - Constants.FIELD_MARGIN
            && robotVelocity.getY() > 0.0)
        || (robotPosition.getY() < Constants.FIELD_MARGIN && robotVelocity.getY() < 0.0)) {
      robotVelocity = new Translation2d(robotVelocity.getX(), 0.0);
      cancelY = true;
    }
    Logger.recordOutput("Trajectory/IsCancelingX", cancelX);
    Logger.recordOutput("Trajectory/IsCancelingY", cancelY);

    robotAcceleration = getAcceleration();
    lastRobotVelocity = robotVelocity;

    robotPosition =
        robotPosition
            .plus(robotVelocity.times(lagCompensation.getValue()))
            .plus(
                robotAcceleration.times(
                    lagCompensation.getValue() * lagCompensation.getValue() * (1.0 / 2.0)));
    robotVelocity = robotVelocity.plus(robotAcceleration.times(lagCompensation.getValue()));

    turretPosition = robotPosition.plus(robotToTurret.rotateBy(robotYaw));

    targetPosition = findTargetPosition(); // no velocity set

    turretToCurrentTarget = targetPosition.minus(turretPosition);
    turretVelocity =
        robotVelocity
            .plus(
                robotToTurret
                    .rotateBy(robotYaw.plus(Rotation2d.fromRadians(Math.PI / 2)))
                    .times(robotRotationalVelocity))
            .minus(targetVelocity);

    // Logger.recordOutput("Trajectory/TurretPosition", new Pose2d(turretPosition,
    // Rotation2d.fromDegrees(getTurretTarget())));
    Logger.recordOutput("Trajectory/TurretVelocity", new Pose2d(turretVelocity, Rotation2d.kZero));

    if (turretVelocity.getNorm() > (1.0 / 50.0)) {
      newton();
    } else {
      turretToProjectedTarget = turretToCurrentTarget;
      hasPreviousTimeOfFlightGuess = false;
      hoodAngle = lookupAngle(turretToCurrentTarget.getNorm());
      shootSpeed = lookupSpeed(turretToCurrentTarget.getNorm());
    }
    Logger.recordOutput("Trajectory/Distance", turretToProjectedTarget.getNorm());
    Logger.recordOutput(
        "Trajectory/ProjectedHub",
        new Pose2d(turretToProjectedTarget.plus(turretPosition), Rotation2d.kZero));

    if (turretToProjectedTargetDistance > 5.50) {
      Logger.recordOutput("Trajectory/IsExtrapolating", true);
    } else {
      Logger.recordOutput("Trajectory/IsExtrapolating", false);
    }
  }

  private void newton() {
    if (!hasPreviousTimeOfFlightGuess) {
      timeOfFlight = lookupTime(turretToCurrentTarget.getNorm());
    } // otherwise use last value
    int i;
    for (i = 0; i < numberOfIterations; i++) {
      turretToProjectedTarget = turretToCurrentTarget.minus(turretVelocity.times(timeOfFlight));
      turretToProjectedTargetDistance = turretToProjectedTarget.getNorm();
      timeOfFlight =
          timeOfFlight
              - (timeOfFlight - lookupTime(turretToProjectedTargetDistance))
                  / (1.0
                      - (lookupTimePrime(turretToProjectedTargetDistance))
                          * (turretToProjectedTarget.dot(turretVelocity.unaryMinus())
                              / turretToProjectedTargetDistance));
    }
    hasPreviousTimeOfFlightGuess = true;
    if (timeOfFlight > 3.5) {
      System.out.println("Newton Solution Diverged. TOF: " + timeOfFlight);
      timeOfFlight = lookupTime(turretToCurrentTarget.getNorm());
      hasPreviousTimeOfFlightGuess = false;
    }
    turretToProjectedTarget = turretToCurrentTarget.minus(turretVelocity.times(timeOfFlight));
    turretToProjectedTargetDistance = turretToProjectedTarget.getNorm();
    hoodAngle = lookupAngle(turretToProjectedTargetDistance);
    shootSpeed = lookupSpeed(turretToProjectedTargetDistance);
  }

  private Translation2d findTargetPosition() {
    Translation2d turret = Util.flipIfRed(turretPosition);

    if (turret.getX() > Units.inchesToMeters(181.56)) {
      if (turret.getY() > Constants.FIELD_WIDTH * (2.0 / 3.0)) {
        target = Constants.MIDDLE_LEFT_SHUTTLE_TARGET_POSITION;
        isTargetingHub = false;
      } else if (turret.getY() > Constants.FIELD_WIDTH * (1.0 / 2.0)) {
        target = Constants.LEFT_SHUTTLE_TARGET_POSITION;
        isTargetingHub = false;
      } else if (turret.getY() > Constants.FIELD_WIDTH * (1.0 / 3.0)) {
        target = Constants.RIGHT_SHUTTLE_TARGET_POSITION;
        isTargetingHub = false;
      } else {
        target = Constants.MIDDLE_RIGHT_SHUTTLE_TARGET_POSITION;
        isTargetingHub = false;
      }
    } else {
      target = Constants.HUB_POSITION;
      isTargetingHub = true;
    }

    if (isPreAimingSupplier.getAsBoolean()) {
      target = Constants.HUB_POSITION;
      isTargetingHub = true;
    }

    return Util.flipIfRed(target);
  }

  @AutoLogOutput
  private Translation2d getAcceleration() {
    // double currentTime = accelerationTimer.get();
    // System.out.println(currentTime);
    // double dt = currentTime - lastTime;
    // System.out.println(dt);
    return robotVelocity.minus(lastRobotVelocity).div(0.02);
  }

  private double lookupTime(double distance) {
    if (!isTargetingHub) {
      return 0.78 + 0.0951 * distance;
    } else { // real hub
      return 0.78 + 0.0951 * distance;
    }
  }

  private double lookupTimePrime(double distance) {
    if (!isTargetingHub) {
      return 0.0951;
    } else { // real hub
      return 0.0951;
    }
  }

  private double lookupAngle(double distance) {
    if (Constants.currentMode == Mode.SIM) {
      return 91.33289 - 11.95018 * distance + 0.880906 * (Math.pow(distance, 2.0));
    } else {
      if (!isTargetingHub) {
        return 2.33
            + 8.31 * distance
            - 1.12 * (Math.pow(distance, 2.0))
            + 0.0647 * (Math.pow(distance, 3.0));
      } else { // real hub
        return 2.33
            + 8.31 * distance
            - 1.12 * (Math.pow(distance, 2.0))
            + 0.0647 * (Math.pow(distance, 3.0));
      }
    }
  }

  private double lookupSpeed(double distance) {
    if (Constants.currentMode == Mode.SIM) {
      return 5.3731 + 0.356504 * (distance) + 0.0279446 * (Math.pow(distance, 2.0));
    } else {
      if (!isTargetingHub) {
        return 25.7 + 3.81 * distance;
      } else { // real hub
        return 25.7 + 3.81 * distance;
      }
    }
  }
}
