package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ProjectileSimulator.Pose3dTime;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Trajectory extends SubsystemBase {

  private int NUM_ITER = 5;
  private Function<Double, Double> speedLU =
      (Double dist) -> {
        // return dist * (8.0 / 6.0) + 1.0;
        return 9.7 - 2.6 * (dist) + 0.719 * (Math.pow(dist, 2.0)) - 0.0514 * (Math.pow(dist, 3.0));
      };
  private Function<Double, Double> angleLU =
      (Double dist) -> {
        // return Units.radiansToDegrees(Math.atan2(Units.feetToMeters(6.0), dist)) + 30.0;
        return 128.0 - 48.9 * dist + 12.7 * (Math.pow(dist, 2.0)) - 1.11 * (Math.pow(dist, 3.0));
      };
  //   private Function<Double, Double> ToF = (Double distance) -> {
  //     return 0.0;
  //   };

  private double speed = 0.0;
  private double angle = 0.0;
  private double yaw = 0.0;

  Supplier<Pose2d> drivePose;
  Supplier<Pose2d> driveVelocity;
  ProjectileSimulator fuelSim;

  public Trajectory(Supplier<Pose2d> drive, Supplier<Pose2d> velocity, ProjectileSimulator sim) {
    drivePose = drive;
    driveVelocity = velocity;
    fuelSim = sim;
  }

  //   public double getSpeed() {
  //     return speed;
  //   }

  //   public double getAngle() {
  //     return angle;
  //   }

  //   public double getYaw() {
  //     return yaw;
  //   }

  //   public void setTrajectory(Pose2d robotPose, Translation2d robotVelocity, double ToF) {
  //     Translation2d target =
  //         Constants.HUB_POSE
  //             .getTranslation()
  //             .minus(robotPose.getTranslation())
  //             .minus(robotVelocity.times(ToF));
  //     double dist = target.getNorm();
  //     speed = speedLU.apply(dist);
  //     angle = angleLU.apply(dist);
  //     yaw = target.getAngle().getDegrees();
  //   }

  //   public void runNewtonRaphson() {
  //     double finalToF = 0.0;
  //     double dist = 0.0;
  //     Pose2d drive = drivePose.get();
  //     Translation2d velocity = driveVelocity.get().plus(driveVelocity.);
  //     Translation2d currdist =
  // Constants.HUB_POSE.getTranslation().minus(drive.getTranslation().plus(Constants.ROBOT_CENTER_TO_TURRET_CENTER.rotateBy(drive.getRotation())));
  //     double tof = ToF.apply(currdist.getNorm());

  //     for (int i = 0; i < NUM_ITER; i++) {}

  //     setTrajectory(drive, velocity, finalToF);
  //   }

  //   public void periodic() {
  //     runNewtonRaphson();
  //   }

  /** Yaw, speed, angle */
  public double[] getSpeedYawAndAngle() {
    double[] res = new double[3];
    Translation2d drivePoseCurrent = drivePose.get().getTranslation();
    double dist = Constants.HUB_POSE.minus(drivePoseCurrent).getNorm();
    Pose2d driveVelocityCurrent = driveVelocity.get();

    speed = speedLU.apply(dist);
    angle = angleLU.apply(dist);
    Logger.recordOutput("Field/Dist", dist);
    Logger.recordOutput("Field/vel", speed);
    Logger.recordOutput("Field/angle", angle);
    // System.out.println(dist);
    // System.out.println(String.format("dist:%f", dist));
    // System.out.println(String.format("speed before:%f", speed));
    // System.out.println(String.format("angle before:%f", angle));
    yaw = Constants.HUB_POSE.minus(drivePoseCurrent).getAngle().getDegrees();
    // System.out.println(String.format("yaw: %f", yaw));
    List<Pose3dTime> trajectory =
        fuelSim.simulate(
            drivePoseCurrent, speed, yaw, angle, driveVelocityCurrent.getTranslation());
    double tof = trajectory.get(trajectory.size() - 1).getTime();
    // System.out.println(String.format("Time: %f", tof));
    Translation2d projected =
        Constants.HUB_POSE
            .minus(drivePoseCurrent)
            .minus(driveVelocity.get().getTranslation().times(tof));
    double distProjected = projected.getNorm();
    speed = speedLU.apply(distProjected);
    angle = angleLU.apply(distProjected);
    // System.out.println(String.format("dist:%f", dist));
    // System.out.println(String.format("dist proj: %f", distProjected));
    // System.out.println(String.format("speed:%f", speed));
    // System.out.println(String.format("angle:%f", angle));

    res[0] = speed;
    res[1] = projected.getAngle().getDegrees();
    res[2] = angle;
    return res;
  }
}
