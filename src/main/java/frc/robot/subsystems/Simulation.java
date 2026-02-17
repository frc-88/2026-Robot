package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.FieldObject3D;
import frc.robot.util.ProjectileSimulator;
import frc.robot.util.TrajectorySolver;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Function;
import java.util.function.Supplier;

public class Simulation extends SubsystemBase {
  Pose2d HUB_POSE = new Pose2d(4.0, 4.1, new Rotation2d());
  Random rand = new Random();
  private List<FieldObject3D> allFuel = new ArrayList<>();
  private double timeSinceLastShot = 0.0;
  Function<Double, Double> pitch;

  ProjectileSimulator sim = new ProjectileSimulator();
  TrajectorySolver trajectorySolver;

  Supplier<Pose2d> drivePose1;
  Supplier<Pose2d> velocity1;

  public Simulation(Supplier<Pose2d> drivePose, Supplier<Pose2d> velocity) {
    drivePose1 = drivePose;
    velocity1 = velocity;
    trajectorySolver = new TrajectorySolver(drivePose, velocity);

    for (int i = 0; i < 12; i++) {
      allFuel.add(
          new FieldObject3D(
              String.format("Field/Fuel%d", i), String.format("Field/Fuel%dTime", i)));
    }
  }

  public void periodic() {
    for (int i = 0; i < allFuel.size(); i++) {
      FieldObject3D fuel = allFuel.get(i);
      if (fuel.getCount() == 0) {
        if (fuel.hasTrajectory() && timeSinceLastShot > 2) {
          fuel.setPose();
          timeSinceLastShot = 0;
        } else if (fuel.hasTrajectory() && timeSinceLastShot <= 2) {

        } else {
          double speed = trajectorySolver.getShootSpeed();
          double angle = trajectorySolver.getAngle();
          double yaw = trajectorySolver.getYaw();
          fuel.setTrajectory(
              sim.simulate(
                  drivePose1.get().getTranslation(),
                  speed,
                  yaw,
                  angle,
                  velocity1
                      .get()
                      .getTranslation()
                      .plus(
                          Constants.robotToTurret
                              .rotateBy(drivePose1.get().getRotation().plus(Rotation2d.kCCW_90deg))
                              .times(velocity1.get().getRotation().getRadians()))));
        }
      } else {
        if (!fuel.setPose()) {
          double speed = trajectorySolver.getShootSpeed();
          double angle = trajectorySolver.getAngle();
          double yaw = trajectorySolver.getYaw();
          fuel.setTrajectory(
              sim.simulate(
                  drivePose1
                      .get()
                      .getTranslation()
                      .plus(Constants.robotToTurret.rotateBy(drivePose1.get().getRotation())),
                  speed,
                  yaw,
                  angle,
                  velocity1
                      .get()
                      .getTranslation()
                      .plus(
                          (Constants.robotToTurret.rotateBy(
                                  drivePose1.get().getRotation().plus(Rotation2d.kCCW_90deg)))
                              .times(velocity1.get().getRotation().getRadians()))));
        }
      }
    }
    timeSinceLastShot++;
  }
}
