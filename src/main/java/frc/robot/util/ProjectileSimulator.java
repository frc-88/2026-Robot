package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;

public class ProjectileSimulator {
  private boolean flag = true;
  private boolean aboveTrajectory = false;
  private double MAX_SIM_TIME = 10.0;

  public class Pose3dTime extends Pose3d {
    public double time = 0;

    public Pose3dTime(Translation3d translation, Rotation3d rotation, double currentTime) {
      super(translation, rotation);
      time = currentTime;
    }

    public Pose3d toPose3d() {
      return new Pose3d(getTranslation(), getRotation());
    }

    public double getTime() {
      return time;
    }
  }

  public static final double INITIAL_SPEED = 10.0; // m/s
  public static final double INITIAL_PITCH = Math.toRadians(65); // vertical angle
  public static final double INITIAL_YAW = Math.toRadians(45); // horizontal angle

  public static final double DIAMETER = 0.1501; // meters
  public static final double MASS = 0.22; // kg

  public static final double DRAG_COEFF = 0.47;
  public static final double MAGNUS_COEFF = 0.2;
  public static final double AIR_DENSITY = 1.225;
  public static final double GRAVITY = 9.81;

  public static final double TIME_STEP = 0.02; // seconds

  private final double radius = DIAMETER / 2.0;
  private final double area = Math.PI * radius * radius;

  public List<Pose3dTime> simulate(
      Translation2d initial,
      double initialSpeed,
      double yawDeg,
      double pitchDeg,
      Translation2d robotVelocity) {

    double time = 0.0;

    double yawRad = Math.toRadians(yawDeg);

    double vx =
        initialSpeed * Math.cos(Units.degreesToRadians(pitchDeg)) * Math.cos(yawRad)
            + robotVelocity.getX();
    double vy =
        initialSpeed * Math.cos(Units.degreesToRadians(pitchDeg)) * Math.sin(yawRad)
            + robotVelocity.getY();
    double vz = initialSpeed * Math.sin(Units.degreesToRadians(pitchDeg));

    double[] state = {
      initial.getX(),
      initial.getY(),
      Units.inchesToMeters(22.0), // initial height
      vx,
      vy,
      vz
    };

    double targetHeight = Units.feetToMeters(5.5);

    List<Pose3dTime> trajectory = new ArrayList<>();
    trajectory.add(
        new Pose3dTime(new Translation3d(state[0], state[1], state[2]), new Rotation3d(), time));

    double previousZ = state[2];

    while (time < MAX_SIM_TIME && state[2] >= 0) {

      state = rk4Step(state, TIME_STEP);
      time += TIME_STEP;

      trajectory.add(
          new Pose3dTime(new Translation3d(state[0], state[1], state[2]), new Rotation3d(), time));

      if (previousZ >= targetHeight && state[2] < targetHeight) {
        break;
      }

      previousZ = state[2];
    }

    // System.out.println(trajectory.size());
    return trajectory;
  }

  private double[] rk4Step(double[] s, double dt) {
    double[] k1 = derivatives(s);
    double[] k2 = derivatives(add(s, scale(k1, dt / 2)));
    double[] k3 = derivatives(add(s, scale(k2, dt / 2)));
    double[] k4 = derivatives(add(s, scale(k3, dt)));

    double[] out = new double[6];
    for (int i = 0; i < 6; i++) {
      out[i] = s[i] + dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
    return out;
  }

  private double[] derivatives(double[] s) {
    double vx = s[3], vy = s[4], vz = s[5];
    double v = Math.sqrt(vx * vx + vy * vy + vz * vz);

  
    double dragFactor = -0.5 * AIR_DENSITY * DRAG_COEFF * area * v / MASS;
    double fxD = dragFactor * vx;
    double fyD = dragFactor * vy;
    double fzD = dragFactor * vz;
    
    double fxM = 0.0;
    double fyM = 0.0;
    double fzM = 0.5 * AIR_DENSITY * MAGNUS_COEFF * area * v * v / MASS;

    return new double[] {vx, vy, vz, fxD + fxM, fyD + fyM, fzD + fzM - GRAVITY};
  }

  private double[] add(double[] a, double[] b) {
    double[] out = new double[a.length];
    for (int i = 0; i < a.length; i++) {
      out[i] = a[i] + b[i];
    }
    return out;
  }

  private double[] scale(double[] a, double s) {
    double[] out = new double[a.length];
    for (int i = 0; i < a.length; i++) {
      out[i] = a[i] * s;
    }
    return out;
  }
}
