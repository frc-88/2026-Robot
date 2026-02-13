// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // Intake
  public static final int INTAKE_MAIN = 2;

  // Spinner
  public static final int SPINNER_MAIN = 6;

  // Hopper Feeder
  public static final int HOPPER_FEEDER_MAIN = 19; // Not used

  // Shooter Feeder
  public static final int FEEDER_MAIN = 18;

  // Shooter
  public static final int SHOOTER_MAIN = 12;
  public static final int SHOOTER_FOLLOWER = 5;
  public static final int HOOD = 0;
  public static final double HOOD_DEGREES_PER_ROTATION = 0;

  // Drive
  // See generated/TunerConstants.java

  public static Translation2d HUB_POSE =
      new Translation2d(Units.inchesToMeters(188.178500), Units.inchesToMeters(158.955000));
  public static Translation2d ROBOT_CENTER_TO_TURRET_CENTER =
      new Translation2d(0, 0); // absolute when yaw = 0 TODO
  public static Rotation2d ANGLE_OF_ROBOT_ZERO_HEADING_TO_TURRET = Rotation2d.kZero; // TODO

  public static Translation2d getTurretPose(Pose2d drivePose) {
    return drivePose
        .getTranslation()
        .plus(ROBOT_CENTER_TO_TURRET_CENTER.rotateBy(drivePose.getRotation()));
  }
}
