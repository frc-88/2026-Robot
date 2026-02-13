// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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
  public static final int HOPPER_FEEDER_MAIN = 19;

  // Shooter Feeder
  public static final int FEEDER_MAIN = 18;

  // Shooter
  public static final int SHOOTER_MAIN = 12;
  public static final int SHOOTER_FOLLOWER = 5;

  // Drive
  // See generated/TunerConstants.java

  public static final Translation2d ZEROED_YAW_ROBOT_CENTER_TO_TURRET_CENTER =
      new Translation2d(0.3, (-3 / 4) * Math.PI); // GUESS
  // After we zero yaw, this would be the heading corresponding to the turret direction and the
  // distance
  // Angle should be around -3PI/4 Rad

}
