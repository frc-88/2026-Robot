// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

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

  // Climber
  public static final int CLIMBER_LIFT = 21;
  public static final int CLIMBER_PIVOT = 22;
  public static final int CLIMBER_CANRANGE = 8;
  public static final double CLIMBER_PIVOT_ROTATIONS_TO_ROBOT_ROTATIONS = (144 * 37) / 9;
  public static final double CLIMBER_LIFT_ROTATIONS_TO_ROBOT_INCHES = (1.0 / 11.1125);

  // Drive
  // See generated/TunerConstants.java

}
