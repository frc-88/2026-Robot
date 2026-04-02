// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rectangle2d;
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

  public static enum ModeCode {
    COMP,
    DEVELOPMENT
  }

  public static ModeCode MODE = ModeCode.DEVELOPMENT;

  // Intake
  public static final int INTAKE_PIVOT = 14;
  public static final int INTAKE_ROLLER = 15;
  public static final int INTAKE_PIVOT_ROLLER = 16;
  public static final double INTAKE_PIVOT_MOTOR_ROTATIONS_TO_ROTATIONS =
      (28.0 / 12.0) * (3.0) * (4.0) * (48.0 / 16.0);

  // Spinner
  public static final int SPINNER_MAIN = 21;

  // Feeder
  public static final int FEEDER_MAIN = 4;

  // Shooter
  public static final int SHOOTER_MAIN = 7;
  public static final int SHOOTER_FOLLOWER = 6;
  public static final double SHOOTER_GEAR_RATIO = 24.0 / 18.0;

  // Climber
  public static final int CLIMBER_LIFT = 16;
  public static final int CLIMBER_PIVOT = 19;
  public static final int CLIMBER_CANRANGE = 7;
  public static final double CLIMBER_PIVOT_ROTATIONS_TO_ROBOT_ROTATIONS = (144 * 37) / 9;
  public static final double CLIMBER_LIFT_ROTATIONS_TO_ROBOT_INCHES = (1.0 / 11.1125);

  // Turret
  public static final int TURRET_MOTOR_ID = 3;
  public static final int TURRET_RETRACTOMATIC_ID = 17;
  public static final int TURRET_CANCODER_ID2 = 3;

  // Drive
  public static final int BASE_PIGEON = 0;
  // See generated/TunerConstants.java

  // HOOD Milk
  public static final int HOOD = 5;
  public static final double MINION_ROT_TO_ANGLE = (1.0 / (287.0 / 54.0)) * 360.0;

  // Robot and Field constants
  public static final double FIELD_WIDTH = 8.07; // meters
  public static final double FIELD_LENGTH = 16.54; // meters

  public static final Translation2d ROBOT_TO_TURRET =
      // new Translation2d(Units.inchesToMeters(-10.431),
      //     Units.inchesToMeters(8.992));
      new Translation2d(Units.inchesToMeters(-6.745), Units.inchesToMeters(-5.750));

        public static final double FIELD_MARGIN = Units.inchesToMeters(22.0);
  public static Translation2d HUB_POSITION =
      new Translation2d(Units.inchesToMeters(181.56), Units.inchesToMeters(158.84));
  public static Translation2d RIGHT_SHUTTLE_TARGET_POSITION = new Translation2d(3.0, 1.5);
  public static Translation2d LEFT_SHUTTLE_TARGET_POSITION = new Translation2d(3.0, 6.5);

  public static double HUB_RADIUS_TOLERANCE = Units.inchesToMeters((41.0 - 6.0) / 2.0);

  public static double RIGHT_TRENCH_Y = Units.inchesToMeters(49.82 / 2.0);
  public static double LEFT_TRENCH_Y = Constants.FIELD_WIDTH - Units.inchesToMeters(49.82 / 2.0);

  public static double FUEL_SCORING_TIME = 0.25;
}
