// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

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
  public static final int INTAKE_MAIN = 2;

  // Spinner
  public static final int SPINNER_MAIN = 17;

  // Shooter Feeder
  public static final int FEEDER_MAIN = 19;

  // Shooter
  public static final int SHOOTER_MAIN = 12;
  public static final int SHOOTER_FOLLOWER = 5;
  public static final double HOOD_DEGREES_PER_ROTATION = 0;

  // Turret
  public static final int TURRET_MOTOR_ID = 6;
  public static final int TURRET_CANCODER_ID1 = 15;
  public static final int TURRET_CANCODER_ID2 = 12;
  public static final int TURRET_CANCODER_GEAR_RATIO = 1;
  public static final int TURRET_GEAR_RATIO = 1;
  public static final double TURRET_COUNTS_PER_REV = 1;

  //TODO This
  public static final double FIELD_WIDTH = 8.05;
  public static final double FIELD_LENGTH = 17.55;

  // Drive
  // See generated/TunerConstants.java

  // HOOD Milk
  public static int HOOD = 16;
  public static double MINION_ROT_TO_ANGLE = (1.0 / (287.0 / 54.0)) * 360.0;

  public static Translation2d robotToTurret =
      // new Translation2d(
      //     Units.inchesToMeters(Math.hypot(6.745, 5.75)),
      //     Rotation2d.fromDegrees(-30.0)); // 6.745, -5.75
      new Translation2d(Units.inchesToMeters(-6.745), Units.inchesToMeters(-5.750));
  public static Translation2d HUB =
      new Translation2d(Units.inchesToMeters(181.56), Units.inchesToMeters(158.84));
  public static Translation2d RIGHT_SHUTTLE_TARGET = new Translation2d(0.5, 0.5);
  public static Translation2d LEFT_SHUTTLE_TARGET = new Translation2d(0.5, 7.5);
}
