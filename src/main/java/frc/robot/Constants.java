// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

<<<<<<< HEAD
import edu.wpi.first.math.geometry.Rotation2d;
=======
>>>>>>> origin/turret
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
<<<<<<< HEAD
  public static final int SPINNER_MAIN = 6;

  // Hopper Feeder
  public static final int HOPPER_FEEDER_MAIN = 19; // Not used
=======
  public static final int SPINNER_MAIN = 17;
>>>>>>> origin/turret

  // Shooter Feeder
  public static final int FEEDER_MAIN = 19;

  // Shooter
  public static final int SHOOTER_MAIN = 12;
  public static final int SHOOTER_FOLLOWER = 5;
  public static final int HOOD = 0;
  public static final double HOOD_DEGREES_PER_ROTATION = 0;

  // Turret
  public static final int TURRET_MOTOR_ID = 6;
  public static final int TURRET_CANCODER_ID = 8;
<<<<<<< HEAD
  public static final int TURRET_CANCODER_GEAR_RATIO = 0;
  public static final int TURRET_GEAR_RATIO = 0;
  public static final double TURRET_COUNTS_PER_REV = 0;
  public static final int CANCODER_ = 0;
=======
  public static final int TURRET_CANCODER_GEAR_RATIO = 1;
  public static final int TURRET_GEAR_RATIO = 1;
  public static final double TURRET_COUNTS_PER_REV = 1;
>>>>>>> origin/turret

  // Drive
  // See generated/TunerConstants.java

<<<<<<< HEAD
  public static Translation2d HUB_POSE =
      new Translation2d(Units.inchesToMeters(188.178500), Units.inchesToMeters(158.955000));
  public static Translation2d robotToTurret =
      new Translation2d(0.3, Rotation2d.kCW_90deg); // 6.745, -5.75
=======
  // HOOD Milk
  public static int HOOD = 16;
  public static double MINION_ROT_TO_ANGLE = (1.0 / (287.0 / 54.0)) * 360.0;

  public static Translation2d robotToTurret =
      // new Translation2d(
      //     Units.inchesToMeters(Math.hypot(6.745, 5.75)),
      //     Rotation2d.fromDegrees(-30.0)); // 6.745, -5.75
      new Translation2d(Units.inchesToMeters(-6.745), Units.inchesToMeters(-5.750));
>>>>>>> origin/turret
}
