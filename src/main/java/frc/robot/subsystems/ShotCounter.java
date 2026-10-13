package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ShotDetector;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Counts balls shot per match and sorts them into scoring shots and shuttle shots.
 *
 * <p>This is a watcher only. It owns no motors, no command ever requires it, and it changes nothing
 * about how the robot behaves. It extends SubsystemBase only so that periodic() runs once per loop.
 * Everything it needs comes in through suppliers from RobotContainer, so the other subsystems only
 * had to expose two getters.
 *
 * <p>Each detected ball goes into exactly one bucket, decided by what the trajectory solver was
 * aiming at when the ball left:
 *
 * <ul>
 *   <li><b>Hub</b> (scoring): TrajectorySolver.getIsTargetingHub() is true.
 *   <li><b>Shuttle, midfield</b>: aiming at a shuttle target from the neutral zone.
 *   <li><b>Shuttle, full field</b>: aiming at a shuttle target from the opponent's zone
 *       (TrajectorySolver.getIsFullField()). Split out because the shooter is commanded to 100 rps
 *       there, which it can't reach, so these shots behave differently.
 * </ul>
 *
 * <p>Hub shots fired while our hub is inactive are also counted separately (they score nothing).
 *
 * <p>Balls moved by pushing or by reversing the intake never pass through the shooter, so they are
 * not counted. Shuttle counts are therefore "balls shuttled by the shooter", not all balls moved.
 *
 * <p>Counts reset when autonomous starts, or when teleop starts without an autonomous just before
 * it (a practice enable), and from the "ShotCounter/Reset" dashboard button. A since-boot total
 * that match resets don't touch is kept for the component-usage tracker (shooter wheel wear).
 */
public class ShotCounter extends SubsystemBase {
  public static final int HUB = 0;
  public static final int SHUTTLE_MID = 1;
  public static final int SHUTTLE_FULL = 2;
  private static final String[] CATEGORY_NAMES = {"Hub", "ShuttleMid", "ShuttleFull"};
  private static final String[] KEY_SUFFIXES = {
    "/Auto",
    "/Teleop",
    "/Total",
    "/FeedSeconds",
    "/TriggerSeconds",
    "/BPSFeed",
    "/BPSTrigger",
    "/BPSFlow"
  };
  // Log keys built once, so periodic() doesn't create new strings every loop.
  private static final String[][] KEYS = new String[3][KEY_SUFFIXES.length];

  static {
    for (int c = 0; c < 3; c++) {
      for (int k = 0; k < KEY_SUFFIXES.length; k++) {
        KEYS[c][k] = "ShotCounter/" + CATEGORY_NAMES[c] + KEY_SUFFIXES[k];
      }
    }
  }

  /** Two shots closer than this are part of the same burst, for the flow-rate number (s). */
  private static final double BURST_GAP_SEC = 0.6;
  /** A teleop enable within this long of autonomous ending is the same match (s). */
  private static final double SAME_MATCH_GAP_SEC = 10.0;

  // Tunable from the dashboard (saved on the roboRIO like the other preference constants).
  // Lockout is the main calibration knob; see shot-counter doc.
  private final DoublePreferenceConstant p_dipThreshold =
      new DoublePreferenceConstant("ShotCounter/DipThreshold", 1.5);
  private final DoublePreferenceConstant p_lockoutSec =
      new DoublePreferenceConstant("ShotCounter/LockoutSec", 0.10);

  private final ShotDetector.Config config = new ShotDetector.Config();
  private final ShotDetector detector = new ShotDetector(config);

  // Inputs
  private final DoubleSupplier velocity;
  private final DoubleSupplier setpoint;
  private final BooleanSupplier shooterOn;
  private final BooleanSupplier feederGateOpen;
  private final BooleanSupplier targetingHub;
  private final BooleanSupplier fullField;
  private final BooleanSupplier hubActive;

  // Per-match tallies, indexed [phase][category], phase 0 = auto, 1 = teleop.
  private final int[][] shots = new int[2][3];
  private final double[][] feedSec = new double[2][3]; // feeder gate open, by aim mode
  private final double[][] triggerSec = new double[2][3]; // shoot command running, by aim mode
  private final int[] burstIntervals = new int[3];
  private final double[] burstSec = new double[3];
  private int hubShotsWhileInactive = 0;
  private int lastCategory = -1;
  private double lastShotTimeForBurst = Double.NEGATIVE_INFINITY;
  private int shotsSinceBoot = 0;

  private double lastLoopTime = Double.NaN;
  private double lastAutoEndTime = Double.NEGATIVE_INFINITY;
  private boolean wasAuto = false;

  public ShotCounter(
      DoubleSupplier velocity,
      DoubleSupplier setpoint,
      BooleanSupplier shooterOn,
      BooleanSupplier feederGateOpen,
      BooleanSupplier targetingHub,
      BooleanSupplier fullField,
      BooleanSupplier hubActive) {
    this.velocity = velocity;
    this.setpoint = setpoint;
    this.shooterOn = shooterOn;
    this.feederGateOpen = feederGateOpen;
    this.targetingHub = targetingHub;
    this.fullField = fullField;
    this.hubActive = hubActive;

    SmartDashboard.putData("ShotCounter/Reset", resetCommand());
  }

  /** Call from Robot.autonomousInit() and Robot.teleopInit(). */
  public void onModeStart(boolean autonomous) {
    double now = Timer.getFPGATimestamp();
    if (autonomous || now - lastAutoEndTime > SAME_MATCH_GAP_SEC) {
      resetMatch();
    }
  }

  public void resetMatch() {
    for (int p = 0; p < 2; p++) {
      for (int c = 0; c < 3; c++) {
        shots[p][c] = 0;
        feedSec[p][c] = 0.0;
        triggerSec[p][c] = 0.0;
      }
    }
    for (int c = 0; c < 3; c++) {
      burstIntervals[c] = 0;
      burstSec[c] = 0.0;
    }
    hubShotsWhileInactive = 0;
    lastCategory = -1;
    lastShotTimeForBurst = Double.NEGATIVE_INFINITY;
    detector.reset();
  }

  public Command resetCommand() {
    return new InstantCommand(this::resetMatch).ignoringDisable(true);
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    double dt = Double.isNaN(lastLoopTime) ? 0.0 : Math.min(now - lastLoopTime, 0.2);
    lastLoopTime = now;

    boolean enabled = DriverStation.isEnabled();
    boolean auto = DriverStation.isAutonomous();
    if (wasAuto && !(enabled && auto)) {
      lastAutoEndTime = now;
    }
    wasAuto = enabled && auto;

    config.dipThreshold = p_dipThreshold.getValue();
    config.lockoutSec = p_lockoutSec.getValue();

    boolean on = enabled && shooterOn.getAsBoolean();
    boolean gate = on && feederGateOpen.getAsBoolean();
    int phase = auto ? 0 : 1;
    int category = currentCategory();

    if (on) {
      triggerSec[phase][category] += dt;
    }
    if (gate) {
      feedSec[phase][category] += dt;
    }

    boolean shot = detector.update(now, velocity.getAsDouble(), setpoint.getAsDouble(), on, gate);

    if (shot) {
      shots[phase][category]++;
      shotsSinceBoot++;
      if (category == HUB && !auto && !hubActive.getAsBoolean()) {
        hubShotsWhileInactive++;
      }
      double gap = now - lastShotTimeForBurst;
      if (category == lastCategory && gap < BURST_GAP_SEC) {
        burstIntervals[category]++;
        burstSec[category] += gap;
      }
      lastShotTimeForBurst = now;
      lastCategory = category;
    }

    log(shot, category);
  }

  private int currentCategory() {
    if (targetingHub.getAsBoolean()) {
      return HUB;
    }
    return fullField.getAsBoolean() ? SHUTTLE_FULL : SHUTTLE_MID;
  }

  private void log(boolean shot, int category) {
    Logger.recordOutput("ShotCounter/Shot", shot); // true on the loop a ball is detected
    Logger.recordOutput("ShotCounter/Armed", detector.isArmed());
    Logger.recordOutput("ShotCounter/TrackingError", detector.getLastError());
    if (shot) {
      Logger.recordOutput("ShotCounter/LastShotCategory", CATEGORY_NAMES[category]);
      Logger.recordOutput("ShotCounter/LastDipDepth", detector.getLastDipDepth());
    }

    int total = 0;
    for (int c = 0; c < 3; c++) {
      int cShots = shots[0][c] + shots[1][c];
      double cFeed = feedSec[0][c] + feedSec[1][c];
      double cTrigger = triggerSec[0][c] + triggerSec[1][c];
      total += cShots;
      Logger.recordOutput(KEYS[c][0], shots[0][c]);
      Logger.recordOutput(KEYS[c][1], shots[1][c]);
      Logger.recordOutput(KEYS[c][2], cShots);
      Logger.recordOutput(KEYS[c][3], cFeed);
      Logger.recordOutput(KEYS[c][4], cTrigger);
      // Balls per second of feeder-gate-open time (includes time with an empty hopper).
      Logger.recordOutput(KEYS[c][5], cFeed > 1.0 ? cShots / cFeed : 0.0);
      // Balls per second of shoot-button time (adds spin-up and aiming time).
      Logger.recordOutput(KEYS[c][6], cTrigger > 1.0 ? cShots / cTrigger : 0.0);
      // Rate while balls are actually flowing: shots inside bursts / burst duration.
      Logger.recordOutput(KEYS[c][7], burstSec[c] > 0.5 ? burstIntervals[c] / burstSec[c] : 0.0);
    }
    Logger.recordOutput("ShotCounter/Total", total);
    Logger.recordOutput("ShotCounter/HubWhileInactive", hubShotsWhileInactive);
    Logger.recordOutput("ShotCounter/SinceBoot", shotsSinceBoot);
  }

  public int getMatchTotal(int category) {
    return shots[0][category] + shots[1][category];
  }

  /** Shots since the code started. Resets on reboot; the usage tracker should persist it. */
  public int getShotsSinceBoot() {
    return shotsSinceBoot;
  }
}
