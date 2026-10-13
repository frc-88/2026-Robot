package frc.robot.util;

/**
 * Tells a jam apart from an empty hopper, using the shot counter plus the ball-path motors.
 *
 * <p>Both look the same from the shooter: the feeder is told to feed and no balls come out. The
 * difference shows up in the motors that push balls toward the shooter:
 *
 * <ul>
 *   <li><b>Empty:</b> every ball-path motor spins near its commanded speed on low current. There's
 *       nothing to push.
 *   <li><b>Jammed:</b> at least one ball-path motor is "bogged": far below its commanded speed
 *       while drawing high current, because it's pushing against balls that won't move.
 * </ul>
 *
 * <p>A bogged motor on its own is not a jam. On the October feeder, bogged is normal while balls
 * are flowing. And a motor that bogs for a moment usually clears itself. The detector only calls
 * a jam once no ball has come out for {@code starveSec} of feeding <i>and</i> a stage has been
 * bogged for at least {@code bogHoldSec}. Once called, the jam stays latched until a ball comes
 * out or the shoot command ends, so a motor hovering around its threshold doesn't flip the state
 * between jammed and empty.
 *
 * <p>Logged anti-jam presses in 2026 showed jams at both stages: the HotTub spinner stalled (0
 * rps, ~60 A, feeder spinning freely) at NECMP and CMP, and the feeder bogged (15-45 rps, 55-90 A,
 * floor spinning freely) in the October practices.
 *
 * <p>No WPILib dependencies, so it can be unit tested and mirrored by the offline tool.
 */
public class FeedJamDetector {
  public enum State {
    /** Shoot command not running. */
    IDLE,
    /** Feeding, no ball yet this window (spin-up and ball travel). */
    STARTING,
    /** Balls are coming out. */
    FLOWING,
    /** Feeding, no balls for starveSec, nothing bogged: hopper empty, or balls not reaching it. */
    EMPTY,
    /** Feeding, no balls for starveSec, and a stage is bogged. */
    JAMMED
  }

  /** One motor in the ball path, upstream of the shooter. */
  public static class Stage {
    public final String name;
    /** Below this speed (rps) the motor counts as slowed down... */
    public final double bogVelocity;
    /** ...but only if it's also drawing more than this (A). Excludes an unpowered motor. */
    public final double bogCurrent;

    public Stage(String name, double bogVelocity, double bogCurrent) {
      this.name = name;
      this.bogVelocity = bogVelocity;
      this.bogCurrent = bogCurrent;
    }
  }

  public static class Config {
    /** Feeding time with no ball out before it's called starved (s). */
    public double starveSec = 0.6;
    /** How long a stage must stay bogged to count (s). */
    public double bogHoldSec = 0.3;
  }

  private final Config config;
  private final Stage[] stages;
  private final double[] bogSec;
  private final boolean[] bogged;

  private double lastTime = Double.NaN;
  private double starvedSec = 0.0;
  private boolean hadShotThisWindow = false;
  private State state = State.IDLE;
  private int jammedStage = -1;

  public FeedJamDetector(Config config, Stage... stages) {
    this.config = config;
    this.stages = stages;
    this.bogSec = new double[stages.length];
    this.bogged = new boolean[stages.length];
  }

  /**
   * Call once per loop.
   *
   * @param timestampSec loop time (s)
   * @param shooterOn shoot command running
   * @param feederGateOpen feeder being told to feed
   * @param shotThisLoop the shot counter detected a ball on this loop
   * @param velocity each stage's measured velocity (rps), same order as the stages
   * @param current each stage's measured current (A), same order as the stages
   */
  public State update(
      double timestampSec,
      boolean shooterOn,
      boolean feederGateOpen,
      boolean shotThisLoop,
      double[] velocity,
      double[] current) {
    double dt = Double.isNaN(lastTime) ? 0.0 : Math.min(timestampSec - lastTime, 0.2);
    lastTime = timestampSec;

    if (!shooterOn) {
      starvedSec = 0.0;
      hadShotThisWindow = false;
      for (int i = 0; i < stages.length; i++) {
        bogSec[i] = 0.0;
        bogged[i] = false;
      }
      jammedStage = -1;
      state = State.IDLE;
      return state;
    }

    if (shotThisLoop) {
      starvedSec = 0.0;
      hadShotThisWindow = true;
      jammedStage = -1; // a ball came out, so whatever was jammed has cleared
    } else if (feederGateOpen) {
      starvedSec += dt; // only feeding time counts, so gate flicker doesn't restart the clock
    }

    for (int i = 0; i < stages.length; i++) {
      bogged[i] =
          feederGateOpen
              && velocity[i] < stages[i].bogVelocity
              && current[i] > stages[i].bogCurrent;
      bogSec[i] = bogged[i] ? bogSec[i] + dt : 0.0;
    }

    if (starvedSec < config.starveSec) {
      state = hadShotThisWindow ? State.FLOWING : State.STARTING;
    } else {
      if (jammedStage < 0) { // not latched yet: blame the stage that has been bogged longest
        double longest = config.bogHoldSec;
        for (int i = 0; i < stages.length; i++) {
          if (bogSec[i] >= longest) {
            longest = bogSec[i];
            jammedStage = i;
          }
        }
      }
      state = jammedStage >= 0 ? State.JAMMED : State.EMPTY;
    }
    return state;
  }

  public State getState() {
    return state;
  }

  /** Index of the stage blamed for the jam, or -1. */
  public int getJammedStage() {
    return jammedStage;
  }

  public String getJammedStageName() {
    return jammedStage >= 0 ? stages[jammedStage].name : "None";
  }

  public boolean isBogged(int stage) {
    return bogged[stage];
  }

  /** Feeding time since the last ball came out (s). */
  public double getStarvedSec() {
    return starvedSec;
  }

  public int getStageCount() {
    return stages.length;
  }

  public String getStageName(int stage) {
    return stages[stage].name;
  }
}
