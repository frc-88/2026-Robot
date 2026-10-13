package frc.robot.util;

/**
 * Detects individual balls leaving the shooter from the flywheel's speed.
 *
 * <p>Each ball that passes through the shooter pulls energy out of the flywheel, so the measured
 * speed drops sharply for about a tenth of a second and then the velocity loop pulls it back up.
 * This class watches for those dips. It has no WPILib or hardware dependencies, so it can be unit
 * tested on a laptop and its logic matches the offline Python tool (shot_counter.py) line for line.
 *
 * <p>The signal it watches is the tracking error, e = velocity - setpoint, not the raw velocity.
 * When the robot drives toward or away from the target the setpoint moves, and the flywheel speed
 * follows it; subtracting the setpoint keeps that motion from looking like a ball.
 *
 * <p>How a shot is counted, each loop:
 *
 * <ol>
 *   <li>Only look while armed: the shooter is commanded on, the feeder gate is open (or closed for
 *       less than {@code hangoverSec}, since balls already in the path still come out), and the
 *       flywheel is above {@code minVelocity}.
 *   <li>Keep the highest error seen over the last {@code windowSec}.
 *   <li>If the error falls at least {@code dipThreshold} below that peak, count one shot, as long
 *       as: the peak wasn't more than {@code overspeedMargin} above setpoint, the speed is now at
 *       least {@code belowSetpoint} under setpoint, and the last shot was at least {@code
 *       lockoutSec} ago. The first two rules reject a flywheel that is above its setpoint and
 *       slowing back down to it (after the setpoint drops), which looks like a dip but isn't a
 *       ball; a ball always pulls the flywheel below its setpoint.
 *   <li>After counting, wait for the error to climb {@code rearmThreshold} back up from the bottom
 *       of the dip before another shot can be counted. This stops one long dip from counting twice.
 * </ol>
 */
public class ShotDetector {
  /** Tuning values. Defaults are the values validated against the 2026 match logs. */
  public static class Config {
    /** Drop below the recent peak that counts as a ball, in rotations per second (motor shaft). */
    public double dipThreshold = 1.5;
    /** Recovery from the bottom of a dip needed before the next ball can count (rps). */
    public double rearmThreshold = 0.75;
    /** Minimum time between two counted balls (s). The main calibration knob; see the doc. */
    public double lockoutSec = 0.10;
    /** Below this flywheel speed nothing is counted (rps). */
    public double minVelocity = 10.0;
    /** A peak more than this far above setpoint means the loop is braking, not a ball (rps). */
    public double overspeedMargin = 2.0;
    /** The dip must take the speed at least this far under setpoint (rps). */
    public double belowSetpoint = 0.75;
    /** Keep looking this long after the feeder gate closes, for balls already in the path (s). */
    public double hangoverSec = 0.30;
    /** How far back the peak is taken from (s). */
    public double windowSec = 0.20;
  }

  private static final int BUFFER_SIZE = 32; // > windowSec / shortest loop period, with margin

  private final Config config;

  // Ring buffer of recent (time, error) samples for the rolling peak. Fixed arrays so the
  // detector allocates nothing per loop.
  private final double[] bufTime = new double[BUFFER_SIZE];
  private final double[] bufError = new double[BUFFER_SIZE];
  private int bufStart = 0;
  private int bufCount = 0;

  private boolean inDip = false;
  private double trough = 0.0;
  private double lastShotTime = Double.NEGATIVE_INFINITY;
  private double lastGateOpenTime = Double.NEGATIVE_INFINITY;

  // Exposed for logging and tuning.
  private double lastError = 0.0;
  private double lastDipDepth = 0.0;
  private boolean armed = false;

  public ShotDetector() {
    this(new Config());
  }

  public ShotDetector(Config config) {
    this.config = config;
  }

  /**
   * Feed one loop's worth of data. Call exactly once per robot loop.
   *
   * @param timestampSec loop timestamp in seconds
   * @param velocity measured flywheel velocity, rps (motor shaft)
   * @param setpoint commanded flywheel velocity, rps (motor shaft)
   * @param shooterOn true while the shoot command is running
   * @param feederGateOpen true while the feeder is being told to feed
   * @return true if a ball was detected on this loop
   */
  public boolean update(
      double timestampSec,
      double velocity,
      double setpoint,
      boolean shooterOn,
      boolean feederGateOpen) {
    if (feederGateOpen) {
      lastGateOpenTime = timestampSec;
    }

    armed =
        shooterOn
            && (timestampSec - lastGateOpenTime) <= config.hangoverSec
            && velocity > config.minVelocity;

    if (!armed) {
      clearBuffer();
      inDip = false;
      return false;
    }

    double error = velocity - setpoint;
    lastError = error;

    if (inDip) {
      trough = Math.min(trough, error);
      if (error - trough >= config.rearmThreshold) {
        inDip = false;
        clearBuffer();
      } else {
        return false;
      }
    }

    // Drop samples older than the window, then take the peak of what's left.
    while (bufCount > 0 && bufTime[bufStart] < timestampSec - config.windowSec) {
      bufStart = (bufStart + 1) % BUFFER_SIZE;
      bufCount--;
    }
    double peak = error;
    for (int k = 0; k < bufCount; k++) {
      peak = Math.max(peak, bufError[(bufStart + k) % BUFFER_SIZE]);
    }
    push(timestampSec, error);

    if (peak - error >= config.dipThreshold
        && peak <= config.overspeedMargin
        && error <= -config.belowSetpoint
        && timestampSec - lastShotTime >= config.lockoutSec) {
      lastShotTime = timestampSec;
      lastDipDepth = peak - error;
      inDip = true;
      trough = error;
      return true;
    }
    return false;
  }

  /** Forget all history, e.g. at the start of a match. Does not touch any counts. */
  public void reset() {
    clearBuffer();
    inDip = false;
    lastShotTime = Double.NEGATIVE_INFINITY;
    lastGateOpenTime = Double.NEGATIVE_INFINITY;
  }

  public boolean isArmed() {
    return armed;
  }

  /** Tracking error (velocity - setpoint) seen on the last armed loop, rps. */
  public double getLastError() {
    return lastError;
  }

  /** Depth of the most recent counted dip, rps. Useful for checking the threshold has margin. */
  public double getLastDipDepth() {
    return lastDipDepth;
  }

  public double getLastShotTime() {
    return lastShotTime;
  }

  private void push(double t, double e) {
    if (bufCount == BUFFER_SIZE) { // full: overwrite the oldest
      bufStart = (bufStart + 1) % BUFFER_SIZE;
      bufCount--;
    }
    int idx = (bufStart + bufCount) % BUFFER_SIZE;
    bufTime[idx] = t;
    bufError[idx] = e;
    bufCount++;
  }

  private void clearBuffer() {
    bufStart = 0;
    bufCount = 0;
  }
}
