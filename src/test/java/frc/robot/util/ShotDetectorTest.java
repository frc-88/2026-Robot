package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import org.junit.jupiter.api.Test;

/**
 * Checks ShotDetector against made-up flywheel traces. Run with ./gradlew test. These don't need a
 * robot or a log file.
 */
public class ShotDetectorTest {
  private static final double DT = 0.02; // 50 Hz robot loop
  private static final double SETPOINT = 41.0; // typical hub shot, rps

  /** Flywheel speed for a ball that hits at tHit: drops ~6 rps, recovers in ~0.1 s. */
  private static double ballDip(double t, double tHit) {
    double x = t - tHit;
    if (x < 0 || x > 0.14) {
      return 0.0;
    }
    return x < 0.04 ? -6.0 * (x / 0.04) : -6.0 * (1.0 - (x - 0.04) / 0.10);
  }

  private static int run(
      double seconds, double[] ballTimes, boolean gate, double setpointRampPerSec, long seed) {
    ShotDetector det = new ShotDetector();
    Random noise = new Random(seed);
    int count = 0;
    for (double t = 0; t < seconds; t += DT) {
      double sp = SETPOINT + setpointRampPerSec * t;
      double v = sp + 0.3 * noise.nextGaussian(); // ~0.3 rps measurement noise
      for (double tb : ballTimes) {
        v += ballDip(t, tb);
      }
      if (det.update(t, v, sp, true, gate)) {
        count++;
      }
    }
    return count;
  }

  private static double[] evenlySpaced(int n, double start, double spacing) {
    double[] out = new double[n];
    for (int i = 0; i < n; i++) {
      out[i] = start + i * spacing;
    }
    return out;
  }

  @Test
  public void fewFalseCountsWithNoBalls() {
    // 0.3 rps is the measured speed noise at a steady hold (Oct 8-9 practice logs). Noise alone
    // occasionally makes a 1.5 rps dip; the real no-ball sessions showed about 1 count per 20 s,
    // so allow up to 2 in 30 s here.
    int falseCounts = run(30.0, new double[0], true, 0.0, 1);
    assertTrue(falseCounts <= 2, "steady flywheel with noise gave " + falseCounts);
  }

  @Test
  public void countsEachBall() {
    double[] balls = evenlySpaced(20, 1.0, 0.25); // 4 balls per second
    assertEquals(20, run(7.0, balls, true, 0.0, 2), "20 balls at 4 per second");
  }

  @Test
  public void countsBallsWhileSetpointMoves() {
    // Driving toward the hub at ~2 m/s moves the setpoint ~8 rps/s; that alone isn't a ball.
    // (Kept to 3 s so the setpoint stays well above the 10 rps minimum.)
    double[] balls = evenlySpaced(10, 0.4, 0.25);
    assertEquals(10, run(3.0, balls, true, -8.0, 3), "setpoint ramping down");
    assertTrue(run(3.0, new double[0], true, -8.0, 4) <= 1, "ramp with no balls");
  }

  @Test
  public void setpointStepDownIsNotABall() {
    ShotDetector det = new ShotDetector();
    int count = 0;
    double v = 55.0;
    for (double t = 0; t < 3.0; t += DT) {
      double sp = t < 1.0 ? 55.0 : 45.0; // aim target switches, setpoint drops 10 rps
      v += (sp - v) * 0.15; // flywheel brakes toward the new setpoint
      if (det.update(t, v, sp, true, true)) {
        count++;
      }
    }
    assertEquals(0, count, "flywheel braking after a setpoint drop");
  }

  @Test
  public void coastingDownToNewSetpointIsNotABall() {
    // Same as above but the flywheel slows at a coast-like 12 rps/s and stops at the setpoint.
    ShotDetector det = new ShotDetector();
    int count = 0;
    double v = 55.0;
    for (double t = 0; t < 3.0; t += DT) {
      double sp = t < 1.0 ? 55.0 : 45.0;
      v = Math.max(sp, v - 12.0 * DT);
      if (det.update(t, v, sp, true, true)) {
        count++;
      }
    }
    assertEquals(0, count, "flywheel coasting down after a setpoint drop");
  }

  @Test
  public void nothingCountedWithFeederClosed() {
    double[] balls = evenlySpaced(5, 1.0, 0.3);
    assertEquals(0, run(3.0, balls, false, 0.0, 5), "gate closed the whole time");
  }

  @Test
  public void oneLongDipCountsOnce() {
    ShotDetector det = new ShotDetector();
    int count = 0;
    for (double t = 0; t < 2.0; t += DT) {
      double v = SETPOINT;
      if (t > 1.0 && t < 1.4) {
        v -= 8.0 * (t - 1.0) / 0.4; // slow 0.4 s sag with no recovery in between
      }
      if (det.update(t, v, SETPOINT, true, true)) {
        count++;
      }
    }
    assertEquals(1, count, "a single long sag");
  }
}
