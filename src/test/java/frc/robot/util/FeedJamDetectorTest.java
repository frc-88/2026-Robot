package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.util.FeedJamDetector.Config;
import frc.robot.util.FeedJamDetector.Stage;
import frc.robot.util.FeedJamDetector.State;
import org.junit.jupiter.api.Test;

/** Checks FeedJamDetector on made-up traces. Stage thresholds match FeedPathMonitor. */
public class FeedJamDetectorTest {
  private static final double DT = 0.02;
  private static final double[] FREE_V = {100.0, 90.0}; // feeder, spinner running free
  private static final double[] FREE_I = {12.0, 8.0};

  private static FeedJamDetector make() {
    return new FeedJamDetector(
        new Config(), new Stage("Feeder", 55.0, 25.0), new Stage("Spinner", 20.0, 40.0));
  }

  /** Runs from t0 to t1 with fixed inputs; shots every shotEvery seconds (0 = none). */
  private static State run(
      FeedJamDetector det,
      double t0,
      double t1,
      boolean gate,
      double shotEvery,
      double[] v,
      double[] i) {
    State s = State.IDLE;
    double nextShot = t0 + 0.3;
    for (double t = t0; t < t1; t += DT) {
      boolean shot = shotEvery > 0 && t >= nextShot;
      if (shot) {
        nextShot += shotEvery;
      }
      s = det.update(t, true, gate, shot, v, i);
    }
    return s;
  }

  @Test
  public void flowingStaysFlowing() {
    FeedJamDetector det = make();
    assertEquals(State.FLOWING, run(det, 0, 5, true, 0.25, FREE_V, FREE_I));
  }

  @Test
  public void startingBeforeFirstBall() {
    FeedJamDetector det = make();
    assertEquals(State.STARTING, run(det, 0, 0.25, true, 0, FREE_V, FREE_I));
  }

  @Test
  public void emptyHopper() {
    FeedJamDetector det = make();
    run(det, 0, 2, true, 0.25, FREE_V, FREE_I);
    assertEquals(State.EMPTY, run(det, 2, 3, true, 0, FREE_V, FREE_I), "motors free, no balls");
  }

  @Test
  public void feederJam() {
    FeedJamDetector det = make();
    run(det, 0, 2, true, 0.25, FREE_V, FREE_I);
    State s = run(det, 2, 3, true, 0, new double[] {30.0, 90.0}, new double[] {75.0, 8.0});
    assertEquals(State.JAMMED, s);
    assertEquals("Feeder", det.getJammedStageName());
  }

  @Test
  public void spinnerJam() {
    // NECMP/CMP pattern: spinner stalled at its current limit, feeder spinning free.
    FeedJamDetector det = make();
    run(det, 0, 2, true, 0.25, FREE_V, FREE_I);
    State s = run(det, 2, 3, true, 0, new double[] {105.0, 0.0}, new double[] {10.0, 60.0});
    assertEquals(State.JAMMED, s);
    assertEquals("Spinner", det.getJammedStageName());
  }

  @Test
  public void boggedWhileBallsFlowIsNotAJam() {
    // October feeder: slow and loaded during normal feeding, but balls keep coming out.
    FeedJamDetector det = make();
    State s = run(det, 0, 5, true, 0.3, new double[] {35.0, 90.0}, new double[] {70.0, 8.0});
    assertEquals(State.FLOWING, s);
  }

  @Test
  public void jamStaysLatchedUntilABallComesOut() {
    FeedJamDetector det = make();
    run(det, 0, 2, true, 0.25, FREE_V, FREE_I);
    run(det, 2, 3, true, 0, new double[] {30.0, 90.0}, new double[] {75.0, 8.0});
    // Feeder creeps back above its threshold, but still no balls: still a jam.
    assertEquals(State.JAMMED, run(det, 3, 4, true, 0, new double[] {60.0, 90.0}, FREE_I));
    // A ball finally comes out.
    assertEquals(State.FLOWING, run(det, 4, 4.4, true, 0.1, FREE_V, FREE_I));
  }

  @Test
  public void deadMotorIsNotAJam() {
    // 0 rps and 0 A is a motor that isn't running (a health-monitor fault), not a jam.
    FeedJamDetector det = make();
    run(det, 0, 2, true, 0.25, FREE_V, FREE_I);
    assertEquals(State.EMPTY, run(det, 2, 3, true, 0, new double[] {0.0, 90.0}, FREE_I));
  }

  @Test
  public void gateFlickerDoesNotRestartTheClock() {
    FeedJamDetector det = make();
    run(det, 0, 2, true, 0.25, FREE_V, FREE_I);
    run(det, 2, 2.4, true, 0, FREE_V, FREE_I); // 0.4 s starved
    run(det, 2.4, 2.5, false, 0, FREE_V, FREE_I); // gate drops briefly (turret flicker)
    State s = run(det, 2.5, 2.8, true, 0, FREE_V, FREE_I); // 0.3 s more: 0.7 s total
    assertEquals(State.EMPTY, s);
    assertTrue(det.getStarvedSec() > 0.6);
  }

  @Test
  public void shooterOffIsIdle() {
    FeedJamDetector det = make();
    run(det, 0, 2, true, 0, new double[] {30.0, 90.0}, new double[] {75.0, 8.0});
    assertEquals(State.IDLE, det.update(2.0, false, false, false, FREE_V, FREE_I));
  }
}
