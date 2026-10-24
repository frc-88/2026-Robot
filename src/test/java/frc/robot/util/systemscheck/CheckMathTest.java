package frc.robot.util.systemscheck;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class CheckMathTest {
  private static final double EPS = 1e-9;

  @Test
  void twoPointFitRecoversKnownLine() {
    // volts = 0.3 + 2.7 * speed
    CheckMath.Fit fit = CheckMath.twoPointFit(0.3 + 2.7 * 0.6, 0.6, 0.3 + 2.7 * 1.7, 1.7);
    assertEquals(0.3, fit.kS(), EPS);
    assertEquals(2.7, fit.kV(), EPS);
  }

  @Test
  void twoPointFitIsNaNWhenSpeedDoesNotChange() {
    CheckMath.Fit fit = CheckMath.twoPointFit(2.0, 0.0, 5.0, 0.0); // jammed wheel
    assertTrue(Double.isNaN(fit.kS()));
    assertTrue(Double.isNaN(fit.kV()));
  }

  @Test
  void medianOfOthersExcludesSelfAndUnincluded() {
    double[] values = {10.0, 1.0, 2.0, 3.0};
    boolean[] all = {true, true, true, true};
    assertEquals(2.0, CheckMath.medianOfOthers(values, all, 0), EPS); // median of 1,2,3
    assertEquals(3.0, CheckMath.medianOfOthers(values, all, 1), EPS); // median of 10,2,3

    boolean[] noLast = {true, true, true, false};
    assertEquals(1.5, CheckMath.medianOfOthers(values, noLast, 0), EPS); // median of 1,2
  }

  @Test
  void medianOfOthersSkipsNaNAndHandlesNone() {
    double[] values = {1.0, Double.NaN, 3.0, 5.0};
    boolean[] all = {true, true, true, true};
    assertEquals(4.0, CheckMath.medianOfOthers(values, all, 0), EPS); // median of 3,5
    assertTrue(Double.isNaN(CheckMath.medianOfOthers(values, new boolean[4], 0)));
  }

  @Test
  void ratioToOthersFlagsTheOddModule() {
    double[] kS = {0.6, 0.3, 0.3, 0.3}; // FL needs twice the static voltage
    boolean[] all = {true, true, true, true};
    assertEquals(2.0, CheckMath.ratioToOthers(kS, all, 0), EPS);
    assertEquals(1.0, CheckMath.ratioToOthers(kS, all, 1), EPS);
  }

  @Test
  void ratioToOthersIsNaNWhenMedianIsZero() {
    double[] values = {1.0, 0.0, 0.0, 0.0};
    boolean[] all = {true, true, true, true};
    assertTrue(Double.isNaN(CheckMath.ratioToOthers(values, all, 0)));
  }
}
