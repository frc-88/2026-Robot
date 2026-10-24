package frc.robot.util.systemscheck;

import java.util.Arrays;

/** Pure math used by the systems checks. No WPILib dependencies, so it is unit tested directly. */
public final class CheckMath {
  private CheckMath() {}

  /** Static-friction voltage and velocity constant from two steady-state points. */
  public record Fit(double kS, double kV) {}

  /**
   * Fits volts = kS + kV * speed through two steady-state (volts, speed) points.
   *
   * @return NaN for both constants when the two speeds are effectively equal (mechanism did not
   *     speed up between the steps, e.g. jammed or disconnected), since the slope is undefined.
   */
  public static Fit twoPointFit(double volts1, double speed1, double volts2, double speed2) {
    double dSpeed = speed2 - speed1;
    if (!Double.isFinite(dSpeed) || Math.abs(dSpeed) < 1e-6) {
      return new Fit(Double.NaN, Double.NaN);
    }
    double kV = (volts2 - volts1) / dSpeed;
    return new Fit(volts1 - kV * speed1, kV);
  }

  /**
   * Median of every included, finite value except {@code values[index]}. NaN if none qualify.
   *
   * <p>Used to compare one swerve module against the other three: a module is judged against its
   * siblings rather than a fixed number, which cancels battery, temperature and grease state.
   */
  public static double medianOfOthers(double[] values, boolean[] include, int index) {
    double[] others = new double[values.length];
    int n = 0;
    for (int i = 0; i < values.length; i++) {
      if (i != index && include[i] && Double.isFinite(values[i])) {
        others[n++] = values[i];
      }
    }
    if (n == 0) {
      return Double.NaN;
    }
    double[] sorted = Arrays.copyOf(others, n);
    Arrays.sort(sorted);
    return n % 2 == 1 ? sorted[n / 2] : (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0;
  }

  /** {@code values[index]} divided by the median of the others; NaN if that median is 0 or NaN. */
  public static double ratioToOthers(double[] values, boolean[] include, int index) {
    double median = medianOfOthers(values, include, index);
    if (!Double.isFinite(median) || median == 0.0 || !Double.isFinite(values[index])) {
      return Double.NaN;
    }
    return values[index] / median;
  }
}
