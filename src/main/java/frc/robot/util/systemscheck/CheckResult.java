package frc.robot.util.systemscheck;

/**
 * One row of the systems-check report.
 *
 * <p>{@code low}/{@code high} are the pass band; NaN while a check is measurement-only. {@code
 * action} is what the pit crew should do when the status is WARN, FAIL or SKIPPED (empty
 * otherwise), following the health monitor's rule that every listed item names its action.
 */
public record CheckResult(
    String name,
    double value,
    String units,
    double low,
    double high,
    CheckStatus status,
    String action) {

  /** A logged value with no pass/fail limit yet (baseline collection). */
  public static CheckResult measured(String name, double value, String units) {
    return new CheckResult(name, value, units, Double.NaN, Double.NaN, CheckStatus.MEASURED, "");
  }

  /** A check that could not run. */
  public static CheckResult skipped(String name, String action) {
    return new CheckResult(
        name, Double.NaN, "", Double.NaN, Double.NaN, CheckStatus.SKIPPED, action);
  }

  /** A yes/no outcome, e.g. an operator confirmation. */
  public static CheckResult passFail(String name, boolean passed, String failAction) {
    return new CheckResult(
        name,
        passed ? 1.0 : 0.0,
        "",
        Double.NaN,
        Double.NaN,
        passed ? CheckStatus.PASS : CheckStatus.FAIL,
        passed ? "" : failAction);
  }

  /** One dashboard line, e.g. "MEASURED FL drive kS = 0.312 V". */
  public String toLine() {
    String text = String.format("%-8s  %s", status, name);
    if (!Double.isNaN(value) && !units.isEmpty()) {
      text += String.format(" = %.3f %s", value, units);
    }
    if (!Double.isNaN(low) || !Double.isNaN(high)) {
      text += String.format("  [%.3f, %.3f]", low, high);
    }
    if (!action.isEmpty()) {
      text += "  -> " + action;
    }
    return text;
  }
}
