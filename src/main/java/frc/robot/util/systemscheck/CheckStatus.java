package frc.robot.util.systemscheck;

/**
 * Outcome of one systems-check measurement.
 *
 * <p>{@link #MEASURED} is for checks that are still collecting baseline data: the value is logged
 * and shown, but no pass/fail limit exists yet. Severity orders the overall result: the worst
 * status across all results decides what the dashboard reports.
 */
public enum CheckStatus {
  PASS(0),
  MEASURED(0),
  SKIPPED(1),
  WARN(2),
  FAIL(3);

  public final int severity;

  CheckStatus(int severity) {
    this.severity = severity;
  }
}
