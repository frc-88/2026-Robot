package frc.robot.util.usage;

import java.util.Objects;
import java.util.Properties;

/**
 * An odometer for one wear part, such as the tread on the front-left swerve module.
 *
 * <p>This class is pure bookkeeping: it adds up usage and tracks when the next inspection is due.
 * It knows nothing about motors, files, or dashboards, which is what lets it be unit-tested on a
 * laptop. {@link TreadUsageTracker} feeds it and shows its state.
 *
 * <p>It keeps two running totals, in whatever unit the caller uses (meters, for tread):
 *
 * <ul>
 *   <li><b>lifetime</b> — never reset. Used for cross-checks and recovery.
 *   <li><b>since replacement</b> — set back to zero when the part is replaced. Inspection warnings
 *       and the service log use this one.
 * </ul>
 *
 * <p>The next inspection is due when "since replacement" reaches {@code nextInspectionAt}. That
 * point moves in two ways:
 *
 * <ul>
 *   <li>{@link #inspected(Grade)} sets it to <i>the current distance</i> plus the grade's snooze
 *       distance. It is measured from where the inspection happened, not added to the old due
 *       point, so a late inspection still earns the full snooze.
 *   <li>{@link #replaced(String)} sets it to the schedule's first-inspection distance.
 * </ul>
 */
public final class UsageMeter {
  /** Label used until a part's design has been recorded with a replacement. */
  public static final String UNKNOWN_LABEL = "unknown";

  /**
   * How far a part may go before its first inspection, and after each inspection grade. All
   * values are in the meter's unit (meters, for tread).
   */
  public record Schedule(
      double firstInspection, double afterGood, double afterWorn, double afterPoor) {
    /** How much further the part may go after being graded {@code grade}. */
    public double snooze(Grade grade) {
      return switch (grade) {
        case GOOD -> afterGood;
        case WORN -> afterWorn;
        case POOR -> afterPoor;
      };
    }
  }

  private final String name;
  private final Schedule schedule;

  private double lifetime = 0.0;
  private double sinceReplacement = 0.0;
  private double nextInspectionAt;
  private String label = UNKNOWN_LABEL;
  private Grade lastGrade = null;
  private double lastGradedAt = Double.NaN;

  /**
   * Creates a meter with no usage recorded.
   *
   * @param name unique name, also used as the key prefix when saving, e.g. {@code "Tread/FL"}
   * @param schedule inspection distances for this kind of part
   */
  public UsageMeter(String name, Schedule schedule) {
    this.name = Objects.requireNonNull(name);
    this.schedule = Objects.requireNonNull(schedule);
    this.nextInspectionAt = schedule.firstInspection();
  }

  /**
   * Adds usage. Zero, negative, NaN, and infinite amounts are ignored, so a bad sensor reading
   * can never make the count go backwards or become unreadable.
   */
  public void add(double amount) {
    if (!(amount > 0.0) || Double.isInfinite(amount)) {
      return;
    }
    lifetime += amount;
    sinceReplacement += amount;
  }

  /** True once the part has reached the point where it should be looked at. */
  public boolean isInspectionDue() {
    return sinceReplacement >= nextInspectionAt;
  }

  /** How much more use until the next inspection. Negative means it is overdue by that much. */
  public double remainingUntilInspection() {
    return nextInspectionAt - sinceReplacement;
  }

  /**
   * Records an inspection. The next inspection becomes due after the grade's snooze distance,
   * counted from the current distance. Does not change the "since replacement" total.
   */
  public void inspected(Grade grade) {
    Objects.requireNonNull(grade);
    lastGrade = grade;
    lastGradedAt = sinceReplacement;
    nextInspectionAt = sinceReplacement + schedule.snooze(grade);
  }

  /**
   * Records a replacement: "since replacement" goes back to zero, the new part's design label is
   * stored, and the first-inspection distance applies again. The lifetime total is kept.
   *
   * @param newLabel short code for the new part's design, e.g. {@code "tread-A"}
   */
  public void replaced(String newLabel) {
    sinceReplacement = 0.0;
    nextInspectionAt = schedule.firstInspection();
    label = (newLabel == null || newLabel.isBlank()) ? UNKNOWN_LABEL : newLabel.trim();
    lastGrade = null;
    lastGradedAt = Double.NaN;
  }

  public String getName() {
    return name;
  }

  public double getLifetime() {
    return lifetime;
  }

  public double getSinceReplacement() {
    return sinceReplacement;
  }

  public double getNextInspectionAt() {
    return nextInspectionAt;
  }

  public String getLabel() {
    return label;
  }

  /** The most recent inspection grade since the last replacement, or {@code null} if none. */
  public Grade getLastGrade() {
    return lastGrade;
  }

  /** "Since replacement" distance at the last inspection, or NaN if there has been none. */
  public double getLastGradedAt() {
    return lastGradedAt;
  }

  // ---- Saving and loading ---------------------------------------------------------------------

  /** Writes this meter's state into {@code out}, with every key starting "{@code name.}". */
  public void writeTo(Properties out) {
    String p = name + ".";
    out.setProperty(p + "lifetime", Double.toString(lifetime));
    out.setProperty(p + "sinceReplacement", Double.toString(sinceReplacement));
    out.setProperty(p + "nextInspectionAt", Double.toString(nextInspectionAt));
    out.setProperty(p + "label", label);
    out.setProperty(p + "lastGrade", lastGrade == null ? "none" : lastGrade.name());
    out.setProperty(p + "lastGradedAt", Double.toString(lastGradedAt));
  }

  /**
   * Restores this meter's state from {@code in}. All-or-nothing: if any value is missing or
   * invalid, nothing is changed and this returns false.
   *
   * @return true if the saved state was read in full
   */
  public boolean readFrom(Properties in) {
    String p = name + ".";
    try {
      double newLifetime = parseCount(in.getProperty(p + "lifetime"));
      double newSince = parseCount(in.getProperty(p + "sinceReplacement"));
      double newNext = parseCount(in.getProperty(p + "nextInspectionAt"));
      String newLabel = in.getProperty(p + "label");
      String gradeText = in.getProperty(p + "lastGrade");
      String gradedAtText = in.getProperty(p + "lastGradedAt");
      if (newLabel == null || gradeText == null || gradedAtText == null) {
        return false;
      }
      Grade newGrade = gradeText.equals("none") ? null : Grade.valueOf(gradeText);
      double newGradedAt = Double.parseDouble(gradedAtText);
      if (newSince > newLifetime) {
        return false;
      }

      lifetime = newLifetime;
      sinceReplacement = newSince;
      nextInspectionAt = newNext;
      label = newLabel.isBlank() ? UNKNOWN_LABEL : newLabel;
      lastGrade = newGrade;
      lastGradedAt = newGradedAt;
      return true;
    } catch (IllegalArgumentException e) { // includes NumberFormatException
      return false;
    }
  }

  /** Parses a saved total, which must be a finite number that is zero or more. */
  private static double parseCount(String text) {
    if (text == null) {
      throw new IllegalArgumentException("missing value");
    }
    double value = Double.parseDouble(text);
    if (!Double.isFinite(value) || value < 0.0) {
      throw new IllegalArgumentException("invalid value: " + text);
    }
    return value;
  }
}
