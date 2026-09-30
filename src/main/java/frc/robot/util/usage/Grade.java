package frc.robot.util.usage;

/**
 * The condition grade someone records when they inspect a wear part (for example, wheel tread).
 *
 * <p>The grade decides how much more use the part gets before the next inspection warning. A part
 * in good shape can go longer; a worn one gets checked again sooner. The distances for each grade
 * are set per part in a {@link UsageMeter.Schedule}.
 */
public enum Grade {
  /** No visible wear concern. */
  GOOD("Good"),
  /** Visible wear, but fine for now. */
  WORN("Worn"),
  /** Should be replaced; the team is choosing to run it a little longer. */
  POOR("Poor");

  private final String label;

  Grade(String label) {
    this.label = label;
  }

  /** The word shown on the dashboard and written to the service log, e.g. "Good". */
  public String label() {
    return label;
  }

  /**
   * Finds the grade matching a dashboard label ("Good", "Worn", "Poor"), ignoring upper/lower
   * case.
   *
   * @return the grade, or {@code null} if the text is not a grade (e.g. the "(select grade)"
   *     placeholder)
   */
  public static Grade fromLabel(String text) {
    if (text == null) {
      return null;
    }
    for (Grade grade : values()) {
      if (grade.label.equalsIgnoreCase(text.trim())) {
        return grade;
      }
    }
    return null;
  }
}
