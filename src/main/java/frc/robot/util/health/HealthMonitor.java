package frc.robot.util.health;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Layer 2 of the health system: the aggregator.
 *
 * <p>Every {@link Fault} registers itself here when it is created. {@link #periodic()} (called once
 * per loop from {@code Robot.robotPeriodic()}) then:
 *
 * <ol>
 *   <li>polls every fault, skipping a short startup grace period after boot;
 *   <li>answers "is anything wrong right now?" (live, for the dashboard);
 *   <li>keeps a latch answering "did anything go wrong this match?", reset at the start of each
 *       match (or each enable on the bench — see {@link ResetScope});
 *   <li>at the end of the match, if anything latched, publishes the most urgent latched type (error
 *       beats warning) through {@link #getEndOfMatchDisplay()} for {@link #DISPLAY_SECONDS}.
 * </ol>
 *
 * <p>This class is generic: it knows nothing about CAN, motors, or LEDs. It only reads robot mode
 * from the {@link DriverStation} and never commands any hardware.
 *
 * <p><b>Displaying the result (LEDs etc.):</b> this class does not push anything to a display.
 * Whatever shows health to people — an LED subsystem, for example — asks {@link
 * #getEndOfMatchDisplay()} in its own {@code periodic()} and decides for itself what to show and
 * how it ranks against its other patterns (alliance color, game piece, ...). That keeps every LED
 * decision in the LED code, and keeps this class free of any LED knowledge.
 */
public final class HealthMonitor {
  /** Faults are not evaluated for this long after the first loop; devices read absent at boot. */
  public static final double STARTUP_GRACE_SECONDS = 2.0;

  /** How long {@link #getEndOfMatchDisplay()} reports a result after the match ends. */
  public static final double DISPLAY_SECONDS = 5.0;

  /**
   * When the latch is cleared.
   *
   * <ul>
   *   <li>{@link #PER_MATCH} (FMS attached): one scope spans auto + teleop. Cleared when auto
   *       enables; the teleop enable that follows auto continues the same scope. The display fires
   *       only when teleop ends, not in the gap between auto and teleop.
   *   <li>{@link #PER_ENABLE} (no FMS, i.e. bench/practice): every enable is its own test, cleared
   *       on enable and displayed on disable.
   * </ul>
   *
   * <p>Chosen automatically at each enable from {@link DriverStation#isFMSAttached()}.
   */
  public enum ResetScope {
    PER_MATCH,
    PER_ENABLE
  }

  private static HealthMonitor instance;

  public static HealthMonitor getInstance() {
    if (instance == null) {
      instance = new HealthMonitor();
    }
    return instance;
  }

  private final List<Fault> faults = new ArrayList<>();
  private final Alert latchedSummaryAlert = new Alert("", AlertType.kInfo);

  private double firstLoopTime = Double.NaN;
  private boolean wasEnabled = false;
  private boolean lastEnabledWasAuto = false;
  private ResetScope scope = ResetScope.PER_ENABLE;

  private AlertType endOfMatchDisplay = null;
  private double displayUntil = 0.0;
  private String lastSummary = "";

  private HealthMonitor() {}

  /** Called from the {@link Fault} constructor. */
  void register(Fault fault) {
    for (Fault existing : faults) {
      if (existing.getKey().equals(fault.getKey())) {
        DriverStation.reportWarning(
            "HealthMonitor: duplicate fault key '" + fault.getKey() + "'", false);
      }
    }
    faults.add(fault);
  }

  /** Call once per loop from {@code robotPeriodic()}, after the command scheduler has run. */
  public void periodic() {
    double now = Timer.getTimestamp();
    if (Double.isNaN(firstLoopTime)) {
      firstLoopTime = now;
    }
    boolean inGrace = now - firstLoopTime < STARTUP_GRACE_SECONDS;
    boolean enabled = DriverStation.isEnabled();

    // --- Scope start: disabled -> enabled ---
    if (enabled && !wasEnabled) {
      boolean isAuto = DriverStation.isAutonomous();
      scope = DriverStation.isFMSAttached() ? ResetScope.PER_MATCH : ResetScope.PER_ENABLE;
      boolean continuesMatch = scope == ResetScope.PER_MATCH && !isAuto && lastEnabledWasAuto;
      if (!continuesMatch) {
        resetLatches();
      }
      endOfMatchDisplay = null; // re-enabling ends any display in progress
    }

    // --- Detection ---
    for (Fault fault : faults) {
      fault.update(!inGrace, enabled);
    }

    // --- Scope end: enabled -> disabled ---
    if (!enabled && wasEnabled) {
      boolean autoToTeleopGap = scope == ResetScope.PER_MATCH && lastEnabledWasAuto;
      if (!autoToTeleopGap) {
        AlertType worst = highestLatched();
        if (worst != null) {
          endOfMatchDisplay = worst;
          displayUntil = now + DISPLAY_SECONDS;
        }
      }
    }

    if (endOfMatchDisplay != null && now >= displayUntil) {
      endOfMatchDisplay = null;
    }

    if (enabled) {
      lastEnabledWasAuto = DriverStation.isAutonomous();
    }
    wasEnabled = enabled;

    logAndPublish(inGrace);
  }

  // ----- Queries (live, latched, and display) -----

  /**
   * What a display (e.g. the LEDs) should currently show for robot health.
   *
   * <p>Returns {@link AlertType#kError} or {@link AlertType#kWarning} for {@link #DISPLAY_SECONDS}
   * after a match (or bench enable) ends with a latched fault, and null the rest of the time. Call
   * it every loop from the display's own {@code periodic()}; which color or pattern each type gets
   * is the display's decision.
   */
  public AlertType getEndOfMatchDisplay() {
    return endOfMatchDisplay;
  }

  /** True if any fault is active right now. */
  public boolean anyActive() {
    return highestActive() != null;
  }

  /** Most urgent type active right now (kError beats kWarning), or null if nothing is active. */
  public AlertType highestActive() {
    AlertType worst = null;
    for (Fault fault : faults) {
      if (fault.isActive()) worst = moreUrgent(worst, fault.getType());
    }
    return worst;
  }

  /** True if any fault has latched since the last reset. */
  public boolean latchedThisMatch() {
    return highestLatched() != null;
  }

  /** Most urgent type latched since the last reset, or null if nothing latched. */
  public AlertType highestLatched() {
    AlertType worst = null;
    for (Fault fault : faults) {
      if (fault.hasLatched()) worst = moreUrgent(worst, fault.getType());
    }
    return worst;
  }

  /** Read-only view of every registered fault. */
  public List<Fault> getFaults() {
    return List.copyOf(faults);
  }

  public ResetScope getScope() {
    return scope;
  }

  /** Clears every fault's latch. Called automatically at scope start; public for manual use. */
  public void resetLatches() {
    for (Fault fault : faults) {
      fault.resetLatch();
    }
  }

  // ----- Internals -----

  private void logAndPublish(boolean inGrace) {
    List<String> active = new ArrayList<>();
    List<String> latched = new ArrayList<>();
    for (Fault fault : faults) {
      Logger.recordOutput(fault.getLogKey(), fault.isActive());
      if (fault.isActive()) {
        active.add(fault.getKey());
      }
      if (fault.hasLatched()) {
        latched.add(
            fault.getOccurrences() > 1
                ? fault.getKey() + " (x" + fault.getOccurrences() + ")"
                : fault.getKey());
      }
    }

    Logger.recordOutput("Health/InStartupGrace", inGrace);
    Logger.recordOutput("Health/ResetScope", scope.name());
    Logger.recordOutput("Health/ActiveFaults", active.toArray(new String[0]));
    Logger.recordOutput("Health/HighestActive", nameOf(highestActive()));
    Logger.recordOutput("Health/LatchedFaults", latched.toArray(new String[0]));
    Logger.recordOutput("Health/HighestLatched", nameOf(highestLatched()));
    Logger.recordOutput("Health/EndOfMatchDisplay", nameOf(endOfMatchDisplay));

    // Keep the latched list visible on the dashboard after the underlying alerts clear, so the
    // pit crew can see what happened during the match without opening a log.
    String since = scope == ResetScope.PER_MATCH ? "this match" : "since last enable";
    String summary = latched.isEmpty() ? "" : "Faults " + since + ": " + String.join(", ", latched);
    if (!summary.equals(lastSummary)) {
      latchedSummaryAlert.setText(summary);
      latchedSummaryAlert.set(!summary.isEmpty());
      lastSummary = summary;
    }
  }

  /**
   * Returns the more urgent of two alert types: kError beats kWarning beats kInfo. Either argument
   * may be null, meaning "nothing".
   */
  public static AlertType moreUrgent(AlertType a, AlertType b) {
    if (a == null) return b;
    if (b == null) return a;
    return rank(a) >= rank(b) ? a : b;
  }

  private static int rank(AlertType type) {
    switch (type) {
      case kError:
        return 2;
      case kWarning:
        return 1;
      default:
        return 0;
    }
  }

  /** Log-friendly name: "ERROR", "WARNING", or "NONE". */
  private static String nameOf(AlertType type) {
    if (type == null) return "NONE";
    switch (type) {
      case kError:
        return "ERROR";
      case kWarning:
        return "WARNING";
      default:
        return "INFO";
    }
  }
}
