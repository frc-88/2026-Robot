package frc.robot.util.health;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
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
 *       beats warning) through {@link #getEndOfMatchDisplay()} for {@link #DISPLAY_SECONDS};
 *   <li>saves the match record to the roboRIO (WPILib {@link Preferences}) as it changes, so it
 *       survives a power-off. At the next boot it is shown again through {@link
 *       #getPreviousRunDisplay()} and a dashboard alert, until the next enable starts a new record.
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
   * If the first enable after boot comes this soon after robot code starts, with the field system
   * attached, robot code must have restarted mid-match (in a normal match, code has been running
   * for minutes before auto). Logged as {@code Health/RestartedMidMatch}.
   */
  public static final double MID_MATCH_RESTART_WINDOW_SECONDS = 5.0;

  /** Preferences keys for the saved record. Stored on the roboRIO, so they survive power-off. */
  private static final String SAVED_RECORD_KEY = "Health/SavedRecord";

  private static final String SAVED_RECORD_TYPE_KEY = "Health/SavedRecordType";

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
  private final List<Runnable> scopeStartListeners = new ArrayList<>();
  private final Alert latchedSummaryAlert = new Alert("", AlertType.kInfo);
  private final Alert previousRunAlert = new Alert("", AlertType.kInfo);

  private double firstLoopTime = Double.NaN;
  private boolean wasEnabled = false;
  private boolean lastEnabledWasAuto = false;
  private ResetScope scope = ResetScope.PER_ENABLE;

  private AlertType endOfMatchDisplay = null;
  private double displayUntil = 0.0;
  private String lastSummary = "";

  // Saved record / previous-run replay
  private final boolean persistenceEnabled = !Logger.hasReplaySource();
  private final String bootRecord; // record found at boot; never cleared, logged for evidence
  private AlertType previousRunDisplay = null;
  private boolean hasEnabledSinceBoot = false;
  private boolean restartedMidMatch = false;
  private String recordLabel = "";
  private String lastSavedRecord = "";
  private boolean bootRecordLogged = false;

  private HealthMonitor() {
    // Load the record saved before the last power-off / restart, if any, and replay it.
    bootRecord = persistenceEnabled ? Preferences.getString(SAVED_RECORD_KEY, "") : "";
    lastSavedRecord = bootRecord;
    if (!bootRecord.isEmpty()) {
      String savedType = Preferences.getString(SAVED_RECORD_TYPE_KEY, "");
      previousRunDisplay = savedType.equals("ERROR") ? AlertType.kError : AlertType.kWarning;
      previousRunAlert.setText("Before last power-off/restart - " + bootRecord);
      previousRunAlert.set(true);
    }
  }

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

  /**
   * Registers code to run each time a new match record begins: when auto enables in a real match,
   * or on every enable on the bench (see {@link ResetScope}). It runs right after the latches are
   * cleared and before faults are checked that loop, so it can take a snapshot (e.g. the starting
   * battery voltage) that faults in the new record can use. It does not run on the teleop enable
   * that continues a match.
   */
  public void onScopeStart(Runnable listener) {
    scopeStartListeners.add(listener);
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
      if (scope == ResetScope.PER_MATCH
          && !hasEnabledSinceBoot
          && now - firstLoopTime < MID_MATCH_RESTART_WINDOW_SECONDS) {
        // Log-only: the record from before the restart is in Health/PreviousRun.
        restartedMidMatch = true;
      }
      hasEnabledSinceBoot = true;
      boolean continuesMatch = scope == ResetScope.PER_MATCH && !isAuto && lastEnabledWasAuto;
      if (!continuesMatch) {
        resetLatches();
        recordLabel = buildRecordLabel();
        clearSavedRecord(); // a new run starts fresh: nothing carries over from a past one
        for (Runnable listener : scopeStartListeners) {
          listener.run();
        }
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

  /**
   * What a display should show for a record carried over from before the last power-off or code
   * restart.
   *
   * <p>Returns the most urgent type from the saved record ({@link AlertType#kError} or {@link
   * AlertType#kWarning}) from boot until the next enable starts a new record, and null otherwise.
   * Kept separate from {@link #getEndOfMatchDisplay()} so the display can show "carried over from
   * last time" differently from "just happened".
   */
  public AlertType getPreviousRunDisplay() {
    return previousRunDisplay;
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

  /** "bench" without the field system, otherwise e.g. "NECMP2 Q52". */
  private static String buildRecordLabel() {
    if (!DriverStation.isFMSAttached()) {
      return "bench";
    }
    String type;
    switch (DriverStation.getMatchType()) {
      case Practice:
        type = "P";
        break;
      case Qualification:
        type = "Q";
        break;
      case Elimination:
        type = "E";
        break;
      default:
        type = "M";
        break;
    }
    String event = DriverStation.getEventName();
    return (event.isEmpty() ? "" : event + " ") + type + DriverStation.getMatchNumber();
  }

  /** Ends any previous-run replay and erases the saved record. */
  private void clearSavedRecord() {
    previousRunDisplay = null;
    previousRunAlert.set(false);
    saveRecord("", null);
  }

  /** Writes the record to the roboRIO only when it changes (a few writes per match at most). */
  private void saveRecord(String record, AlertType type) {
    if (!persistenceEnabled || record.equals(lastSavedRecord)) {
      return;
    }
    Preferences.setString(SAVED_RECORD_KEY, record);
    Preferences.setString(SAVED_RECORD_TYPE_KEY, type == null ? "" : nameOf(type));
    lastSavedRecord = record;
  }

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
    Logger.recordOutput("Health/PreviousRunDisplay", nameOf(previousRunDisplay));
    Logger.recordOutput("Health/RestartedMidMatch", restartedMidMatch);
    if (!bootRecordLogged) {
      Logger.recordOutput("Health/PreviousRun", bootRecord);
      bootRecordLogged = true;
    }

    // Keep the latched list visible on the dashboard after the underlying alerts clear, so the
    // pit crew can see what happened during the match without opening a log.
    String since = scope == ResetScope.PER_MATCH ? "this match" : "since last enable";
    String list = String.join(", ", latched);
    String summary = latched.isEmpty() ? "" : "Faults " + since + " (" + recordLabel + "): " + list;
    if (!summary.equals(lastSummary)) {
      latchedSummaryAlert.setText(summary);
      latchedSummaryAlert.set(!summary.isEmpty());
      lastSummary = summary;
    }

    // Save as faults are recorded, not just at match end, so a power loss mid-match keeps them.
    // Nothing is saved before the first enable: the in-memory record is empty until then, and the
    // saved record from before power-off must survive until a new run starts.
    if (hasEnabledSinceBoot) {
      saveRecord(latched.isEmpty() ? "" : recordLabel + ": " + list, highestLatched());
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
