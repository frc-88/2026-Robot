package frc.robot.util.usage;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;
import java.util.Properties;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Wheel-tread odometer: counts how far each swerve wheel has travelled, warns on the dashboard when
 * a module is due for a tread inspection, and records inspections and replacements.
 *
 * <p>Design: {@code component-usage-tracker-design.md} in the FRC 2026 Robot project.
 *
 * <h2>What it does each loop</h2>
 *
 * <ol>
 *   <li>Reads the four wheel positions the drive already has (no extra CAN traffic) and adds the
 *       distance each wheel turned since last loop, forwards or backwards, enabled or disabled.
 *   <li>Saves the counters to the roboRIO when the robot is disabled, and every minute while
 *       enabled. Saving happens on a background thread so it can never slow the robot loop.
 *   <li>Raises a "Maintenance" dashboard warning for any module past its inspection point.
 *   <li>Logs everything under {@code /RealOutputs/Usage/...}.
 * </ol>
 *
 * <h2>What the pit crew does (dashboard, "Usage" folder, robot disabled)</h2>
 *
 * <ul>
 *   <li><b>Record Inspection</b>: pick the module and a grade (Good / Worn / Poor). The warning
 *       clears, and the next one comes after that grade's distance.
 *   <li><b>Record Replacement</b>: pick the module and type a short label for the new tread's
 *       design. That module's count goes back to zero.
 * </ul>
 *
 * <p>Both actions add a row to {@code /home/lvuser/usage/service-log.csv}, and "Last Action" on the
 * dashboard confirms what was recorded.
 *
 * <h2>Deliberately NOT part of the health system</h2>
 *
 * <p>These warnings are plain {@link Alert}s in their own "Maintenance" group, not health {@code
 * Fault}s. An inspection coming due can wait several matches, so it must never feed the health
 * system's per-match latch or the end-of-match LED display.
 *
 * <h2>Modes</h2>
 *
 * <ul>
 *   <li>Real robot: counts, saves to {@code /home/lvuser/usage}, full dashboard.
 *   <li>Simulation: counts and the dashboard works, but nothing is saved to disk (handy for trying
 *       the pit workflow).
 *   <li>Log replay: does nothing. The saved counters are not AdvantageKit inputs, so reading them
 *       during replay would make replays differ from the real run.
 * </ul>
 */
public class TreadUsageTracker extends SubsystemBase {
  /**
   * Module order matches {@code Drive}: 0 = front left, 1 = front right, 2 = back left, 3 = back
   * right.
   */
  private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};

  /**
   * Inspection distances in meters. Starting values; replace them once the service log has real
   * data. About 240 m per module per match (2026 logs), so 2000 m is roughly 8 matches.
   */
  private static final UsageMeter.Schedule TREAD_SCHEDULE =
      new UsageMeter.Schedule(
          2000.0, // first inspection after a replacement
          2000.0, // after a "Good" inspection
          1000.0, // after a "Worn" inspection
          250.0); // after a "Poor" inspection

  /**
   * Biggest believable wheel movement in one loop. Top speed is about 4.4 m/s, or 0.09 m per 20 ms
   * loop. A bigger jump means the drive motor rebooted and its position reset, so it is skipped.
   */
  private static final double MAX_STEP_METERS = 1.0;

  private static final double SAVE_PERIOD_SECONDS = 60.0;
  private static final Path STORE_DIR = Paths.get("/home/lvuser/usage");

  private static final String DASH = "Usage/";
  private static final String ALERT_GROUP = "Maintenance";
  private static final int NO_SELECTION = -1;
  private static final int ALL_MODULES = 4;
  private static final int DASHBOARD_SUMMARY_EVERY_N_LOOPS = 25; // about twice a second

  private static final DateTimeFormatter TIMESTAMP_FORMAT =
      DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss");
  private static final ZoneId TEAM_TIME_ZONE = ZoneId.of("America/New_York");

  private final Supplier<double[]> wheelPositionsRad;
  private final double wheelRadiusMeters;
  private final boolean active; // false in log replay
  private final UsageStore store; // null in simulation and replay: nothing is saved
  private final ExecutorService saver; // background thread for file writes; null when no store

  private final UsageMeter[] meters = new UsageMeter[MODULE_NAMES.length];
  private final double[] lastPositionRad = new double[MODULE_NAMES.length];
  private final String[] logPrefix = new String[MODULE_NAMES.length];

  private final Alert[] dueAlerts = new Alert[MODULE_NAMES.length];
  private final String[] dueAlertText = new String[MODULE_NAMES.length];
  private final Alert dataMissingAlert =
      new Alert(
          ALERT_GROUP,
          "Tread usage data was not found on the roboRIO, so the counters started from zero."
              + " Expected the first time; otherwise the saved data was lost (e.g. reimage).",
          AlertType.kWarning);
  private final Alert backupUsedAlert =
      new Alert(
          ALERT_GROUP,
          "Tread usage data was loaded from the backup copy; up to one save (about a minute of"
              + " driving) may be missing.",
          AlertType.kInfo);
  private final Alert saveFailedAlert = new Alert(ALERT_GROUP, "", AlertType.kWarning);
  private final Alert serviceLogFailedAlert = new Alert(ALERT_GROUP, "", AlertType.kWarning);

  private final SendableChooser<Integer> moduleChooser = new SendableChooser<>();
  private final SendableChooser<String> gradeChooser = new SendableChooser<>();

  // Written by the background save thread, read by the robot loop.
  private volatile String saveError = null;
  private volatile String serviceLogError = null;

  private boolean wasEnabled = false;
  private double lastSaveTime;
  private int loopCount = 0;

  /**
   * @param wheelPositionsRad supplies the four drive wheel positions in radians, in module order
   *     (e.g. {@code drive::getWheelRadiusCharacterizationPositions})
   * @param wheelRadiusMeters wheel radius used to turn radians into meters (the drive's configured
   *     radius, so this count matches odometry)
   */
  public TreadUsageTracker(Supplier<double[]> wheelPositionsRad, double wheelRadiusMeters) {
    this.wheelPositionsRad = wheelPositionsRad;
    this.wheelRadiusMeters = wheelRadiusMeters;
    this.active = Constants.currentMode != Mode.REPLAY;
    this.store = Constants.currentMode == Mode.REAL ? new UsageStore(STORE_DIR) : null;
    this.saver =
        store == null
            ? null
            : Executors.newSingleThreadExecutor(
                runnable -> {
                  Thread thread = new Thread(runnable, "TreadUsageSaver");
                  thread.setDaemon(true);
                  thread.setPriority(Thread.MIN_PRIORITY);
                  return thread;
                });

    for (int i = 0; i < MODULE_NAMES.length; i++) {
      meters[i] = new UsageMeter("Tread/" + MODULE_NAMES[i], TREAD_SCHEDULE);
      lastPositionRad[i] = Double.NaN;
      logPrefix[i] = "Usage/Tread/" + MODULE_NAMES[i] + "/";
      dueAlerts[i] = new Alert(ALERT_GROUP, "", AlertType.kWarning);
    }
    lastSaveTime = Timer.getFPGATimestamp();

    if (!active) {
      return;
    }
    if (store != null) {
      loadSavedCounters();
    }
    setUpDashboard();
  }

  @Override
  public void periodic() {
    if (!active) {
      return;
    }
    countWheelTravel();

    boolean enabled = DriverStation.isEnabled();
    boolean justDisabled = wasEnabled && !enabled;
    boolean periodicSaveDue =
        enabled && Timer.getFPGATimestamp() - lastSaveTime >= SAVE_PERIOD_SECONDS;
    if (justDisabled || periodicSaveDue) {
      saveInBackground();
    }
    wasEnabled = enabled;

    updateAlerts();
    logState();
    if (++loopCount % DASHBOARD_SUMMARY_EVERY_N_LOOPS == 0) {
      publishDashboardSummary();
    }
  }

  // ---- Counting --------------------------------------------------------------------------------

  private void countWheelTravel() {
    double[] positions = wheelPositionsRad.get();
    if (positions == null || positions.length < MODULE_NAMES.length) {
      return;
    }
    for (int i = 0; i < MODULE_NAMES.length; i++) {
      double position = positions[i];
      if (!Double.isFinite(position)) {
        continue;
      }
      double last = lastPositionRad[i];
      lastPositionRad[i] = position;
      if (Double.isNaN(last)) {
        continue; // first reading after boot: nothing to compare against yet
      }
      double stepMeters = Math.abs(position - last) * wheelRadiusMeters;
      if (stepMeters <= MAX_STEP_METERS) {
        meters[i].add(stepMeters);
      }
    }
  }

  // ---- Saving and loading ----------------------------------------------------------------------

  private void loadSavedCounters() {
    UsageStore.Loaded loaded = store.load();
    if (loaded.source() == UsageStore.Source.NONE) {
      dataMissingAlert.set(true);
      return;
    }
    backupUsedAlert.set(loaded.source() == UsageStore.Source.BACKUP);
    boolean allRead = true;
    for (UsageMeter meter : meters) {
      allRead &= meter.readFrom(loaded.properties());
    }
    if (!allRead) {
      dataMissingAlert.setText(
          "Some tread usage data on the roboRIO was missing or unreadable; those modules started"
              + " from zero.");
      dataMissingAlert.set(true);
    }
  }

  /** Copies the counters now, then writes them to the roboRIO on the background thread. */
  private void saveInBackground() {
    lastSaveTime = Timer.getFPGATimestamp();
    if (store == null) {
      return;
    }
    Properties snapshot = new Properties();
    for (UsageMeter meter : meters) {
      meter.writeTo(snapshot);
    }
    saver.execute(
        () -> {
          try {
            store.save(snapshot);
            saveError = null;
          } catch (IOException | RuntimeException e) {
            saveError = String.valueOf(e.getMessage());
          }
        });
  }

  // ---- Dashboard actions -----------------------------------------------------------------------

  private void setUpDashboard() {
    moduleChooser.setDefaultOption("(select module)", NO_SELECTION);
    for (int i = 0; i < MODULE_NAMES.length; i++) {
      moduleChooser.addOption(MODULE_NAMES[i], i);
    }
    moduleChooser.addOption("All four", ALL_MODULES);

    gradeChooser.setDefaultOption("(select grade)", "");
    for (Grade grade : Grade.values()) {
      gradeChooser.addOption(grade.label(), grade.label());
    }

    SmartDashboard.putData(DASH + "Module", moduleChooser);
    SmartDashboard.putData(DASH + "Grade", gradeChooser);
    SmartDashboard.putString(DASH + "New Tread Label", "");
    SmartDashboard.putData(
        DASH + "Record Inspection",
        Commands.runOnce(this::recordInspection)
            .ignoringDisable(true)
            .withName("Record Inspection"));
    SmartDashboard.putData(
        DASH + "Record Replacement",
        Commands.runOnce(this::recordReplacement)
            .ignoringDisable(true)
            .withName("Record Replacement"));
    setStatus("Ready. Select a module, then record an inspection or a replacement.");
    publishDashboardSummary();
  }

  private void recordInspection() {
    int[] modules = selectedModulesOrNull();
    if (modules == null) {
      return;
    }
    Grade grade = Grade.fromLabel(gradeChooser.getSelected());
    if (grade == null) {
      setStatus("Not recorded: select a grade first.");
      return;
    }
    List<String> rows = new ArrayList<>();
    List<String> summary = new ArrayList<>();
    for (int i : modules) {
      UsageMeter meter = meters[i];
      meter.inspected(grade);
      rows.add(
          serviceRow(
              i,
              "inspected",
              grade.label(),
              meter.getSinceReplacement(),
              meter.getLabel(),
              meter.getLabel()));
      summary.add(
          String.format(
              "%s at %.2f km, next at %.2f km",
              MODULE_NAMES[i],
              meter.getSinceReplacement() / 1000.0,
              meter.getNextInspectionAt() / 1000.0));
    }
    finishAction(
        rows, "Inspection recorded (" + grade.label() + "): " + String.join("; ", summary));
  }

  private void recordReplacement() {
    int[] modules = selectedModulesOrNull();
    if (modules == null) {
      return;
    }
    String newLabel = SmartDashboard.getString(DASH + "New Tread Label", "").trim();
    if (newLabel.isEmpty()) {
      setStatus("Not recorded: type a label for the new tread first.");
      return;
    }
    List<String> rows = new ArrayList<>();
    List<String> summary = new ArrayList<>();
    for (int i : modules) {
      UsageMeter meter = meters[i];
      double reached = meter.getSinceReplacement();
      String oldLabel = meter.getLabel();
      meter.replaced(newLabel);
      rows.add(serviceRow(i, "replaced", "", reached, oldLabel, newLabel));
      summary.add(
          String.format(
              "%s (old %s reached %.2f km)", MODULE_NAMES[i], oldLabel, reached / 1000.0));
    }
    finishAction(
        rows,
        "Replacement recorded, new tread \"" + newLabel + "\": " + String.join("; ", summary));
  }

  /**
   * The modules picked on the dashboard, or null (with a status message) if nothing can be done.
   */
  private int[] selectedModulesOrNull() {
    if (!DriverStation.isDisabled()) {
      setStatus("Not recorded: disable the robot first.");
      return null;
    }
    Integer selection = moduleChooser.getSelected();
    if (selection == null || selection == NO_SELECTION) {
      setStatus("Not recorded: select a module first.");
      return null;
    }
    return selection == ALL_MODULES ? new int[] {0, 1, 2, 3} : new int[] {selection};
  }

  /** Logs the service rows, writes them to the service log, saves the counters, and confirms. */
  private void finishAction(List<String> rows, String status) {
    Logger.recordOutput("Usage/ServiceRecords", rows.toArray(new String[0]));
    if (store != null) {
      saver.execute(
          () -> {
            try {
              for (String row : rows) {
                store.appendServiceRecord(row);
              }
              serviceLogError = null;
            } catch (IOException | RuntimeException e) {
              serviceLogError = String.valueOf(e.getMessage());
            }
          });
    }
    saveInBackground();
    setStatus(status);
    publishDashboardSummary();
  }

  /**
   * One service-log row. {@code distance} is the "since replacement" distance at the moment of the
   * action (for a replacement, how far the old tread got).
   */
  private String serviceRow(
      int module, String action, String grade, double distance, String oldLabel, String newLabel) {
    UsageMeter meter = meters[module];
    String match =
        DriverStation.isFMSAttached()
            ? DriverStation.getMatchType() + " " + DriverStation.getMatchNumber()
            : "";
    return UsageStore.csvRow(
        timestamp(),
        RobotController.getSerialNumber(),
        DriverStation.getEventName(),
        match,
        MODULE_NAMES[module],
        action,
        grade,
        String.format("%.1f", distance),
        String.format("%.1f", meter.getLifetime()),
        String.format("%.1f", meter.getNextInspectionAt()),
        oldLabel,
        newLabel);
  }

  /** Local wall-clock time, which the Driver Station sets on the roboRIO when it connects. */
  private static String timestamp() {
    ZonedDateTime now = ZonedDateTime.now(TEAM_TIME_ZONE);
    if (now.getYear() < 2025) {
      return "clock-not-set"; // roboRIO has not been given the time by a Driver Station yet
    }
    return now.format(TIMESTAMP_FORMAT);
  }

  private void setStatus(String message) {
    SmartDashboard.putString(DASH + "Last Action", message);
    Logger.recordOutput("Usage/LastAction", message);
  }

  // ---- Alerts, logging, dashboard summary ------------------------------------------------------

  private void updateAlerts() {
    for (int i = 0; i < MODULE_NAMES.length; i++) {
      UsageMeter meter = meters[i];
      boolean due = meter.isInspectionDue();
      if (due) {
        String text = inspectionDueText(i);
        if (!text.equals(dueAlertText[i])) {
          dueAlerts[i].setText(text);
          dueAlertText[i] = text;
        }
      }
      dueAlerts[i].set(due);
    }

    String saveProblem = saveError;
    if (saveProblem != null) {
      saveFailedAlert.setText("Could not save tread usage data: " + saveProblem);
    }
    saveFailedAlert.set(saveProblem != null);

    String logProblem = serviceLogError;
    if (logProblem != null) {
      serviceLogFailedAlert.setText("Could not write the tread service log: " + logProblem);
    }
    serviceLogFailedAlert.set(logProblem != null);
  }

  private String inspectionDueText(int module) {
    UsageMeter meter = meters[module];
    String lastGrade =
        meter.getLastGrade() == null
            ? ""
            : String.format(
                ", last graded %s at %.2f km",
                meter.getLastGrade().label(), meter.getLastGradedAt() / 1000.0);
    return String.format(
        "%s tread: inspection due. %.2f km since replacement%s.",
        MODULE_NAMES[module], meter.getSinceReplacement() / 1000.0, lastGrade);
  }

  private void logState() {
    for (int i = 0; i < MODULE_NAMES.length; i++) {
      UsageMeter meter = meters[i];
      String prefix = logPrefix[i];
      Logger.recordOutput(prefix + "LifetimeMeters", meter.getLifetime());
      Logger.recordOutput(prefix + "SinceReplacementMeters", meter.getSinceReplacement());
      Logger.recordOutput(prefix + "NextInspectionAtMeters", meter.getNextInspectionAt());
      Logger.recordOutput(prefix + "InspectionDue", meter.isInspectionDue());
      Logger.recordOutput(prefix + "Label", meter.getLabel());
      Logger.recordOutput(
          prefix + "LastGrade", meter.getLastGrade() == null ? "" : meter.getLastGrade().label());
    }
  }

  /**
   * One readable line per module, e.g. "1.23 km since replacement; inspect at 2.00 km; tread-A".
   */
  private void publishDashboardSummary() {
    for (int i = 0; i < MODULE_NAMES.length; i++) {
      UsageMeter meter = meters[i];
      SmartDashboard.putString(
          DASH + MODULE_NAMES[i],
          String.format(
              "%.2f km since replacement; inspect at %.2f km; %s",
              meter.getSinceReplacement() / 1000.0,
              meter.getNextInspectionAt() / 1000.0,
              meter.getLabel()));
    }
  }
}
