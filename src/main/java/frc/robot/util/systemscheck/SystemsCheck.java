package frc.robot.util.systemscheck;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Pit systems check: commands small motions on each subsystem and compares sensor feedback with
 * what the physics says it should be. Runs only in Test mode, started from the dashboard.
 *
 * <p>Dashboard keys (all under SmartDashboard/SystemsCheck):
 *
 * <ul>
 *   <li>{@code Select}: which check to run ("All" or one subsystem).
 *   <li>{@code Run}: starts the selected check (added by RobotContainer).
 *   <li>{@code State}: IDLE, RUNNING, ABORTED, or the overall result.
 *   <li>{@code Results}: one line per measurement (string array, for logs and NT viewers).
 *   <li>{@code Prompt}, {@code Yes}, {@code No}: operator confirmations ("are the wheels...?").
 * </ul>
 *
 * <p>Every result also appears in the "SystemsCheck Alerts" group, which is what the pit-check
 * Elastic layout (deploy/pit-check.json) displays.
 *
 * <p>While each check runs, every subsystem it does not use is held by a "quiet" command (rollers
 * and shooter stopped, positioned mechanisms holding their last target), so default commands can't
 * move them and add noise to the measurement. Disabling the robot aborts the check.
 */
public class SystemsCheck {
  public static final String ALL = "All";

  private static final String KEY = "SystemsCheck";
  private static final String PROMPT_KEY = KEY + "/Prompt";
  private static final String YES_KEY = KEY + "/Yes";
  private static final String NO_KEY = KEY + "/No";
  private static final String ALERT_GROUP = "SystemsCheck Alerts";

  private final Map<Subsystem, Supplier<Command>> quietCommands;
  private final Map<String, Function<SystemsCheck, Command>> checks = new LinkedHashMap<>();
  private final LoggedDashboardChooser<String> chooser =
      new LoggedDashboardChooser<>(KEY + "/Select");

  private final List<CheckResult> results = new ArrayList<>();
  private final List<Alert> alerts = new ArrayList<>();
  private String state = "IDLE";

  /**
   * @param quietCommands for every subsystem on the robot, a factory for the command that holds it
   *     still while a check uses some other subsystem. The keys are also the Run command's
   *     requirements, so a check can never run alongside driver commands.
   */
  public SystemsCheck(Map<Subsystem, Supplier<Command>> quietCommands) {
    this.quietCommands = quietCommands;
    chooser.addDefaultOption(ALL, ALL);
    SmartDashboard.putString(PROMPT_KEY, "");
    SmartDashboard.putBoolean(YES_KEY, false);
    SmartDashboard.putBoolean(NO_KEY, false);
    publish();
  }

  /** Adds a check to the chooser. "All" runs every check in the order they were added. */
  public void addCheck(String name, Function<SystemsCheck, Command> builder) {
    checks.put(name, builder);
    chooser.addOption(name, name);
  }

  /**
   * The command behind the dashboard Run button. Does nothing unless the robot is enabled in Test
   * mode. Rejects stray driver/button-box commands instead of being interrupted by them.
   */
  public Command runCommand() {
    return Commands.defer(this::buildRun, quietCommands.keySet())
        .onlyIf(DriverStation::isTestEnabled)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Run systems check");
  }

  private Command buildRun() {
    String selection = chooser.get();
    List<Command> steps = new ArrayList<>();
    for (var entry : checks.entrySet()) {
      if (ALL.equals(selection) || entry.getKey().equals(selection)) {
        steps.add(withHolds(entry.getValue().apply(this)));
      }
    }
    if (steps.isEmpty()) {
      steps.add(
          Commands.runOnce(
              () ->
                  record(
                      CheckResult.skipped(
                          "Selection", "No check named \"" + selection + "\"; pick another"))));
    }

    return Commands.sequence(
            Commands.runOnce(() -> begin(selection)),
            Commands.sequence(steps.toArray(Command[]::new)))
        .finallyDo(this::end);
  }

  /** Runs {@code check} while holding every subsystem it doesn't use. */
  private Command withHolds(Command check) {
    Set<Subsystem> used = new HashSet<>(check.getRequirements());
    List<Command> holds = new ArrayList<>();
    for (var entry : quietCommands.entrySet()) {
      if (!used.contains(entry.getKey())) {
        holds.add(entry.getValue().get());
      }
    }
    return Commands.deadline(check, holds.toArray(Command[]::new));
  }

  /** Adds a result to the report, the log and the dashboard alert list. */
  public void record(CheckResult result) {
    results.add(result);
    String base = KEY + "/Values/" + result.name();
    Logger.recordOutput(base + "/Value", result.value());
    Logger.recordOutput(base + "/Status", result.status().name());
    Logger.recordOutput(base + "/Low", result.low());
    Logger.recordOutput(base + "/High", result.high());

    // Every result goes in the alert group: that's what Elastic shows (it can't display a
    // string[]). Errors = FAIL, warnings = WARN/SKIPPED, info = PASS/MEASURED.
    AlertType type =
        switch (result.status()) {
          case FAIL -> AlertType.kError;
          case WARN, SKIPPED -> AlertType.kWarning;
          default -> AlertType.kInfo;
        };
    Alert alert = new Alert(ALERT_GROUP, result.toLine(), type);
    alert.set(true);
    alerts.add(alert);
    publish();
  }

  /**
   * Asks the operator a yes/no question on the dashboard and records the answer. "No" is a FAIL
   * with {@code noAction}; no answer within the timeout is SKIPPED. Hands-off: the operator only
   * looks and clicks.
   */
  public Command confirm(String name, String question, String noAction, double timeoutSec) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  resetAnswers();
                  SmartDashboard.putString(PROMPT_KEY, question);
                  Logger.recordOutput(KEY + "/Prompt", question);
                }),
            Commands.waitUntil(() -> answeredYes() || answeredNo()).withTimeout(timeoutSec),
            Commands.runOnce(
                () -> {
                  boolean yes = answeredYes();
                  boolean no = answeredNo();
                  if (yes == no) { // neither, or both clicked
                    record(
                        CheckResult.skipped(
                            name, "No clear answer within " + (int) timeoutSec + " s; re-run"));
                  } else {
                    record(CheckResult.passFail(name, yes, noAction));
                  }
                }))
        .finallyDo(
            () -> {
              resetAnswers();
              SmartDashboard.putString(PROMPT_KEY, "");
              Logger.recordOutput(KEY + "/Prompt", "");
            });
  }

  /**
   * Applies an output every loop, waits {@code settleSec} for it to reach steady state, then
   * averages each signal over {@code sampleSec} and passes the means to {@code onDone}.
   */
  public static Command holdAndSample(
      Subsystem requirement,
      Runnable apply,
      double settleSec,
      double sampleSec,
      DoubleSupplier[] signals,
      Consumer<double[]> onDone) {
    Timer timer = new Timer();
    double[] sums = new double[signals.length];
    int[] count = {0};
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              Arrays.fill(sums, 0.0);
              count[0] = 0;
              timer.restart();
            }),
        Commands.run(
                () -> {
                  apply.run();
                  if (timer.get() >= settleSec) {
                    for (int i = 0; i < signals.length; i++) {
                      sums[i] += signals[i].getAsDouble();
                    }
                    count[0]++;
                  }
                },
                requirement)
            .until(() -> timer.get() >= settleSec + sampleSec && count[0] > 0),
        Commands.runOnce(
            () -> {
              double[] means = new double[sums.length];
              for (int i = 0; i < sums.length; i++) {
                means[i] = sums[i] / count[0];
              }
              onDone.accept(means);
            }));
  }

  private void begin(String selection) {
    for (Alert alert : alerts) {
      alert.close();
    }
    alerts.clear();
    results.clear();
    state = "RUNNING: " + selection;
    Logger.recordOutput(KEY + "/Selection", selection);
    Logger.recordOutput(KEY + "/StartBatteryVolts", RobotController.getBatteryVoltage());
    publish();
  }

  private void end(boolean interrupted) {
    state = interrupted ? "ABORTED" : overall();
    publish();
  }

  private String overall() {
    int worst = 0;
    for (CheckResult result : results) {
      worst = Math.max(worst, result.status().severity);
    }
    if (worst >= CheckStatus.FAIL.severity) {
      return "NOT READY: see failures";
    }
    if (worst >= CheckStatus.WARN.severity) {
      return "READY: see warnings";
    }
    if (worst >= CheckStatus.SKIPPED.severity) {
      return "INCOMPLETE: see skipped checks";
    }
    return "COMPLETE";
  }

  private void publish() {
    String[] lines = results.stream().map(CheckResult::toLine).toArray(String[]::new);
    SmartDashboard.putString(KEY + "/State", state);
    SmartDashboard.putStringArray(KEY + "/Results", lines);
    Logger.recordOutput(KEY + "/State", state);
    Logger.recordOutput(KEY + "/Results", lines);
  }

  private static boolean answeredYes() {
    return SmartDashboard.getBoolean(YES_KEY, false);
  }

  private static boolean answeredNo() {
    return SmartDashboard.getBoolean(NO_KEY, false);
  }

  private static void resetAnswers() {
    SmartDashboard.putBoolean(YES_KEY, false);
    SmartDashboard.putBoolean(NO_KEY, false);
  }
}
