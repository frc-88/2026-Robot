package frc.robot.util.health;

import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.function.BooleanSupplier;

/**
 * Layer 1 of the health system: one detector watching one concern.
 *
 * <p>A Fault is a condition ("this is wrong right now") plus a name and a WPILib {@link AlertType}:
 * {@link AlertType#kError} for problems worth stopping the between-match routine to investigate,
 * {@link AlertType#kWarning} for real problems that can wait until there's time. It knows nothing
 * about lights. Creating one registers it with the {@link HealthMonitor}, which polls it every
 * loop, so a subsystem only has to <i>declare</i> its faults once in its constructor; it never has
 * to call anything from {@code periodic()}.
 *
 * <p>Each Fault owns a WPILib {@link Alert} with the same text, so an active fault appears in the
 * normal Alerts widget and in the log under {@code /RealOutputs/Alerts} exactly like any other
 * Alert. On top of that the Fault tracks:
 *
 * <ul>
 *   <li><b>active</b> — the condition is true right now (after any debounce).
 *   <li><b>latched</b> — the fault was active at some point while enabled since the last reset.
 *       This is what the end-of-match display uses, so a fault that appeared and cleared mid-match
 *       is still reported.
 *   <li><b>occurrences</b> — how many separate times it went active while enabled since the last
 *       reset (a device that dropped off three times vs. once).
 * </ul>
 *
 * <p>Faults are observers only. Reading the condition must never command hardware.
 */
public class Fault {
  private final String key;
  private final String logKey;
  private final String message;
  private final AlertType type;
  private final BooleanSupplier condition;
  private final Debouncer debouncer; // null when no debounce is requested
  private final Alert alert;

  private boolean active = false;
  private boolean latched = false;
  private boolean countedThisEpisode = false;
  private int occurrences = 0;

  /**
   * Creates and registers a fault with no extra debounce (the condition is used as-is).
   *
   * @param key short unique log key, e.g. {@code "Shooter/Follower"}. Logged under {@code
   *     Health/Faults/<key>}.
   * @param message text shown in the Alerts widget while active
   * @param type {@link AlertType#kError} or {@link AlertType#kWarning}
   * @param condition returns true while the fault is present. Polled once per loop.
   */
  public Fault(String key, String message, AlertType type, BooleanSupplier condition) {
    this(key, message, type, condition, 0.0);
  }

  /**
   * Creates and registers a fault whose condition must hold continuously for {@code
   * debounceSeconds} before it counts as active. Use this for noisy "sustained" conditions; use
   * zero for transients that should be caught (and latched) on a single sample.
   */
  public Fault(
      String key,
      String message,
      AlertType type,
      BooleanSupplier condition,
      double debounceSeconds) {
    this.key = key;
    this.logKey = "Health/Faults/" + key;
    this.message = message;
    if (type == AlertType.kInfo) {
      // kInfo is for status messages, not faults. Treat it as a warning rather than crash at boot.
      DriverStation.reportWarning(
          "Fault '" + key + "' uses kInfo; faults must be kError or kWarning. Using kWarning.",
          false);
      type = AlertType.kWarning;
    }
    this.type = type;
    this.condition = condition;
    this.debouncer =
        debounceSeconds > 0.0 ? new Debouncer(debounceSeconds, DebounceType.kRising) : null;
    this.alert = new Alert(message, type);
    HealthMonitor.getInstance().register(this);
  }

  /**
   * Rung 1 (presence) fault for a CTRE Phoenix 6 device: TalonFX, TalonFXS, CANcoder, Pigeon2, etc.
   * Always an error ({@link AlertType#kError}).
   *
   * <p>Uses {@link ParentDevice#isConnected()}, which reports disconnected once the device's
   * Version frame has not been seen for 0.5 s — that latency window is already the debounce, so no
   * extra debounce is added. Reading it only checks the RIO-side signal cache; it puts nothing on
   * the CAN bus.
   *
   * <p>Only evaluated on a real robot. In sim and log replay these hardware objects have no device
   * behind them and would always read disconnected.
   *
   * @param key log key, e.g. {@code "Shooter/Follower"}
   * @param description human-readable device name, e.g. {@code "Shooter follower motor"}
   * @param device the device object the subsystem already owns
   */
  public static Fault disconnected(String key, String description, ParentDevice device) {
    return new Fault(
        key,
        description + " disconnected (CAN ID " + device.getDeviceID() + ").",
        AlertType.kError,
        () -> Constants.currentMode == Mode.REAL && !device.isConnected());
  }

  /**
   * Called by {@link HealthMonitor} once per loop.
   *
   * @param evaluate false during the startup grace period; the condition is not polled and the
   *     fault reads inactive
   * @param latchEnabled true while the robot is enabled; only then do activations latch and count
   */
  void update(boolean evaluate, boolean latchEnabled) {
    boolean raw = evaluate && condition.getAsBoolean();
    active = debouncer == null ? raw : debouncer.calculate(raw);
    alert.set(active);

    if (!active) {
      countedThisEpisode = false;
    } else if (latchEnabled && !countedThisEpisode) {
      // New episode (or a fault already present when the robot enabled): remember it.
      latched = true;
      occurrences++;
      countedThisEpisode = true;
    }
  }

  /** Clears the latch and occurrence count. Called by the monitor at the start of a new scope. */
  void resetLatch() {
    latched = false;
    occurrences = 0;
    // Let a fault that is active right now be counted again once latching resumes.
    countedThisEpisode = false;
  }

  /** Full AdvantageKit log key, precomputed to avoid building a string every loop. */
  String getLogKey() {
    return logKey;
  }

  public String getKey() {
    return key;
  }

  public String getMessage() {
    return message;
  }

  public AlertType getType() {
    return type;
  }

  /** True if the fault is present right now. */
  public boolean isActive() {
    return active;
  }

  /** True if the fault has been active while enabled since the last reset. */
  public boolean hasLatched() {
    return latched;
  }

  /** Number of separate activations while enabled since the last reset. */
  public int getOccurrences() {
    return occurrences;
  }
}
