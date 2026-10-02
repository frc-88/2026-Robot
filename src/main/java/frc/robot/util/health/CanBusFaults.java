package frc.robot.util.health;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Health faults for the roboRIO's built-in CAN bus (shooter, intake, hood, turret, feeder, hot
 * tub). The drivetrain's CANivore bus is separate and is not checked here.
 *
 * <p>These read the statistics the roboRIO already keeps about its CAN bus ({@link
 * RobotController#getCANStatus()}), the same numbers AdvantageKit logs under {@code
 * SystemStats/CANBus}. Reading them sends nothing on the bus.
 *
 * <ul>
 *   <li>{@code CAN/BusErrors} (warning): any CAN errors at all, or the roboRIO's transmit queue
 *       overflowed. A healthy bus shows none.
 *   <li>{@code CAN/BusErrorsSevere} (error): an error count reached {@link #SEVERE_ERROR_COUNT}, or
 *       the roboRIO's CAN controller went bus-off.
 *   <li>{@code CAN/HighUtilization} (warning): the bus stayed busier than {@link #HIGH_UTILIZATION}
 *       for {@link #HIGH_UTILIZATION_SECONDS}.
 * </ul>
 *
 * <p><b>Reading the error counts:</b> they are not totals. Each one rises with every error and
 * falls by 1 with every frame that gets through cleanly, so it shows recent trouble, not a match
 * total. Under the CAN standard a transmit error adds 8 and a receive error usually adds 1. At 128
 * the controller becomes "error-passive" and struggles to send and receive (Newton Q23 reached 128
 * and the shooter dropped off the bus). 96 is the "error warning" limit many CAN controllers use,
 * one step before that.
 */
public final class CanBusFaults {
  /** Either error count at or above this is an error: the bus is close to error-passive (128). */
  public static final int SEVERE_ERROR_COUNT = 96;

  /** Utilization (0 to 1) above which the bus is getting crowded. TODO: confirm from logs. */
  public static final double HIGH_UTILIZATION = 0.70;

  /** How long utilization must stay high; single ~0.5 s spikes are normal. TODO: from logs. */
  public static final double HIGH_UTILIZATION_SECONDS = 2.0;

  // Bus-off and transmit-full are running totals, so the checks look for an increase.
  // -1 = no reading yet (the first reading only sets the starting point).
  private int lastTxFullCount = -1;
  private int lastBusOffCount = -1;

  private CanBusFaults() {}

  /** Creates the CAN bus faults. Pass false off the real robot (simulation has no CAN bus). */
  public static void create(boolean real) {
    CanBusFaults checks = new CanBusFaults();

    new Fault(
        "CAN/BusErrors",
        "roboRIO CAN bus errors detected. Check CAN wiring, connectors, and bus load.",
        AlertType.kWarning,
        () -> real && checks.hasErrors());

    new Fault(
        "CAN/BusErrorsSevere",
        "roboRIO CAN bus near failure (error count "
            + SEVERE_ERROR_COUNT
            + "+ or bus-off). Devices may drop out.",
        AlertType.kError,
        () -> real && checks.hasSevereErrors());

    new Fault(
        "CAN/HighUtilization",
        "roboRIO CAN bus busier than "
            + Math.round(HIGH_UTILIZATION * 100)
            + "%. Check for added devices or raised signal rates.",
        AlertType.kWarning,
        () -> real && RobotController.getCANStatus().percentBusUtilization > HIGH_UTILIZATION,
        HIGH_UTILIZATION_SECONDS);
  }

  private boolean hasErrors() {
    CANStatus status = RobotController.getCANStatus();
    boolean txFullIncreased = lastTxFullCount >= 0 && status.txFullCount > lastTxFullCount;
    lastTxFullCount = status.txFullCount;
    return status.receiveErrorCount > 0 || status.transmitErrorCount > 0 || txFullIncreased;
  }

  private boolean hasSevereErrors() {
    CANStatus status = RobotController.getCANStatus();
    boolean busOffIncreased = lastBusOffCount >= 0 && status.busOffCount > lastBusOffCount;
    lastBusOffCount = status.busOffCount;
    return status.receiveErrorCount >= SEVERE_ERROR_COUNT
        || status.transmitErrorCount >= SEVERE_ERROR_COUNT
        || busOffIncreased;
  }
}
