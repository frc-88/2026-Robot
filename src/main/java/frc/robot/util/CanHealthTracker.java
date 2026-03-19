package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;
import org.littletonrobotics.junction.Logger;

/** Tracks CAN connection health for CTRE Phoenix devices and provides latching indicators. */
public final class CanHealthTracker {
  private CanHealthTracker() {}

  private static final String DASHBOARD_PREFIX = "CAN/";
  private static final String DISCONNECTED_LATCH_KEY = DASHBOARD_PREFIX + "DisconnectedDevices";

  // Last observed connection state; used to detect a transition from connected -> not connected.
  private static final Map<String, Boolean> lastConnected = new ConcurrentHashMap<>();

  // Keeps device names in first-disconnect order until cleared.
  private static final LinkedHashSet<String> disconnectedLatch = new LinkedHashSet<>();

  /**
   * Updates the tracker for a single device.
   *
   * <p>When a device transitions from connected to not connected, the device name is added to the
   * latching set.
   */
  public static void updateDevice(String deviceName, boolean connected) {
    Objects.requireNonNull(deviceName, "deviceName");

    // Publish per-device status.
    SmartDashboard.putBoolean(DASHBOARD_PREFIX + deviceName + "/Connected", connected);
    Logger.recordOutput("CAN/" + deviceName + "/Connected", connected);

    // Transition detection for latching.
    boolean hadConnection = lastConnected.getOrDefault(deviceName, connected);
    lastConnected.put(deviceName, connected);

    if (hadConnection && !connected) {
      disconnectedLatch.add(deviceName);
      publishDisconnectedLatch();
    }
  }

  /** Clears both per-device transition history and the disconnected latching string. */
  public static void clearLatch() {
    lastConnected.clear();
    disconnectedLatch.clear();
    publishDisconnectedLatch();
  }

  private static void publishDisconnectedLatch() {
    String latchString = disconnectedLatch.isEmpty() ? "" : String.join(",", disconnectedLatch);
    SmartDashboard.putString(DISCONNECTED_LATCH_KEY, latchString);
    Logger.recordOutput("CAN/DisconnectedDevices", latchString);
  }
}
