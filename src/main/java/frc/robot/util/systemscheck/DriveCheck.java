package frc.robot.util.systemscheck;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Drive-module check, robot on blocks. MEASUREMENT ONLY: logs kS, kV and current per module with no
 * pass/fail limits until baselines are collected (see the design doc's tuning section).
 *
 * <ol>
 *   <li>Hold each drive motor at {@link #LOW_VOLTS}, then {@link #HIGH_VOLTS}, open loop, with the
 *       modules steered to 0 degrees. Once settled, average speed, applied voltage and stator
 *       current.
 *   <li>Fit kS and kV per module from the two points; log each module against its siblings.
 *   <li>Ask the operator to confirm all four wheels point and spin the same way. This is the only
 *       pass/fail step: a drive motor's own encoder can't see its own inversion (inverting the
 *       motor inverts its sensor too), and a steer offset can be wrong while every encoder agrees
 *       with itself.
 * </ol>
 *
 * <p>Uses voltage control even though the drive normally runs TorqueCurrentFOC: a fixed torque
 * current on a free-spinning wheel just accelerates it to top speed, which tells us nothing.
 *
 * <p>The wheels spin on blocks, so odometry thinks the robot moved. The pose is saved first and
 * restored at the end (including on abort).
 */
public final class DriveCheck {
  private DriveCheck() {}

  private static final String[] NAMES = {"FL", "FR", "BL", "BR"};

  /** Far enough apart that static friction noise at the low point doesn't dominate the slope. */
  private static final double LOW_VOLTS = 2.0;

  private static final double HIGH_VOLTS = 5.0;
  private static final double SETTLE_SEC = 1.0; // includes steering to 0 deg at the start
  private static final double SAMPLE_SEC = 1.0;
  private static final double CONFIRM_TIMEOUT_SEC = 20.0;

  // Signal layout per module in the sampled arrays.
  private static final int SPEED = 0;
  private static final int VOLTS = 1;
  private static final int AMPS = 2;
  private static final int PER_MODULE = 3;

  public static Command build(Drive drive, SystemsCheck check) {
    DoubleSupplier[] signals = new DoubleSupplier[4 * PER_MODULE];
    for (int i = 0; i < 4; i++) {
      final int module = i;
      signals[i * PER_MODULE + SPEED] = () -> drive.getModuleVelocityMetersPerSec(module);
      signals[i * PER_MODULE + VOLTS] = () -> drive.getModuleDriveAppliedVolts(module);
      signals[i * PER_MODULE + AMPS] = () -> drive.getModuleDriveStatorCurrentAmps(module);
    }

    double[][] low = new double[1][];
    double[][] high = new double[1][];
    boolean[] connectedAtStart = new boolean[4];
    Pose2d[] savedPose = new Pose2d[1];

    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  savedPose[0] = drive.getPose();
                  for (int i = 0; i < 4; i++) {
                    connectedAtStart[i] = drive.isModuleConnected(i);
                  }
                },
                drive),
            SystemsCheck.holdAndSample(
                drive,
                () -> drive.runDriveVoltage(LOW_VOLTS),
                SETTLE_SEC,
                SAMPLE_SEC,
                signals,
                means -> low[0] = means),
            SystemsCheck.holdAndSample(
                drive,
                () -> drive.runDriveVoltage(HIGH_VOLTS),
                SETTLE_SEC,
                SAMPLE_SEC,
                signals,
                means -> high[0] = means),
            Commands.runOnce(() -> evaluate(drive, check, connectedAtStart, low[0], high[0])),
            Commands.deadline(
                check.confirm(
                    "Drive wheels aligned",
                    "Drive check: are all four wheels pointing the same way AND spinning the same"
                        + " direction?",
                    "Check the odd module's steer offset (pointing) or drive motor inversion"
                        + " (spinning)",
                    CONFIRM_TIMEOUT_SEC),
                Commands.run(() -> drive.runDriveVoltage(LOW_VOLTS), drive)))
        .finallyDo(
            () -> {
              drive.stop();
              if (savedPose[0] != null) {
                drive.setPose(savedPose[0]);
              }
            })
        .withName("Drive check");
  }

  private static void evaluate(
      Drive drive, SystemsCheck check, boolean[] connectedAtStart, double[] low, double[] high) {
    boolean[] ok = new boolean[4];
    double[] kS = new double[4];
    double[] kV = new double[4];
    double[] amps = new double[4];

    // Reference only: kV if the motor matched its datasheet free speed with no friction.
    double theoreticalKv = 12.0 / TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    Logger.recordOutput("SystemsCheck/Drive/TheoreticalKv", theoreticalKv);

    for (int i = 0; i < 4; i++) {
      String name = NAMES[i] + " drive";
      String base = "SystemsCheck/Drive/" + NAMES[i];
      ok[i] = connectedAtStart[i] && drive.isModuleConnected(i);
      if (!ok[i]) {
        check.record(
            CheckResult.skipped(
                name, "Module disconnected; check its CAN and power connectors (health alerts)"));
        kS[i] = kV[i] = amps[i] = Double.NaN;
        continue;
      }

      int at = i * PER_MODULE;
      // Fit against measured applied volts, not commanded: catches supply limiting or sag.
      CheckMath.Fit fit =
          CheckMath.twoPointFit(
              low[at + VOLTS], low[at + SPEED], high[at + VOLTS], high[at + SPEED]);
      kS[i] = fit.kS();
      kV[i] = fit.kV();
      amps[i] = high[at + AMPS];

      Logger.recordOutput(base + "/LowSpeedMps", low[at + SPEED]);
      Logger.recordOutput(base + "/LowVolts", low[at + VOLTS]);
      Logger.recordOutput(base + "/LowAmps", low[at + AMPS]);
      Logger.recordOutput(base + "/HighSpeedMps", high[at + SPEED]);
      Logger.recordOutput(base + "/HighVolts", high[at + VOLTS]);
      Logger.recordOutput(base + "/HighAmps", high[at + AMPS]);

      check.record(CheckResult.measured(name + " kS", kS[i], "V"));
      check.record(CheckResult.measured(name + " kV", kV[i], "V/(m/s)"));
      check.record(CheckResult.measured(name + " current @ high step", amps[i], "A"));
    }

    // Sibling comparison: the eventual pass/fail metric. Logged now so limits can be set from it.
    for (int i = 0; i < 4; i++) {
      if (!ok[i]) {
        continue;
      }
      String base = "SystemsCheck/Drive/" + NAMES[i];
      Logger.recordOutput(base + "/kSVsSiblings", CheckMath.ratioToOthers(kS, ok, i));
      Logger.recordOutput(base + "/kVVsSiblings", CheckMath.ratioToOthers(kV, ok, i));
      Logger.recordOutput(base + "/AmpsVsSiblings", CheckMath.ratioToOthers(amps, ok, i));
    }
  }
}
