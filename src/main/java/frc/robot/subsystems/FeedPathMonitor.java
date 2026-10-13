package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FeedJamDetector;
import frc.robot.util.FeedJamDetector.Stage;
import frc.robot.util.FeedJamDetector.State;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Watches the ball path while shooting and logs whether it's flowing, empty or jammed.
 *
 * <p>Watch-only, like ShotCounter: it owns no motors, nothing requires it, and it takes no action
 * on a jam. It extends SubsystemBase only so periodic() runs each loop. Construct it after
 * ShotCounter so it sees this loop's shot.
 *
 * <p>Stages watched: the feeder and the floor spinner. The roof and funnel motors can be added as
 * more stages (one supplier pair each), at the cost of two more signal reads per loop.
 */
public class FeedPathMonitor extends SubsystemBase {
  private static final String[] STATE_NAMES = {"Idle", "Starting", "Flowing", "Empty", "Jammed"};
  private static final double SAME_MATCH_GAP_SEC = 10.0;

  // Thresholds from the 2026 logs (see jam-detection doc). Feeder free speed is 82-107 rps,
  // the floor spinner's ~91 rps.
  private static final Stage FEEDER = new Stage("Feeder", 55.0, 25.0);
  private static final Stage SPINNER = new Stage("Spinner", 20.0, 40.0);

  private final DoublePreferenceConstant p_starveSec =
      new DoublePreferenceConstant("FeedPath/StarveSec", 0.6);

  private final FeedJamDetector.Config config = new FeedJamDetector.Config();
  private final FeedJamDetector detector = new FeedJamDetector(config, FEEDER, SPINNER);

  private final BooleanSupplier shooterOn;
  private final BooleanSupplier feederGateOpen;
  private final BooleanSupplier shotThisLoop;
  private final DoubleSupplier[] velocity;
  private final DoubleSupplier[] current;
  private final double[] velocityNow;
  private final double[] currentNow;
  private final String[] boggedKeys;
  private final String[] jamCountKeys;

  // Per match
  private State lastState = State.IDLE;
  private int jamCount = 0;
  private int emptyCount = 0;
  private final int[] jamCountByStage;
  private double jamSec = 0.0;
  private double emptySec = 0.0;

  private double lastLoopTime = Double.NaN;
  private double lastAutoEndTime = Double.NEGATIVE_INFINITY;
  private boolean wasAuto = false;

  public FeedPathMonitor(
      BooleanSupplier shooterOn,
      BooleanSupplier feederGateOpen,
      BooleanSupplier shotThisLoop,
      DoubleSupplier feederVelocity,
      DoubleSupplier feederCurrent,
      DoubleSupplier spinnerVelocity,
      DoubleSupplier spinnerCurrent) {
    this.shooterOn = shooterOn;
    this.feederGateOpen = feederGateOpen;
    this.shotThisLoop = shotThisLoop;
    this.velocity = new DoubleSupplier[] {feederVelocity, spinnerVelocity};
    this.current = new DoubleSupplier[] {feederCurrent, spinnerCurrent};
    int n = detector.getStageCount();
    velocityNow = new double[n];
    currentNow = new double[n];
    jamCountByStage = new int[n];
    boggedKeys = new String[n];
    jamCountKeys = new String[n];
    for (int i = 0; i < n; i++) {
      boggedKeys[i] = "FeedPath/" + detector.getStageName(i) + "Bogged";
      jamCountKeys[i] = "FeedPath/" + detector.getStageName(i) + "Jams";
    }
  }

  /** Call from Robot.autonomousInit() and Robot.teleopInit(), same rule as ShotCounter. */
  public void onModeStart(boolean autonomous) {
    if (autonomous || Timer.getFPGATimestamp() - lastAutoEndTime > SAME_MATCH_GAP_SEC) {
      jamCount = 0;
      emptyCount = 0;
      jamSec = 0.0;
      emptySec = 0.0;
      for (int i = 0; i < jamCountByStage.length; i++) {
        jamCountByStage[i] = 0;
      }
    }
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    double dt = Double.isNaN(lastLoopTime) ? 0.0 : Math.min(now - lastLoopTime, 0.2);
    lastLoopTime = now;

    boolean enabled = DriverStation.isEnabled();
    boolean auto = DriverStation.isAutonomous();
    if (wasAuto && !(enabled && auto)) {
      lastAutoEndTime = now;
    }
    wasAuto = enabled && auto;

    config.starveSec = p_starveSec.getValue();

    boolean on = enabled && shooterOn.getAsBoolean();
    boolean gate = on && feederGateOpen.getAsBoolean();
    if (gate) { // only read the motors while feeding; the detector ignores them otherwise
      for (int i = 0; i < velocityNow.length; i++) {
        velocityNow[i] = velocity[i].getAsDouble();
        currentNow[i] = current[i].getAsDouble();
      }
    }
    State state =
        detector.update(now, on, gate, shotThisLoop.getAsBoolean(), velocityNow, currentNow);

    if (state == State.JAMMED) {
      jamSec += dt;
      if (lastState != State.JAMMED) {
        jamCount++;
        jamCountByStage[detector.getJammedStage()]++;
      }
    } else if (state == State.EMPTY) {
      emptySec += dt;
      if (lastState != State.EMPTY) {
        emptyCount++;
      }
    }
    lastState = state;

    Logger.recordOutput("FeedPath/State", STATE_NAMES[state.ordinal()]);
    Logger.recordOutput("FeedPath/JammedStage", detector.getJammedStageName());
    Logger.recordOutput("FeedPath/StarvedSeconds", detector.getStarvedSec());
    for (int i = 0; i < boggedKeys.length; i++) {
      Logger.recordOutput(boggedKeys[i], detector.isBogged(i));
      Logger.recordOutput(jamCountKeys[i], jamCountByStage[i]);
    }
    Logger.recordOutput("FeedPath/JamCount", jamCount);
    Logger.recordOutput("FeedPath/JamSeconds", jamSec);
    Logger.recordOutput("FeedPath/EmptyCount", emptyCount);
    Logger.recordOutput("FeedPath/EmptySeconds", emptySec);
  }

  public State getState() {
    return detector.getState();
  }
}
