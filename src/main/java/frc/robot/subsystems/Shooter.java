package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.util.Util;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterMain =
      new TalonFX(Constants.SHOOTER_MAIN, CANBus.roboRIO()); // forward +
  private final TalonFX shooterFollower =
      new TalonFX(Constants.SHOOTER_FOLLOWER, CANBus.roboRIO()); // forward -
  private final CANcoder shooterCANcoder = new CANcoder(Constants.SHOOTER_CANCODER);
  private final DigitalInput feederBeamBreak = new DigitalInput(0);

  private final VelocityVoltage requestShooter = new VelocityVoltage(0.0);
  private final VoltageOut sysIdReq = new VoltageOut(0.0);

  private final DoublePreferenceConstant shootSpeed =
      new DoublePreferenceConstant("Shooter/ShootSpeed", 0.0);
  private final DoublePreferenceConstant increaseDuration =
      new DoublePreferenceConstant("Shooter/IncreaseDuration", 0.06);
  private final DoublePreferenceConstant increaseDelay =
      new DoublePreferenceConstant("Shooter/IncreaseDelay", 0.0);
  private final MotionMagicPIDPreferenceConstants shooterConfigConstants =
      new MotionMagicPIDPreferenceConstants("Shooter/ShooterMotors");

  private final Trigger feederBeamBreakTrigger = new Trigger(() -> isBeamBlocked());

  @SuppressWarnings("unused")
  private boolean boosted = false;
  // private final Trigger boostStarted = new Trigger(() -> boosted);
  private final Timer timeSinceBallLastSeen = new Timer();
  // private final Timer timeSinceBoostStarted = new Timer();

  // BPS
  private int ballsCount;
  private double earliestBallTime = 0;
  private double lastBallTime;

  @SuppressWarnings("unused")
  private double ballsPerSecond;

  private double targetVelocity = 0;

  // SysId
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(this::setVoltage, null, this));
  private DoubleSupplier m_targetSpeed;

  public Shooter(DoubleSupplier speed) {
    m_targetSpeed = speed;
    configureTalons();
    configureCANCoder();
    configureSmartDashboardButtons();
  }

  private void configureTalons() {
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    shooterConfig.Slot0.kP = shooterConfigConstants.getKP().getValue();
    shooterConfig.Slot0.kI = shooterConfigConstants.getKI().getValue();
    shooterConfig.Slot0.kD = shooterConfigConstants.getKD().getValue();
    shooterConfig.Slot0.kV = shooterConfigConstants.getKV().getValue();
    shooterConfig.Slot0.kS = shooterConfigConstants.getKS().getValue();

    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    shooterConfig.Feedback.FeedbackRemoteSensorID = Constants.SHOOTER_CANCODER;
    shooterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    shooterConfig.Feedback.SensorToMechanismRatio = 1.0;
    shooterConfig.Feedback.RotorToSensorRatio = Constants.SHOOTER_GEAR_RATIO;
    // TODO Run SysId to re-characterize with CANCoder on flywheel shaft

    shooterMain.getConfigurator().apply(shooterConfig);
    shooterFollower.setControl(new Follower(Constants.SHOOTER_MAIN, MotorAlignmentValue.Opposed));

    timeSinceBallLastSeen.reset();
    feederBeamBreakTrigger.onTrue(
        new InstantCommand(
            () -> {
              timeSinceBallLastSeen.reset();
              timeSinceBallLastSeen.start();
            }));
    feederBeamBreakTrigger.onTrue(new InstantCommand(() -> calculateBPS()));
    // boostStarted.onTrue(new InstantCommand(() -> {timeSinceBoostStarted.reset();
    // timeSinceBoostStarted.start();}));
    shooterMain.getVelocity().setUpdateFrequency(100);
    shooterMain.getMotorVoltage().setUpdateFrequency(500);
  }

  private void configureCANCoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    shooterCANcoder.getConfigurator().apply(config);
  }

  private void configureSmartDashboardButtons() {
    SmartDashboard.putData(
        "Shooter/SysId/Quasistatic Forward", sysIdQuasistatic(Direction.kForward));
    SmartDashboard.putData(
        "Shooter/SysId/Quasistatic Reverse", sysIdQuasistatic(Direction.kReverse));
    SmartDashboard.putData("Shooter/SysId/Dynamic Forward", sysIdDynamic(Direction.kForward));
    SmartDashboard.putData("Shooter/SysId/Dynamic Reverse", sysIdDynamic(Direction.kReverse));
  }

  private void setShooterSpeed(DoubleSupplier speed) {
    shooterMain.setControl(
        requestShooter
            .withVelocity(speed.getAsDouble())
            .withFeedForward(0.0)
            .withUpdateFreqHz(1000.0));
  }
  //   DoubleSupplier speed,
  //   DoubleSupplier FeedForwardIncrease,
  //   DoubleSupplier Delay,
  //   DoubleSupplier Duration) {
  // if (shooterMain.getVelocity().getValueAsDouble() >= (speed.getAsDouble())) { // normal
  //   shooterMain.setControl(
  //       requestShooter
  //           .withVelocity(speed.getAsDouble())
  //           .withFeedForward(0.0)
  //           .withUpdateFreqHz(1000.0));
  //   boosted = false;
  // } else if ((timeSinceBallLastSeen.get() > (Duration.getAsDouble() + Delay.getAsDouble()))
  //     || (timeSinceBallLastSeen.get() < Delay.getAsDouble())) {
  //   shooterMain.setControl(
  //       requestShooter
  //           .withVelocity(speed.getAsDouble())
  //           .withFeedForward(0.0)
  //           .withUpdateFreqHz(1000.0)); // normal or delay time
  //   boosted = false;
  // } else { // in boost duration
  //   // double boost = FeedForwardIncrease.getAsDouble() *
  //   //   ((increaseDuration.getValue() + increaseDelay.getValue() -
  //   // timeSinceBallLastSeen.get())/(2 * increaseDuration.getValue()));
  //   double boost =
  //       FeedForwardIncrease.getAsDouble()
  //           / 3
  //           * (speed.getAsDouble()
  //               - shooterMain
  //                   .getVelocity()
  //                   .getValueAsDouble()); // * Math.pow(speed.getAsDouble() -
  //   // shooterMain.getVelocity().getValueAsDouble(), 2.0)/(3.0);
  //   shooterMain.setControl(
  //       requestShooter
  //           .withVelocity(speed.getAsDouble())
  //           .withFeedForward(boost)
  //           .withUpdateFreqHz(1000.0));
  //   boosted = true;
  // } // this runs if ((timeSinceBallLastSeen.get() > increaseDelay.getValue()) &&
  // // (timeSinceBallLastSeen.get() < (increaseDuration.getValue() + increaseDelay.getValue()))

  private void setVoltage(Voltage volts) {
    shooterMain.setControl(sysIdReq.withOutput(volts));
  }

  private void setShooterSpeed() {
    setShooterSpeed(() -> targetVelocity);
  }

  private boolean atShooterSpeed() {
    return Math.abs(shooterMain.getVelocity().getValueAsDouble() - targetVelocity) < 10.0;
  }

  // This should eventually be moved to utils in base or something
  private void calculateBPS() {
    ballsCount = ballsCount + 1;
    double currentTime = Timer.getFPGATimestamp();
    if (earliestBallTime == 0) { // if it is not yet set
      earliestBallTime = currentTime;
    } else {
      lastBallTime = currentTime;
    }
    if (earliestBallTime > 0 && lastBallTime > 0) {
      ballsPerSecond = (ballsCount) / (lastBallTime - earliestBallTime);
    }
  }

  @AutoLogOutput
  private boolean isBeamBlocked() {
    return !feederBeamBreak.get();
  }

  @AutoLogOutput
  private double getVelocity() {
    return shooterMain.getVelocity().getValueAsDouble();
  }

  @AutoLogOutput
  private double getPosition() {
    return shooterMain.getPosition().getValueAsDouble();
  }

  @AutoLogOutput
  private double getVoltage() {
    return shooterMain.getMotorVoltage().getValueAsDouble();
  }

  @AutoLogOutput
  private double getCANcoderVelocity() {
    return shooterCANcoder.getVelocity().getValueAsDouble();
  }

  @AutoLogOutput
  private double getCANcoderPosition() {
    return shooterCANcoder.getPosition().getValueAsDouble();
  }

  public void periodic() {
    targetVelocity = m_targetSpeed.getAsDouble();

    // Lookup Table Building Override
    // targetVelocity = shootSpeed.getValue();

    if (Util.logif()) {
      SmartDashboard.putNumber("Shooter/TimeSinceBallLastSeen", timeSinceBallLastSeen.get());
      SmartDashboard.putNumber(
          "Shooter/BallsPerSecond",
          (Math.round((100.0 * ballsPerSecond)) / 100.0)); // round to hundredths
      // SmartDashboard.putNumber("Shooter/TimeSinceBoostStarted", timeSinceBoostStarted.get());
      SmartDashboard.putBoolean("Shooter/Boosted", boosted);
    }
  }

  public void resetBPS() {
    ballsCount = 0;
    earliestBallTime = 0;
    lastBallTime = 0;
  }

  public Command runShooter() {
    return new RunCommand(() -> setShooterSpeed(), this);
  }

  public Command stopShooter() {
    return new RunCommand(() -> shooterMain.stopMotor(), this);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public Command setVelocity(double velocity) {
    return new InstantCommand(() -> targetVelocity = velocity);
  }
}
