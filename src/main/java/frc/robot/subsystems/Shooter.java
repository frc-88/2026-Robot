package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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
import frc.robot.Constants;
import frc.robot.util.Util;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterMain =
      new TalonFX(Constants.SHOOTER_MAIN, CANBus.roboRIO()); // forward +
  private final TalonFX shooterFollower =
      new TalonFX(Constants.SHOOTER_FOLLOWER, CANBus.roboRIO()); // forward -
  private DigitalInput feederBeamBreak = new DigitalInput(0);
  private Trigger feederBeamBreakTrigger = new Trigger(() -> isBeamBlocked());

  @SuppressWarnings("unused")
  private boolean boosted = false;
  // private Trigger boostStarted = new Trigger(() -> boosted);
  private Timer timeSinceBallLastSeen = new Timer();
  // private Timer timeSinceBoostStarted = new Timer();

  // BPS
  private int ballsCount;
  private double earliestBallTime = 0;
  private double lastBallTime;

  @SuppressWarnings("unused")
  private double ballsPerSecond;

  // Shooter
  private double targetVelocity = 0;
  private VelocityVoltage requestShooter = new VelocityVoltage(0.0);
  public DoublePreferenceConstant shootSpeed =
      new DoublePreferenceConstant("Shooter/ShootSpeed", 0.0);
  public DoublePreferenceConstant increaseDuration =
      new DoublePreferenceConstant("Shooter/IncreaseDuration", 0.06);
  public DoublePreferenceConstant increaseDelay =
      new DoublePreferenceConstant("Shooter/IncreaseDelay", 0.0);
  public DoublePreferenceConstant shootVoltage =
      new DoublePreferenceConstant("Shooter/ShootVoltage", 0.0);
  private final DoublePreferenceConstant increaseFeedForward =
      new DoublePreferenceConstant("Shooter/IncreaseFeedForward", 0.0);
  private final MotionMagicPIDPreferenceConstants shooterConfigConstants =
      new MotionMagicPIDPreferenceConstants("Shooter/ShooterMotors");

  // Hood
  private double targetAngle = 0;
  private MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0.0);
  private MotionMagicPIDPreferenceConstants hoodConfigConstants =
      new MotionMagicPIDPreferenceConstants("HoodMotor");

  // SysId
  private final VoltageOut m_voltReq = new VoltageOut(0.0);
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
    configureSmartDashboardButtons();
  }

  private void configureTalons() {
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    shooterConfig.Slot0.kP = shooterConfigConstants.getKP().getValue();
    shooterConfig.Slot0.kI = shooterConfigConstants.getKI().getValue();
    shooterConfig.Slot0.kD = shooterConfigConstants.getKD().getValue();
    shooterConfig.Slot0.kV = shooterConfigConstants.getKV().getValue();
    shooterConfig.Slot0.kS = shooterConfigConstants.getKS().getValue();

    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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
  }

  private void configureSmartDashboardButtons() {
    // SmartDashboard.putData("Shooter/RunShooter", runShooter());
    // SmartDashboard.putData("Shooter/StopShooter", stopShooter());
    // SmartDashboard.putData(
    //     "Shooter/SysId/Quasistatic Forward", sysIdQuasistatic(Direction.kForward));
    // SmartDashboard.putData(
    //     "Shooter/SysId/Quasistatic Reverse", sysIdQuasistatic(Direction.kReverse));
    // SmartDashboard.putData("Shooter/SysId/Dynamic Forward", sysIdDynamic(Direction.kForward));
    // SmartDashboard.putData("Shooter/SysId/Dynamic Reverse", sysIdDynamic(Direction.kReverse));
  }

  public void periodic() {
    targetVelocity = m_targetSpeed.getAsDouble();
    if (Util.logif()) {
      SmartDashboard.putNumber(
          "Shooter/ShooterVelocity", shooterMain.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber(
          "Shooter/ShooterVoltage", shooterMain.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber(
          "Shooter/ShooterMTCurrent", shooterMain.getTorqueCurrent().getValueAsDouble());
      SmartDashboard.putNumber(
          "Shooter/ShooterMSCurrent", shooterMain.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putNumber(
          "Shooter/ShooterFTCurrent", shooterFollower.getTorqueCurrent().getValueAsDouble());
      SmartDashboard.putNumber(
          "Shooter/ShooterFSCurrent", shooterFollower.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/TimeSinceBallLastSeen", timeSinceBallLastSeen.get());
      SmartDashboard.putNumber(
          "Shooter/BallsPerSecond",
          (Math.round((100.0 * ballsPerSecond)) / 100.0)); // round to hundredths
      // SmartDashboard.putNumber("Shooter/TimeSinceBoostStarted", timeSinceBoostStarted.get());
      SmartDashboard.putBoolean("Shooter/IsBeamBlocked", isBeamBlocked());
      SmartDashboard.putBoolean("Shooter/Boosted", boosted);
    }
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
    shooterMain.setControl(m_voltReq.withOutput(volts));
  }

  private void setShooterSpeed() {
    setShooterSpeed(() -> targetVelocity);
  }

  public boolean atShooterSpeed() {
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

  public void resetBPS() {
    ballsCount = 0;
    earliestBallTime = 0;
    lastBallTime = 0;
  }

  private boolean isBeamBlocked() {
    return !feederBeamBreak.get();
  }

  public Command runShooter() {
    return new RunCommand(
        () -> setShooterSpeed(),
        // () -> increaseFeedForward.getValue(),
        // () -> increaseDelay.getValue(),
        // () -> increaseDuration.getValue()),
        this);
  }

  // public Command runShooterVoltage() {
  //   return new RunCommand(() -> setVoltage(shootVoltage.getValue()));
  // }

  public Command stopShooter() {
    return new RunCommand(() -> shooterMain.stopMotor(), this);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public InstantCommand setVelocity(double velocity) {
    return new InstantCommand(() -> targetVelocity = velocity, this);
  }

  public InstantCommand setAngle(double angle) {
    return new InstantCommand(() -> targetAngle = angle);
  }
}
