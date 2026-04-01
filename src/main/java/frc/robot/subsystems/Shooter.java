package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// I like blueberries
// especially the tasty
// big round yellow ones

public class Shooter extends SubsystemBase {
  // motors & devices
  private final TalonFX shooterMain =
      new TalonFX(Constants.SHOOTER_MAIN, CANBus.roboRIO()); // forward +
  private final TalonFX shooterFollower =
      new TalonFX(Constants.SHOOTER_FOLLOWER, CANBus.roboRIO()); // forward -

  // output requests
  private final VelocityVoltage requestShooter = new VelocityVoltage(0.0);
  private final VoltageOut sysIdReq = new VoltageOut(0.0);

  // preferences
  private final DoublePreferenceConstant shootSpeed =
      new DoublePreferenceConstant("Shooter/ShootSpeed", 37.3);
  private final MotionMagicPIDPreferenceConstants shooterConfigConstants =
      new MotionMagicPIDPreferenceConstants(
          "Shooter/ShooterMotors", 0.0, 0.0, 0.0, 0.13985, 0.0, 0.0, 0.091582, 0.21515, 0.011146);

  // SysId routine
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second), // lower default ramp rate
              Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              (state) -> Logger.recordOutput("Shooter/SysIdTestState", state.toString())),
          new SysIdRoutine.Mechanism(this::setVoltage, null, this));

  private final DoubleSupplier m_targetSpeed;
  private double targetVelocity = 0;

  public Shooter(DoubleSupplier speed) {
    m_targetSpeed = speed;
    configureTalons();
    configureSmartDashboardButtons();
  }

  private void configureTalons() {
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    TalonFXConfiguration shooterFollowerConfig = new TalonFXConfiguration();

    shooterConfig.Slot0.kP = shooterConfigConstants.getKP().getValue();
    shooterConfig.Slot0.kI = shooterConfigConstants.getKI().getValue();
    shooterConfig.Slot0.kD = shooterConfigConstants.getKD().getValue();
    shooterConfig.Slot0.kV = shooterConfigConstants.getKV().getValue();
    shooterConfig.Slot0.kA = shooterConfigConstants.getKA().getValue();
    shooterConfig.Slot0.kS = shooterConfigConstants.getKS().getValue();
    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    shooterMain.getConfigurator().apply(shooterConfig);

    shooterFollower.getConfigurator().apply(shooterFollowerConfig);
    shooterFollower.setControl(new Follower(Constants.SHOOTER_MAIN, MotorAlignmentValue.Opposed));

    shooterMain.getVelocity().setUpdateFrequency(100);
    // the motorVoltage signal frequency is effectively the follower update rate
    shooterMain.getMotorVoltage().setUpdateFrequency(500);
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

  private void setVoltage(Voltage volts) {
    shooterMain.setControl(sysIdReq.withOutput(volts));
  }

  private void setShooterSpeed() {
    setShooterSpeed(() -> targetVelocity);
  }

  private boolean atShooterSpeed() {
    return Math.abs(shooterMain.getVelocity().getValueAsDouble() - targetVelocity) < 10.0;
  }

  @AutoLogOutput
  private boolean isHealthy() {
    return shooterMain.isConnected()
        && shooterMain.isAlive()
        && shooterFollower.isConnected()
        && shooterFollower.isAlive();
  }

  @AutoLogOutput
  private boolean isMainConnected() {
    return shooterMain.isConnected();
  }

  @AutoLogOutput
  private boolean isFollowerConnected() {
    return shooterFollower.isConnected();
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
  private double getMainVoltage() {
    return shooterMain.getMotorVoltage().getValueAsDouble();
  }

  @AutoLogOutput
  private double getFollowerVoltage() {
    return shooterFollower.getMotorVoltage().getValueAsDouble();
  }

  @AutoLogOutput
  private double getMainCurrent() {
    return shooterMain.getTorqueCurrent().getValueAsDouble();
  }

  @AutoLogOutput
  private double getFollowerCurrent() {
    return shooterFollower.getTorqueCurrent().getValueAsDouble();
  }

  public void periodic() {
    targetVelocity = m_targetSpeed.getAsDouble();

    // Lookup Table Building Override
    // targetVelocity = shootSpeed.getValue();
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
}
