package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
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
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// so much fuel goes by
// suddenly I realize
// saw that one before

public class Feeder extends SubsystemBase {
  // motors & devices
  private final TalonFX m_feeder = new TalonFX(Constants.FEEDER_MAIN, CANBus.roboRIO());

  // output requests
  private final VelocityVoltage m_request = new VelocityVoltage(0.0);
  private final VoltageOut m_voltReq = new VoltageOut(0.0);
  private final DutyCycleOut antiJamRequest = new DutyCycleOut(0.0);

  // preferences
  private final DoublePreferenceConstant p_feedSpeed =
      new DoublePreferenceConstant("Feeder/FeedSpeed", 105.0);
  private final MotionMagicPIDPreferenceConstants p_feederConfigConstants =
      new MotionMagicPIDPreferenceConstants(
          "Feeder/MotorPID", 0., 0., 0., 0., 0., 0., 0.0988, 0., 0.);

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second), // lower default ramp rate to 0.5 V/s
              Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> Logger.recordOutput("Feeder/SysIdTestState", state.toString())),
          new SysIdRoutine.Mechanism(this::setVoltage, null, this));

  private BooleanSupplier m_turretOnTarget;

  public Feeder(BooleanSupplier turretOnTarget) {
    m_turretOnTarget = turretOnTarget;
    configureTalons();

    SmartDashboard.putData("Feeder/RunFeeder", runFeeder());
    SmartDashboard.putData("Feeder/StopFeeder", stopFeeder());
    SmartDashboard.putData(
        "Feeder/SysId/Quasistatic Forward", sysIdQuasistatic(Direction.kForward));
    SmartDashboard.putData(
        "Feeder/SysId/Quasistatic Reverse", sysIdQuasistatic(Direction.kReverse));
    SmartDashboard.putData("Feeder/SysId/Dynamic Forward", sysIdDynamic(Direction.kForward));
    SmartDashboard.putData("Feeder/SysId/Dynamic Reverse", sysIdDynamic(Direction.kReverse));
  }

  private void configureTalons() {
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();

    feederConfig.Slot0.kP = p_feederConfigConstants.getKP().getValue();
    feederConfig.Slot0.kI = p_feederConfigConstants.getKI().getValue();
    feederConfig.Slot0.kD = p_feederConfigConstants.getKD().getValue();
    feederConfig.Slot0.kV = p_feederConfigConstants.getKV().getValue();
    feederConfig.Slot0.kS = p_feederConfigConstants.getKS().getValue();
    feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_feeder.getConfigurator().apply(feederConfig);

    m_feeder.getVelocity().setUpdateFrequency(100);
  }

  @AutoLogOutput
  private Voltage getVoltage() {
    return m_feeder.getMotorVoltage().getValue();
  }

  @AutoLogOutput
  private Current getCurrent() {
    return m_feeder.getTorqueCurrent().getValue();
  }

  @AutoLogOutput
  private AngularVelocity getVelocity() {
    return m_feeder.getVelocity().getValue();
  }

  @AutoLogOutput
  private Angle getPosition() {
    return m_feeder.getPosition().getValue();
  }

  private void setVoltage(Voltage volts) {
    m_feeder.setControl(m_voltReq.withOutput(volts));
  }

  private void setFeederSpeed(DoubleSupplier speed) {
    m_feeder.setControl(m_request.withVelocity(speed.getAsDouble()));
  }

  private void stopFeederMotors() {
    m_feeder.stopMotor();
  }

  private void antiJam() {
    m_feeder.setControl(antiJamRequest.withOutput(-1.0));
  }

  public void periodic() {}

  public Command runFeeder() {
    return new RunCommand(
        () -> setFeederSpeed(() -> m_turretOnTarget.getAsBoolean() ? p_feedSpeed.getValue() : 0.0),
        this);
  }

  public Command antiJamFeeder() {
    return new RunCommand(() -> antiJam(), this);
  }

  public Command stopFeeder() {
    return new RunCommand(() -> stopFeederMotors(), this);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
