package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

// yellow ball party
// conga line around and up
// rainbow to the hub

public class HotTub extends SubsystemBase {
  // motors & devices
  private final TalonFX m_spinner = new TalonFX(Constants.SPINNER_MAIN, CANBus.roboRIO());

  // output requests
  private final VelocityDutyCycle m_request = new VelocityDutyCycle(0.0);
  private final VoltageOut m_voltReq = new VoltageOut(0.0);
  private final DutyCycleOut antiJamRequest = new DutyCycleOut(0.0);

  // preferences
  private final DoublePreferenceConstant p_spinnerSpeed =
      new DoublePreferenceConstant("Spinner/SpinnerSpeed", 65.0);
  private final MotionMagicPIDPreferenceConstants p_spinnerConfigConstants =
      new MotionMagicPIDPreferenceConstants(
          "Spinner/SpinnerMotors", 0., 0., 0., 0.17547, 0., 0., 0.11504, 0.18565, 0.01426);

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second), // lower default ramp rate to 0.5 V/s
              Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> Logger.recordOutput("Spinner/SysIdTestState", state.toString())),
          new SysIdRoutine.Mechanism(this::setVoltage, null, this));

  private final BooleanSupplier m_turretOnTarget;

  private SlewRateLimiter spinnerLimiter = new SlewRateLimiter(p_spinnerSpeed.getValue());

  public HotTub(BooleanSupplier turretOnTarget) {
    m_turretOnTarget = turretOnTarget;

    configureTalons();
    SmartDashboard.putData("Spinner/RunSpinner", runSpinner());
    SmartDashboard.putData("Spinner/StopSpinner", stopSpinner());
    SmartDashboard.putData(
        "Spinner/SysId/Quasistatic Forward", sysIdQuasistatic(Direction.kForward));
    SmartDashboard.putData(
        "Spinner/SysId/Quasistatic Reverse", sysIdQuasistatic(Direction.kReverse));
    SmartDashboard.putData("Spinner/SysId/Dynamic Forward", sysIdDynamic(Direction.kForward));
    SmartDashboard.putData("Spinner/SysId/Dynamic Reverse", sysIdDynamic(Direction.kReverse));
  }

  private void configureTalons() {
    TalonFXConfiguration spinnerConfig = new TalonFXConfiguration();
    spinnerConfig.Slot0.kP = p_spinnerConfigConstants.getKP().getValue();
    spinnerConfig.Slot0.kI = p_spinnerConfigConstants.getKI().getValue();
    spinnerConfig.Slot0.kD = p_spinnerConfigConstants.getKD().getValue();
    spinnerConfig.Slot0.kV = p_spinnerConfigConstants.getKV().getValue();
    spinnerConfig.Slot0.kS = p_spinnerConfigConstants.getKS().getValue();
    spinnerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    spinnerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    spinnerConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    m_spinner.getConfigurator().apply(spinnerConfig);
  }

  @AutoLogOutput
  public boolean isHealthy() {
    return m_spinner.isConnected() && m_spinner.isAlive();
  }

  @AutoLogOutput
  private Voltage getVoltage() {
    return m_spinner.getMotorVoltage().getValue();
  }

  @AutoLogOutput
  private Current getCurrent() {
    return m_spinner.getTorqueCurrent().getValue();
  }

  @AutoLogOutput
  private AngularVelocity getVelocity() {
    return m_spinner.getVelocity().getValue();
  }

  @AutoLogOutput
  private Angle getPosition() {
    return m_spinner.getPosition().getValue();
  }

  private void setVoltage(Voltage volts) {
    m_spinner.setControl(m_voltReq.withOutput(volts));
  }

  private void setSpinnerSpeed(DoubleSupplier speed) {
    m_spinner.setControl(m_request.withVelocity(spinnerLimiter.calculate(speed.getAsDouble())));
  }

  private void stopSpinnerMotors() {
    m_spinner.stopMotor();
  }

  private void antiJam() {
    m_spinner.setControl(antiJamRequest.withOutput(-1.0));
  }

  public void periodic() {}

  public Command runSpinner() {
    return new RunCommand(
        () ->
            setSpinnerSpeed(
                () -> m_turretOnTarget.getAsBoolean() ? p_spinnerSpeed.getValue() : 0.0),
        this);
  }

  public Command antiJamSpinner() {
    return new RunCommand(() -> antiJam(), this);
  }

  public Command stopSpinner() {
    return new RunCommand(() -> setSpinnerSpeed(() -> 0.0), this);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
