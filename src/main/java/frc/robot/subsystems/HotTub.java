package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class HotTub extends SubsystemBase {
  // motors & devices
  private final TalonFX m_spinner = new TalonFX(Constants.SPINNER_MAIN, CANBus.roboRIO());

  // output requests
  private final VelocityDutyCycle m_request = new VelocityDutyCycle(0.0);

  // preferences
  private final DoublePreferenceConstant p_spinnerSpeed =
      new DoublePreferenceConstant("Spinner/SpinnerSpeed", 100.0);
  private final MotionMagicPIDPreferenceConstants p_spinnerConfigConstants =
      new MotionMagicPIDPreferenceConstants(
          "Spinner/SpinnerMotors", 0., 0., 0., 0., 0., 0., 0.01, 0., 0.);

  private final BooleanSupplier m_turretOnTarget;

  public HotTub(BooleanSupplier turretOnTarget) {
    m_turretOnTarget = turretOnTarget;

    configureTalons();
    SmartDashboard.putData("Spinner/RunSpinner", runSpinner());
    SmartDashboard.putData("Spinner/StopSpinner", stopSpinner());
  }

  private void configureTalons() {
    TalonFXConfiguration spinnerConfig = new TalonFXConfiguration();
    spinnerConfig.Slot0.kP = p_spinnerConfigConstants.getKP().getValue();
    spinnerConfig.Slot0.kI = p_spinnerConfigConstants.getKI().getValue();
    spinnerConfig.Slot0.kD = p_spinnerConfigConstants.getKD().getValue();
    spinnerConfig.Slot0.kV = p_spinnerConfigConstants.getKV().getValue();
    spinnerConfig.Slot0.kS = p_spinnerConfigConstants.getKS().getValue();
    spinnerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_spinner.getConfigurator().apply(spinnerConfig);
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

  private void setSpinnerSpeed(DoubleSupplier speed) {
    m_spinner.setControl(m_request.withVelocity(speed.getAsDouble()));
  }

  private void stopSpinnerMotors() {
    m_spinner.stopMotor();
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
    return new RunCommand(() -> setSpinnerSpeed(() -> -p_spinnerSpeed.getValue()), this);
  }

  public Command stopSpinner() {
    return new RunCommand(() -> stopSpinnerMotors(), this);
  }
}
