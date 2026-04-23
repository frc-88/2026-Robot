package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

// whenever I lift
// up and take a look around
// yellow ball deluge

public class Hood extends SubsystemBase {
  // motors & devices
  private final TalonFXS hood = new TalonFXS(Constants.HOOD, CANBus.roboRIO());

  // output requests
  private final MotionMagicVoltage request = new MotionMagicVoltage(0.0);
  private final DutyCycleOut calibrationRequest = new DutyCycleOut(0);

  // preferences
  private final MotionMagicPIDPreferenceConstants hoodConfigConstants =
      new MotionMagicPIDPreferenceConstants(
          "Hood/HoodMotor", 100., 250., 0., 8.0, 0., 0., 0.08, 0.45, 0.);
  private final DoublePreferenceConstant targetPos =
      new DoublePreferenceConstant("Hood/Target", 24.0);
  public DoublePreferenceConstant encoderOffset20Deg =
      new DoublePreferenceConstant(
          "Hood/EncoderOffset", 0.585938); // what the SRX encoder reads when hood is at 20 deg

  private final DoubleSupplier m_pitch;
  private double m_targetPitch = 0.0;
  private boolean isShooting = false;
  private boolean m_calibrated = false;

  public Hood(DoubleSupplier pitch) {
    m_pitch = pitch;
    hood.getRawPulseWidthPosition().setUpdateFrequency(1000);
    configureMinion();
    configureSmartDashboardButtons();
  }

  private void configureMinion() {
    TalonFXSConfiguration hoodConfig = new TalonFXSConfiguration();

    hoodConfig.Slot0.kP = hoodConfigConstants.getKP().getValue();
    hoodConfig.Slot0.kI = hoodConfigConstants.getKI().getValue();
    hoodConfig.Slot0.kD = hoodConfigConstants.getKD().getValue();
    hoodConfig.Slot0.kV = hoodConfigConstants.getKV().getValue();
    hoodConfig.Slot0.kS = hoodConfigConstants.getKS().getValue();
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hoodConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        hoodAngleDegreesToRotationsOfMinion(35.0 - 0.5);
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        hoodAngleDegreesToRotationsOfMinion(13.4 + 0.5);
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    hoodConfig.MotionMagic.MotionMagicCruiseVelocity =
        hoodConfigConstants.getMaxVelocity().getValue();
    hoodConfig.MotionMagic.MotionMagicAcceleration =
        hoodConfigConstants.getMaxAcceleration().getValue();

    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.StatorCurrentLimit = 40.0;

    hood.getConfigurator().apply(hoodConfig);

    setCalibrate();
  }

  private void configureSmartDashboardButtons() {
    SmartDashboard.putData("Hood/Calibrate", calibrate().ignoringDisable(true));
    SmartDashboard.putData("Hood/HardCalibrate", hardStopCalibrate());
    SmartDashboard.putData("Hood/SetPosition", setPositionTargeting());
    SmartDashboard.putData("Hood/SetPositionManual", setPositionManual());
  }

  @AutoLogOutput
  public boolean isHealthy() {
    return hood.isConnected() && hood.isAlive();
  }

  @AutoLogOutput
  private Voltage getVoltage() {
    return hood.getMotorVoltage().getValue();
  }

  @AutoLogOutput
  private Current getCurrent() {
    return hood.getTorqueCurrent().getValue();
  }

  @AutoLogOutput
  private AngularVelocity getVelocity() {
    return hood.getVelocity().getValue();
  }

  @AutoLogOutput
  private Angle getPosition() {
    return hood.getPosition().getValue();
  }

  @AutoLogOutput
  private double getAngle() {
    return minionRotationsToHoodAngleDegrees(hood.getPosition().getValueAsDouble());
  }

  @AutoLogOutput
  private boolean getIsShooting() {
    return isShooting;
  }

  @AutoLogOutput
  private boolean getIsCalibrated() {
    return m_calibrated;
  }

  private double hoodAngleDegreesToRotationsOfMinion(double hoodAngle) {
    return (hoodAngle / 360.0) * (287.0 / 18.0) * (36.0 / 12.0);
  }

  private double minionRotationsToHoodAngleDegrees(double minionRotations) { // inverse of ^^
    return (minionRotations * 360.0) / ((287.0 / 18.0) * (36.0 / 12.0));
  }

  private void calibrateFirst(double angle) {
    if (m_calibrated) {
      setPosition(m_targetPitch);
    } else {
      setCalibrate();
    }
  }

  private void setPosition(double angle) {
    hood.setControl(request.withPosition(hoodAngleDegreesToRotationsOfMinion(angle)));
  }

  private void hardCalibrate() { // TODO: add manual button
    hood.setControl(calibrationRequest.withOutput(-0.16).withIgnoreSoftwareLimits(true));
    if (hood.getStatorCurrent().getValueAsDouble() > 25.0) {
      hood.setPosition(hoodAngleDegreesToRotationsOfMinion(13.4));
      m_calibrated = Math.abs(getAngle() - 13.4) < 1.0;
    }
  }

  private void setCalibrate() {
    hood.setPosition(
        hoodAngleDegreesToRotationsOfMinion(
            20.0
                + encoderRotationsToHoodDegrees(
                    -hood.getRawPulseWidthPosition().getValueAsDouble()
                        + encoderOffset20Deg.getValue())));
    m_calibrated = true;
  }

  @AutoLogOutput
  public double getPulseWidthDeg() {
    return 20.0
        + encoderRotationsToHoodDegrees(
            -hood.getRawPulseWidthPosition().getValueAsDouble() + encoderOffset20Deg.getValue());
  }

  private double encoderRotationsToHoodDegrees(double rotations) {
    return rotations * (30.0 / 287.0) * 360.0;
  }

  private void stopHoodMotor() {
    hood.stopMotor();
  }

  public void periodic() {
    if (isShooting) {
      m_targetPitch = MathUtil.clamp(m_pitch.getAsDouble(), 15.0, 34.0);
    } else {
      m_targetPitch = 15.0;
    }
    // Lookup Table Building Override
    // m_targetPitch = targetPos.getValue();
  }

  public void setNotShooting() {
    isShooting = false;
  }

  public Command setNotShootingCommand() {
    return new InstantCommand(() -> isShooting = false);
  }

  public Command setIsShootingCommand() {
    return new InstantCommand(() -> isShooting = true);
  }

  public Command stopHood() {
    return new RunCommand(() -> stopHoodMotor(), this);
  }

  public Command calibrate() {
    return new InstantCommand(() -> setCalibrate(), this);
  }

  public Command hardStopCalibrate() {
    return new RunCommand(() -> hardCalibrate(), this)
        .until(() -> hood.getStatorCurrent().getValueAsDouble() > 10.0)
        .andThen(setPositionTargeting());
  }

  public Command setPositionTargeting() {
    return new RunCommand(() -> calibrateFirst(m_targetPitch), this);
  }

  public Command setPositionManual() {
    return new RunCommand(() -> calibrateFirst(targetPos.getValue()), this);
  }
}
