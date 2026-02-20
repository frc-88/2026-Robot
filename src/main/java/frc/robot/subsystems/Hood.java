package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Util;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;
import java.util.function.DoubleSupplier;

public class Hood extends SubsystemBase {
  private TalonFXS hood = new TalonFXS(Constants.HOOD, CANBus.roboRIO());

  private final MotionMagicPIDPreferenceConstants hoodConfigConstants =
      new MotionMagicPIDPreferenceConstants("Hood/HoodMotor");
  private DoublePreferenceConstant targetPos = new DoublePreferenceConstant("Hood/Target", 0);
  private MotionMagicVoltage request = new MotionMagicVoltage(0.0);
  private DutyCycleOut calibrationRequest = new DutyCycleOut(0);
  private DoubleSupplier m_pitch;
  private double m_targetPitch = 0.0;

  public boolean isShooting = false;

  public Hood(DoubleSupplier pitch) {
    m_pitch = pitch;
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
        hoodAngleDegreesToRotationsOfMinion(34.5);
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        hoodAngleDegreesToRotationsOfMinion(13.5);
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    hoodConfig.MotionMagic.MotionMagicCruiseVelocity =
        hoodConfigConstants.getMaxVelocity().getValue();
    hoodConfig.MotionMagic.MotionMagicAcceleration =
        hoodConfigConstants.getMaxAcceleration().getValue();
    hood.getConfigurator().apply(hoodConfig);
  }

  private void configureSmartDashboardButtons() {
    // SmartDashboard.putNumber("Hood/Position", hood.getPosition().getValueAsDouble());
    if (Util.logif()) {
      SmartDashboard.putData("Hood/Calibrate", calibrate());
      SmartDashboard.putData("Hood/SetPosition", setPositionTargeting());
      SmartDashboard.putData("Hood/SetPositionManual", setPositionTargeting());
      SmartDashboard.putData("Hood/GoToZero", goToZero());
    }
  }

  public void periodic() {
    // if (isShooting) {
    //   m_targetPitch = 90.0 - m_pitch.getAsDouble();
    // } else {
    //   m_targetPitch = 14.0;
    // }
    m_targetPitch = targetPos.getValue();
    if (Util.logif()) {
      SmartDashboard.putNumber("Hood/Current", hood.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber(
          "Hood/Setpoint", hoodAngleDegreesToRotationsOfMinion(targetPos.getValue()));
      SmartDashboard.putNumber("Hood/CurrentPosition", hood.getPosition().getValueAsDouble());
      SmartDashboard.putNumber(
          "Hood/CurrentAngle",
          minionRotationsToHoodAngleDegrees(hood.getPosition().getValueAsDouble()));
    }
  }

  private double hoodAngleDegreesToRotationsOfMinion(double hoodAngle) {
    return (hoodAngle / 360.0) * (287.0 / 18.0) * (36.0 / 12.0);
  }

  private double minionRotationsToHoodAngleDegrees(double minionRotations) { // inverse of ^^
    return (minionRotations * 360.0) / ((287.0 / 18.0) * (36.0 / 12.0));
  }

  private void setPosition(double angle) {
    hood.setControl(request.withPosition(hoodAngleDegreesToRotationsOfMinion(angle)));
    System.out.println(angle);
  }

  private void setCalibrate() {
    hood.setControl(calibrationRequest.withOutput(-0.16).withIgnoreSoftwareLimits(true));
    if (hood.getStatorCurrent().getValueAsDouble() > 20.0) {
      hood.setPosition(hoodAngleDegreesToRotationsOfMinion(13.5));
    }
  }

  private void stopHoodMotor() {
    hood.stopMotor();
  }

  public Command setNotShooting() {
    return new InstantCommand(() -> isShooting = false);
  }

  public Command setIsShooting() {
    return new InstantCommand(() -> isShooting = true);
  }

  public Command stopHood() {
    return new RunCommand(() -> stopHoodMotor(), this);
  }

  public Command calibrate() {
    return new RunCommand(() -> setCalibrate(), this)
        .until(() -> hood.getStatorCurrent().getValueAsDouble() > 60.0)
        .andThen(stopHood());
  }

  public Command setPositionTargeting() {
    return new RunCommand(() -> setPosition(m_targetPitch), this);
  }

  public Command setPositionManual() {
    return new RunCommand(() -> setPosition(25.0), this);
  }

  public Command goToZero() {
    return new RunCommand(() -> setPosition(0.0), this);
  }
}
