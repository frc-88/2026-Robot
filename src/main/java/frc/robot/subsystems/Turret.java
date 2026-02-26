// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.math.geometry.Rotation2d;
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
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

/** insert new haiku here */
public class Turret extends SubsystemBase {
  private final TalonFX m_turret = new TalonFX(Constants.TURRET_MOTOR_ID, CANBus.roboRIO());
  private final CANcoder m_cancoder66 =
      new CANcoder(Constants.TURRET_CANCODER_ID1, CANBus.roboRIO());
  private final CANcoder m_cancoder50 =
      new CANcoder(Constants.TURRET_CANCODER_ID2, CANBus.roboRIO());

  private final MotionMagicDutyCycle motionMagicReq = new MotionMagicDutyCycle(0.0);

  // Preferences
  private final DoublePreferenceConstant p_proportion =
      new DoublePreferenceConstant("Turret/Zero", 0.0);
  private final DoublePreferenceConstant p_limitBuffer =
      new DoublePreferenceConstant("Turret/Limit Buffer", 0.0);
  private final DoublePreferenceConstant p_syncThreshold =
      new DoublePreferenceConstant("Turret/Sync Threshold", 0.0);
  private final MotionMagicPIDPreferenceConstants p_turretPID =
      new MotionMagicPIDPreferenceConstants(
          "Turret/PID", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);
  private final DoublePreferenceConstant p_forwardLimit =
      new DoublePreferenceConstant("Turret/Forward Limit", 0.0);
  private final DoublePreferenceConstant p_reverseLimit =
      new DoublePreferenceConstant("Turret/Reverse Limit", 0.0);
  private final DoublePreferenceConstant p_spinCompensation =
      new DoublePreferenceConstant("Turret/Spin Compensation", 0.0);

  private Supplier<Rotation2d> m_robotYaw;
  private DoubleSupplier m_robotYawRate;
  private DoubleSupplier m_targetFacing;

  private boolean m_targeting = false;
  private double m_target = 0;

  private boolean m_circumnavigating = false;
  private double m_circumnavigationTarget;

  public Turret(
      Supplier<Rotation2d> driveYawSupplier,
      DoubleSupplier driveGyroRateSupplier,
      DoubleSupplier trajectorySolverFacingSupplier) {
    m_robotYaw = driveYawSupplier;
    m_robotYawRate = driveGyroRateSupplier;
    m_targetFacing = trajectorySolverFacingSupplier;

    configureMotors();
    configureCANCoder();

    SmartDashboard.putData("Calibrate", calibrateZero().ignoringDisable(true));
    SmartDashboard.putData("Aim", aim());

    p_turretPID.addChangeHandler((Double unused) -> configureMotors());
    p_forwardLimit.addChangeHandler((Double unused) -> configureMotors());
    p_reverseLimit.addChangeHandler((Double unused) -> configureMotors());

    sync();
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotionMagic.MotionMagicCruiseVelocity = p_turretPID.getMaxVelocity().getValue();
    config.MotionMagic.MotionMagicAcceleration = p_turretPID.getMaxAcceleration().getValue();
    config.Slot0.kP = p_turretPID.getKP().getValue();
    config.Slot0.kI = p_turretPID.getKI().getValue();
    config.Slot0.kD = p_turretPID.getKD().getValue();
    config.Slot0.kV = p_turretPID.getKV().getValue();
    config.Slot0.kS = p_turretPID.getKS().getValue();
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        turretFacingToFalconEncoderPosition(p_forwardLimit.getValue());
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        turretFacingToFalconEncoderPosition(p_reverseLimit.getValue());
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_turret.getConfigurator().apply(config);
  }

  private void configureCANCoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    m_cancoder66.getConfigurator().apply(config);
    m_cancoder50.getConfigurator().apply(config);
  }

  private void sync() {
    m_turret.setPosition(getCANCoderFacing());
  }

  private void calibrate() {
    // This is only necessary if the CANcoder is moved or adjusted.
    // The turret must be physically moved to its center position.
    // WARNING - doing this when the turret isn't in the "zero"
    // position could cause the turret to move to unsafe positions.
    m_cancoder50.setPosition(0.0);
    m_cancoder66.setPosition(0.0);
    sync();
  }

  @AutoLogOutput
  private double getCANCoderFacing() {
    return (m_cancoder66.getPosition().getValueAsDouble()
            - m_cancoder50.getPosition().getValueAsDouble())
        * p_proportion.getValue();
  }

  @AutoLogOutput
  private boolean encodersHealthy() {
    return m_cancoder66.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Red
        && m_cancoder66.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Invalid
        && m_cancoder50.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Red
        && m_cancoder50.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Invalid;
  }

  private void aimAtTarget() {
    double target =
        (Util.weAreRed() ? m_targetFacing.getAsDouble() : 180.0 - m_targetFacing.getAsDouble());
    target -= m_robotYaw.get().getDegrees();
    goToFacing(m_targeting ? target : 0.0);
  }

  private void goToFacing(double target) {
    goToFacing(target, false);
  }

  private void goToFacing(double target, boolean spinCompensation) {
    m_target = target;
    if (m_circumnavigating && !isFacingSafe(target)) {
      // if we are circumnavigating, ignore the input and keep doing that until we get there
      goToPosition(turretFacingToFalconEncoderPosition(m_circumnavigationTarget), false);
      m_circumnavigating = Math.abs(m_circumnavigationTarget - getFacing()) > 5.0;
    } else if (isFacingSafe(target)) {
      // otherwise go to the input target if it is safe.
      goToPosition(turretFacingToFalconEncoderPosition(target), spinCompensation);
    } else if (isFacingSafe(m_circumnavigationTarget = calcCircumnavigationTarget(target))) {
      // but if the target isn't safe, and our circumnavigation target is, start circumnavigating
      m_circumnavigating = true;
      // TODO? - adjust config here if different PID needed for targeting vs. circumnavigating
      goToPosition(turretFacingToFalconEncoderPosition(m_circumnavigationTarget), false);
    } else {
      System.out.println("Turret unsafe target: " + target);
      // target is unsafe and circumnavigation target is unsafe, ignore it
    }
  }

  private void goToPosition(double position, boolean spinCompensation) {
    if (spinCompensation) {
      m_turret.setControl(
          motionMagicReq
              .withPosition(position)
              .withFeedForward(p_spinCompensation.getValue() * m_robotYawRate.getAsDouble()));
    } else {
      m_turret.setControl(motionMagicReq.withPosition(position));
    }
  }

  public boolean onTarget() {
    return !m_targeting || Math.abs(getFacing() - m_target) < 5.0;
  }

  private boolean notMoving() {
    return Math.abs(turretEncoderPositionToFacing(m_turret.getVelocity().getValueAsDouble()) * 10.)
        < 45.;
  }

  private double calcCircumnavigationTarget(double origin) {
    double target;

    if (origin > 0.0) {
      target = origin - 360.0;
    } else {
      target = origin + 360.0;
    }

    return target;
  }

  @AutoLogOutput
  private double getFacing() {
    return turretEncoderPositionToFacing(getPosition());
  }

  @AutoLogOutput
  private double getPosition() {
    return m_turret.getPosition().getValueAsDouble();
  }

  @AutoLogOutput
  private boolean isSynchronized() {
    return Math.abs(getFacing() - getCANCoderFacing()) < p_syncThreshold.getValue();
  }

  private boolean isFacingSafe(double degrees) {
    return isPositionSafe(turretFacingToFalconEncoderPosition(degrees));
  }

  private boolean isPositionSafe(double position) {
    return (position
            < turretFacingToFalconEncoderPosition(
                p_forwardLimit.getValue() - p_limitBuffer.getValue()))
        && (position
            > turretFacingToFalconEncoderPosition(
                p_reverseLimit.getValue() + p_limitBuffer.getValue()));
  }

  @AutoLogOutput
  private boolean isCurrentPositionSafe() {
    return isPositionSafe(getPosition());
  }

  @AutoLogOutput
  private boolean isTracking() {
    return m_targeting;
  }

  @AutoLogOutput
  private double getTarget() {
    return m_target;
  }

  private double turretEncoderPositionToFacing(double turretPosition) {
    return (turretPosition / (5 * (100 / 12)) * 360.0);
  }

  private double turretFacingToFalconEncoderPosition(double degrees) {
    return (degrees / 360.0) * (5 * (100 / 12));
  }

  public Command calibrateZero() {
    return new InstantCommand(() -> calibrate(), this);
  }

  public Command aim() {
    return new RunCommand(() -> aimAtTarget(), this);
  }

  public Command stop() {
    return new RunCommand(() -> m_turret.stopMotor(), this);
  }

  public Command startTargeting() {
    return new InstantCommand(() -> m_targeting = true);
  }

  public Command stopTargeting() {
    return new InstantCommand(() -> m_targeting = false);
  }

  @Override
  public void periodic() {
    if (Util.logif()) {
      SmartDashboard.putNumber(
          "Turret/Cancoder66Position", m_cancoder66.getPosition().getValueAsDouble());
      SmartDashboard.putNumber(
          "Turret/Cancoder50Position", m_cancoder50.getPosition().getValueAsDouble());
    }
  }
}
