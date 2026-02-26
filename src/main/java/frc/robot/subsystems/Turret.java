// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
  private TalonFX m_turret = new TalonFX(Constants.TURRET_MOTOR_ID, CANBus.roboRIO());
  private CANcoder m_cancoder66 = new CANcoder(Constants.TURRET_CANCODER_ID1, CANBus.roboRIO());
  private CANcoder m_cancoder50 = new CANcoder(Constants.TURRET_CANCODER_ID2, CANBus.roboRIO());

  Supplier<Rotation2d> m_robotYaw;
  DoubleSupplier m_rate;

  DoubleSupplier m_targetFacing;
  private double m_currentTargetFacing = 0.0;

  private MotionMagicDutyCycle motionMagicReq = new MotionMagicDutyCycle(0.0);
  private DutyCycleOut dutyCycleReq = new DutyCycleOut(0);
  private VoltageOut voltageReq = new VoltageOut(0);

  // Preferences
  private DoublePreferenceConstant p_proportion = new DoublePreferenceConstant("Turret/Zero", 0.0);
  private DoublePreferenceConstant p_limitBuffer =
      new DoublePreferenceConstant("Turret/Limit Buffer", 0.0);
  private DoublePreferenceConstant p_syncThreshold =
      new DoublePreferenceConstant("Turret/Sync Threshold", 0.0);
  private MotionMagicPIDPreferenceConstants p_turretPID =
      new MotionMagicPIDPreferenceConstants(
          "Turret/PID", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);
  private DoublePreferenceConstant p_forwardLimit =
      new DoublePreferenceConstant("Turret/Forward Limit", 0.0);
  private DoublePreferenceConstant p_reverseLimit =
      new DoublePreferenceConstant("Turret/Reverse Limit", 0.0);

  private boolean m_tracking = false;
  private boolean m_circumnavigating = false;
  private double m_circumnavigationTarget;
  private double m_defaultFacing = 0.;
  private double m_target = 0;

  public Turret(
      Supplier<Rotation2d> driveYawSupplier,
      DoubleSupplier driveGyroRateSupplier,
      DoubleSupplier trajectorySolverFacingSupplier) {
    m_robotYaw = driveYawSupplier;
    m_rate = driveGyroRateSupplier;
    m_targetFacing = trajectorySolverFacingSupplier;

    configureMotors();
    configureCANCoder();

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

  public void sync() {
    // if (isEncoderConnected()) {
    //   m_turret.setPosition(turretFacingToFalconEncoderPosition(getAbsoluteAngleOfTurret()));
    // } else {
    //   m_turret.setPosition(0.0);
    // }
    m_turret.setPosition(0.0);
  }

  public void calibrateEncoders() {
    // This is only necessary if the CANcoder is moved or adjusted.
    // The turret must be physically moved to its center position.
    // WARNING - doing this when the turret isn't in the "zero"
    // position could cause the turret to move to unsafe positions.
    m_cancoder50.setPosition(0.0);
    m_cancoder66.setPosition(0.0);
    sync();
  }

  public void calibrateZero() {
    m_turret.setPosition(0.0);
  }

  public boolean isEncoderConnected() {
    // if (m_cancoder66.getMagnetHealth().getValue() == MagnetHealthValue.Magnet_Red
    //     || m_cancoder66.getMagnetHealth().getValue() == MagnetHealthValue.Magnet_Invalid) {
    //   return false;
    // }
    return true;
  }

  public void setDutyCycle(double dutyCycle) {
    m_turret.setControl(dutyCycleReq.withOutput(dutyCycle));
  }

  public void setVoltage(double voltage) {
    m_turret.setControl(voltageReq.withOutput(voltage));
  }

  public void goToFacing(double target) {
    goToFacing(target, false);
  }

  public void stopMotors() {
    m_turret.stopMotor();
  }

  public void goToFacing(double target, boolean spinCompensation) {
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

  public void goToDefaultFacing() {
    goToFacing(m_defaultFacing);
  }

  public void setDefaultFacing(double facing) {
    m_defaultFacing = facing;
  }

  public double getDefaultFacing() {
    return m_defaultFacing;
  }

  public boolean onTarget() {
    return !m_tracking || Math.abs(getFacing() - m_target) < 5.0;
  }

  public boolean notMoving() {
    return Math.abs(turretEncoderPositionToFacing(m_turret.getVelocity().getValueAsDouble()) * 10.)
        < 45.;
  }

  public double getTarget() {
    return m_target;
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

  public boolean isFacingSafe(double degrees) {
    return isPositionSafe(turretFacingToFalconEncoderPosition(degrees));
  }

  public double getFacing() {
    return turretEncoderPositionToFacing(getPosition());
  }

  public boolean isSynchronized() {
    return Math.abs(getFacing() - turretFacingToFalconEncoderPosition(getAbsoluteAngleOfTurret()))
        < p_syncThreshold.getValue();
  }

  public void setNeutralMode(NeutralModeValue mode) {
    m_turret.setNeutralMode(mode);
  }

  public void startTracking() {
    m_tracking = true;
  }

  public void stopTracking() {
    m_tracking = false;
  }

  public boolean isTracking() {
    return m_tracking;
  }

  private double getPosition() {
    return m_turret.getPosition().getValueAsDouble();
  }

  private void goToPosition(double position, boolean spinCompensation) {
    // if (spinCompensation) {
    //   m_turret.setControl(
    //       motionMagicReq
    //           .withPosition(position)
    //           .withFeedForward(
    //               5
    //                   * 0.1
    //                   * p_turretPID.getKV().getValue()
    //                   * turretFacingToEncoderPosition(m_rate.getAsDouble())
    //                   / 1023.0));
    // } else {
    m_turret.setControl(motionMagicReq.withPosition(position));
    // }
  }

  @AutoLogOutput(key = "Turret/RobotFieldYaw")
  public double getRobotFieldYaw() {
    return m_robotYaw.get().getDegrees();
  }

  private boolean isPositionSafe(double position) {
    return (position
            < turretFacingToFalconEncoderPosition(
                p_forwardLimit.getValue() - p_limitBuffer.getValue()))
        && (position
            > turretFacingToFalconEncoderPosition(
                p_reverseLimit.getValue() + p_limitBuffer.getValue()));
  }

  private double getAbsoluteAngleOfTurret() {
    return turretEncoderPositionToFacing(m_turret.getPosition().getValueAsDouble());
  }

  private double turretEncoderPositionToFacing(double turretPosition) {
    return (turretPosition / (5 * (100 / 12)) * 360.0);
  }

  private double turretFacingToFalconEncoderPosition(double degrees) {
    return (degrees / 360.0) * (5 * (100 / 12));
  }

  public Command calibrateEncodersFactory() {
    return new InstantCommand(() -> calibrateEncoders(), this);
  }

  public Command calibrateTurret() {
    return new InstantCommand(
        () ->
            m_turret.setPosition(turretFacingToFalconEncoderPosition(getAbsoluteAngleOfTurret())));
  }

  public Command setPositionTargeting() {
    return new RunCommand(() -> goToFacing(m_currentTargetFacing), this);
  }

  public Command setPositionToZero() {
    return new RunCommand(() -> stopMotors(), this);
  }

  @Override
  public void periodic() {
    m_currentTargetFacing =
        (Util.weAreRed() ? m_targetFacing.getAsDouble() : 180.0 - m_targetFacing.getAsDouble())
            - getRobotFieldYaw();
    if (Util.logif()) {
      SmartDashboard.putNumber("Turret/TargetFacingAngle", m_currentTargetFacing);
      SmartDashboard.putNumber(
          "Turret/TalonEncoderPosition", m_turret.getPosition().getValueAsDouble());
      SmartDashboard.putNumber(
          "Turret/CANCoder66Position", m_cancoder66.getPosition().getValueAsDouble());
      SmartDashboard.putNumber(
          "Turret/CANcoderFacingAngle",
          (m_cancoder66.getPosition().getValueAsDouble()
                  - m_cancoder50.getPosition().getValueAsDouble())
              * p_proportion.getValue());
      SmartDashboard.putNumber("Turret/CurrentPosition", getPosition());
      SmartDashboard.putNumber("Turret/CurrentFacingAngle", getFacing());
      SmartDashboard.putBoolean("Turret/Synchonized", isSynchronized());
      SmartDashboard.putBoolean("Turret/Tracking", isTracking());
      SmartDashboard.putBoolean("Turret/Safe", isPositionSafe(getPosition()));
      SmartDashboard.putNumber(
          "Turret/Cancoder66Position", m_cancoder66.getPosition().getValueAsDouble());
      SmartDashboard.putNumber(
          "Turret/Cancoder50Position", m_cancoder50.getPosition().getValueAsDouble());
    }
  }
}
