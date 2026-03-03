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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
      new DoublePreferenceConstant("Turret/Conversion Constant", -262.0588);
  private final DoublePreferenceConstant p_limitBuffer =
      new DoublePreferenceConstant("Turret/Limit Buffer", 10.0);
  private final DoublePreferenceConstant p_syncThreshold =
      new DoublePreferenceConstant("Turret/Sync Threshold", 5.0);
  private final MotionMagicPIDPreferenceConstants p_turretPID =
      new MotionMagicPIDPreferenceConstants(
          "Turret/PID", 40.0, 20.0, 0.0, 3.2, 0.0, 0.0, 0.01, 0.0, 0);
  private final DoublePreferenceConstant p_forwardLimit =
      new DoublePreferenceConstant("Turret/Forward Limit", 225.0);
  private final DoublePreferenceConstant p_reverseLimit =
      new DoublePreferenceConstant("Turret/Reverse Limit", -225.0);
  private final DoublePreferenceConstant p_spinCompensation =
      new DoublePreferenceConstant("Turret/Spin Compensation", 0.0);
  private final DoublePreferenceConstant p_cancoder50offset =
      new DoublePreferenceConstant("Turret/CANCoder50 Offset", 0.0);
  private final DoublePreferenceConstant p_cancoder66offset =
      new DoublePreferenceConstant("Turret/CANCoder66 Offset", 0.0);

  private Supplier<Pose2d> m_robotPose;
  private DoubleSupplier m_robotYawRate;
  private DoubleSupplier m_targetFacing;

  private boolean m_targeting = true;
  private double m_target = 0;

  private boolean m_circumnavigating = false;
  private double m_circumnavigationTarget;

  public Turret(
      Supplier<Pose2d> drivePoseSupplier,
      DoubleSupplier driveGyroRateSupplier,
      DoubleSupplier trajectorySolverFacingSupplier) {
    m_robotPose = drivePoseSupplier;
    m_robotYawRate = driveGyroRateSupplier;
    m_targetFacing = trajectorySolverFacingSupplier;

    configureMotors();
    configureCANCoders();

    SmartDashboard.putData("Turret/Calibrate", calibrateZero().ignoringDisable(true));
    SmartDashboard.putData("Turret/Sync", syncCommand().ignoringDisable(true));
    SmartDashboard.putData("Turret/Aim", aim());
    SmartDashboard.putData("Turret/Start Targeting", startTargeting());
    SmartDashboard.putData("Turret/Stop Targeting", stopTargeting());
    SmartDashboard.putData("Turret/CalibrateEncoderZero", calibrateZero().ignoringDisable(true));

    p_turretPID.addChangeHandler((Double unused) -> configureMotors());
    p_forwardLimit.addChangeHandler((Double unused) -> configureMotors());
    p_reverseLimit.addChangeHandler((Double unused) -> configureMotors());

    m_cancoder50.setPosition(m_cancoder50.getAbsolutePosition().getValue());

    CommandScheduler.getInstance().schedule(syncCommand());
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

  private void configureCANCoders() {
    CANcoderConfiguration config50 = new CANcoderConfiguration();
    CANcoderConfiguration config66 = new CANcoderConfiguration();

    config50.MagnetSensor.MagnetOffset = p_cancoder50offset.getValue();
    config66.MagnetSensor.MagnetOffset = p_cancoder66offset.getValue();

    m_cancoder50.getConfigurator().apply(config50);
    m_cancoder66.getConfigurator().apply(config66);
  }

  private void sync() {
    m_turret.setPosition(turretFacingToFalconEncoderPosition(getCANCoderFacing()));
  }

  private void calibrate() {
    // This is only necessary if the CANcoders are moved or adjusted.
    // The turret must be physically moved to its center position.
    // WARNING - doing this when the turret isn't in the "zero"
    // position could cause the turret to move to unsafe positions.
    p_cancoder50offset.setValue(
        -m_cancoder50.getAbsolutePosition().getValueAsDouble() + p_cancoder50offset.getValue());
    // p_cancoder66offset.setValue(
    // -m_cancoder66.getAbsolutePosition().getValueAsDouble() + p_cancoder66offset.getValue());
    configureCANCoders();
  }

  @AutoLogOutput
  private double getCANCoderFacing() {
    return (m_cancoder50.getAbsolutePosition().getValueAsDouble() * 1325.0); // TODO: WHY
  }

  @AutoLogOutput
  private boolean encodersHealthy() {
    return // m_cancoder66.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Red
    // && m_cancoder66.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Invalid
    m_cancoder50.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Red
        && m_cancoder50.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Invalid;
  }

  private void aimAtTarget() {
    goToFacing(m_targeting ? getTargetFacing() : 0.0);
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
  public double getFacing() {
    return turretEncoderPositionToFacing(getPosition());
  }

  @AutoLogOutput(key = "Turret/Velocity")
  public double getFacingOmega() {
    return turretEncoderPositionToFacing(m_turret.getVelocity().getValueAsDouble());
  }

  @AutoLogOutput
  private double getPosition() {
    return m_turret.getPosition().getValueAsDouble();
  }

  @AutoLogOutput
  private boolean isSynchronized() {
    return Math.abs(getFacing() - getCANCoderFacing()) < p_syncThreshold.getValue();
  }

  @AutoLogOutput
  private double syncError() {
    return getFacing() - getCANCoderFacing();
  }

  @AutoLogOutput
  private double facingError() {
    return getFacing() - getTargetFacing();
  }

  @AutoLogOutput
  private double getTargetFacing() {
    return m_targetFacing.getAsDouble();
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

  @AutoLogOutput
  public boolean onTarget() {
    return m_targeting
        && !m_circumnavigating
        && Math.abs(getFacing() - m_target) < 5.0; // TODO: Lower 5.0 threshold
  }

  public boolean notMoving() {
    return Math.abs(turretEncoderPositionToFacing(m_turret.getVelocity().getValueAsDouble()) * 10.)
        < 45.;
  }

  public Command calibrateZero() {
    return new InstantCommand(() -> calibrate(), this);
  }

  public Command syncCommand() {
    return new InstantCommand(() -> sync(), this);
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
          "Turret/Cancoder66Position", m_cancoder66.getAbsolutePosition().getValueAsDouble());
      SmartDashboard.putNumber(
          "Turret/Constant", getFacing() / m_cancoder50.getAbsolutePosition().getValueAsDouble());

      SmartDashboard.putNumber(
          "Turret/Cancoder50Position", m_cancoder50.getAbsolutePosition().getValueAsDouble());
    }
  }
}
