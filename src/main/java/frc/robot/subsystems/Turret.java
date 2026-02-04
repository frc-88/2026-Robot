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
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;

/** insert new haiku here */
public class Turret extends SubsystemBase {
  private TalonFX m_turret = new TalonFX(Constants.TURRET_MOTOR_ID, CANBus.roboRIO());
  private CANcoder m_cancoder = new CANcoder(Constants.TURRET_CANCODER_ID, CANBus.roboRIO());

  private GyroIO m_gyro;

  private MotionMagicDutyCycle motionMagicReq = new MotionMagicDutyCycle(null);
  private DutyCycleOut dutyCycleReq = new DutyCycleOut(0);
  private VoltageOut voltageReq = new VoltageOut(0);

  // Preferences
  private DoublePreferenceConstant p_zeroPosition =
      new DoublePreferenceConstant("Turret Zero", 0.0);
  private DoublePreferenceConstant p_limitBuffer =
      new DoublePreferenceConstant("Turret Limit Buffer", 0.0);
  private DoublePreferenceConstant p_syncThreshold =
      new DoublePreferenceConstant("Turret Sync Threshold", 0.0);
  private MotionMagicPIDPreferenceConstants p_turretPID =
      new MotionMagicPIDPreferenceConstants("Turret PID", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  private DoublePreferenceConstant p_maxVelocity =
      new DoublePreferenceConstant("Turret Max Velocity", 0);
  private DoublePreferenceConstant p_maxAcceleration =
      new DoublePreferenceConstant("Turret Max Acceleration", 0);
  private DoublePreferenceConstant p_nominalForward =
      new DoublePreferenceConstant("Turret Nominal Forward", 0.0);
  private DoublePreferenceConstant p_nominalReverse =
      new DoublePreferenceConstant("Turret Nominal Reverse", 0.0);
  private DoublePreferenceConstant p_forwardLimit =
      new DoublePreferenceConstant("Turret Forward Limit", 0.0);
  private DoublePreferenceConstant p_reverseLimit =
      new DoublePreferenceConstant("Turret Reverse Limit", 0.0);

  //
  private boolean m_tracking = false;
  private boolean m_circumnavigating = false;
  private double m_circumnavigationTarget;

  private double m_defaultFacing = 0.;
  private boolean m_hasTarget = true;
  private double m_target = 0;

  /** Creates a new Turret. */
  public Turret(GyroIO gyro) {
    m_gyro = gyro;

    configureMotors();
    configureCANCoder();

    p_turretPID.addChangeHandler((Double unused) -> configureMotors());
    p_maxVelocity.addChangeHandler((Double unused) -> configureMotors());
    p_maxAcceleration.addChangeHandler((Double unused) -> configureMotors());
    p_nominalForward.addChangeHandler((Double unused) -> configureMotors());
    p_nominalReverse.addChangeHandler((Double unused) -> configureMotors());
    p_forwardLimit.addChangeHandler((Double unused) -> configureMotors());
    p_reverseLimit.addChangeHandler((Double unused) -> configureMotors());

    // initialize Falcon to correct position when we wake up based on CANcoder absolute position
    sync();
  }

  private void configureMotors() {
    // configure TalonFX
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotionMagic.MotionMagicCruiseVelocity = p_turretPID.getMaxVelocity().getValue();
    config.MotionMagic.MotionMagicAcceleration = p_turretPID.getMaxAcceleration().getValue();
    config.Slot0.kP = p_turretPID.getKP().getValue();
    config.Slot0.kI = p_turretPID.getKI().getValue();
    config.Slot0.kD = p_turretPID.getKD().getValue();
    config.Slot0.kV = p_turretPID.getKV().getValue();
    config.Slot0.kS = p_turretPID.getKS().getValue();
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        turretFacingToEncoderPosition(p_forwardLimit.getValue());
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        turretFacingToEncoderPosition(p_reverseLimit.getValue());
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    m_turret.getConfigurator().apply(config);
  }

  private void configureCANCoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    m_cancoder.getConfigurator().apply(config);
  }

  public void sync() {
    if (isEncoderConnected()) {
      m_turret.setPosition(
          cancoderPostionToFalconPosition(m_cancoder.getAbsolutePosition().getValueAsDouble()));
    } else {
      m_turret.setPosition(0.0);
    }
  }

  public void calibrate() {
    // This is only necessary if the CANcoder is moved or adjusted.
    // The turret must be physically moved to its center position.
    // WARNING - doing this when the turret isn't in the "zero"
    // position could cause the turret to move to unsafe positions.
    p_zeroPosition.setValue(m_cancoder.getAbsolutePosition().getValueAsDouble());
    sync();
  }

  public boolean isEncoderConnected() {
    if (m_cancoder.getMagnetHealth().getValue() == MagnetHealthValue.Magnet_Red
        || m_cancoder.getMagnetHealth().getValue() == MagnetHealthValue.Magnet_Invalid) {
      return false;
    }
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

  public void goToFacing(double target, boolean spinCompensation) {
    m_target = target;
    if (m_circumnavigating && !isFacingSafe(target)) {
      // if we are circumnavigating, ignore the input and keep doing that until we get there
      goToPosition(turretFacingToEncoderPosition(m_circumnavigationTarget), false);
      m_circumnavigating = Math.abs(m_circumnavigationTarget - getFacing()) > 5.0;
    } else if (isFacingSafe(target)) {
      // otherwise go to the input target if it is safe.
      goToPosition(turretFacingToEncoderPosition(target), spinCompensation);
    } else if (isFacingSafe(m_circumnavigationTarget = calcCircumnavigationTarget(target))) {
      // but if the target isn't safe, and our circumnavigation target is, start circumnavigating
      m_circumnavigating = true;
      // TODO? - adjust config here if different PID needed for targeting vs. circumnavigating
      goToPosition(turretFacingToEncoderPosition(m_circumnavigationTarget), false);
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
    return !m_tracking || Math.abs(getFacing() - m_target) < 10.;
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
    return isPositionSafe(turretFacingToEncoderPosition(degrees));
  }

  public double getFacing() {
    return turretEncoderPositionToFacing(getPosition());
  }

  public boolean isSynchronized() {
    return Math.abs(
            getFacing()
                - turretEncoderPositionToFacing(
                    cancoderPostionToFalconPosition(
                        m_cancoder.getAbsolutePosition().getValueAsDouble())))
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

  public boolean isSafeForClimber() {
    return Math.abs(getPosition()) % 180 < 10;
  }

  private double getPosition() {
    return m_turret.getPosition().getValueAsDouble();
  }

  private void goToPosition(double position, boolean spinCompensation) {
    if (spinCompensation) {
      GyroIOInputs gyroInputs = new GyroIOInputs();
      m_gyro.updateInputs(gyroInputs);
      m_turret.setControl(
          motionMagicReq
              .withPosition(position)
              .withFeedForward(
                  5
                      * 0.1
                      * p_turretPID.getKV().getValue()
                      * turretFacingToEncoderPosition(gyroInputs.yawVelocityRadPerSec)
                      / 1023.0));
    } else {
      m_turret.setControl(motionMagicReq.withPosition(position));
    }
  }

  private boolean isPositionSafe(double position) {
    return (position
            < turretFacingToEncoderPosition(p_forwardLimit.getValue() - p_limitBuffer.getValue()))
        && (position
            > turretFacingToEncoderPosition(p_reverseLimit.getValue() + p_limitBuffer.getValue()));
  }

  private double cancoderPostionToFalconPosition(double position) {
    double normalPosition = (position - p_zeroPosition.getValue());

    // if (normalPosition > 180) { normalPosition -= 360; }
    // if (normalPosition < -180) { normalPosition += 360; }

    return turretFacingToEncoderPosition(
        normalPosition * (Constants.TURRET_CANCODER_GEAR_RATIO / Constants.TURRET_GEAR_RATIO));
  }

  private double turretEncoderPositionToFacing(double turretPosition) {
    return (turretPosition / Constants.TURRET_COUNTS_PER_REV) * 360.0;
  }

  private double turretFacingToEncoderPosition(double degrees) {
    return (degrees / 360.0) * Constants.TURRET_COUNTS_PER_REV;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Turret:CANCoder Absolute", m_cancoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber(
        "Turret:CANCoder Position", m_cancoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(
        "Turret:CANCoder Turret Facing",
        turretEncoderPositionToFacing(
            cancoderPostionToFalconPosition(m_cancoder.getAbsolutePosition().getValueAsDouble())));
    SmartDashboard.putNumber("Turret:Position", getPosition());
    SmartDashboard.putNumber("Turret:Facing", getFacing());
    SmartDashboard.putBoolean("Turret:Synchonized", isSynchronized());
    SmartDashboard.putBoolean("Turret:Tracking", isTracking());
    SmartDashboard.putBoolean("Turret:Safe", isPositionSafe(getPosition()));
  }

  public void setHasTarget(boolean hasTarget) {
    m_hasTarget = true;
  }
}
