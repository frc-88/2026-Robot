// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
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
import org.littletonrobotics.junction.AutoLogOutput;

/** insert new haiku here */
public class Turret extends SubsystemBase {
  // motors & devices
  private final TalonFX m_turret = new TalonFX(Constants.TURRET_MOTOR_ID, CANBus.roboRIO());
  private final TalonFX m_retractomatic =
      new TalonFX(Constants.TURRET_RETRACTOMATIC_ID, CANBus.roboRIO());
  private final CANcoder m_CANcoder = new CANcoder(Constants.TURRET_CANCODER_ID2, CANBus.roboRIO());

  private final MotionMagicVoltage motionMagicReq = new MotionMagicVoltage(0.0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
  private final TorqueCurrentFOC torqueReq = new TorqueCurrentFOC(0.0);

  // Preferences
  private final DoublePreferenceConstant p_proportion =
      new DoublePreferenceConstant("Turret/Conversion Constant", -260.0);
  private final DoublePreferenceConstant p_limitBuffer =
      new DoublePreferenceConstant("Turret/Limit Buffer", 10.0);
  private final DoublePreferenceConstant p_syncThreshold =
      new DoublePreferenceConstant("Turret/Sync Threshold", 0.75);
  private final MotionMagicPIDPreferenceConstants p_turretPID =
      new MotionMagicPIDPreferenceConstants(
          "Turret/PID", 100.0, 250.0, 0.0, 1.0, 0.0, 0.0, 0.01, 0.0, 0);
  private final DoublePreferenceConstant p_forwardLimit =
      new DoublePreferenceConstant("Turret/Forward Limit", 225.0);
  private final DoublePreferenceConstant p_reverseLimit =
      new DoublePreferenceConstant("Turret/Reverse Limit", -225.0);
  private final DoublePreferenceConstant p_spinCompensation =
      new DoublePreferenceConstant("Turret/Spin Compensation", 0.0);
  private final DoublePreferenceConstant p_CANcoderOffset =
      new DoublePreferenceConstant("Turret/CANCoder50 Offset", -0.143311);
  private final DoublePreferenceConstant p_goingOutCurrent =
      new DoublePreferenceConstant("Turret/Out Current", 0.0);
  private final DoublePreferenceConstant p_goingInCurrent =
      new DoublePreferenceConstant("Turret/In Current", -9.0);

  private final DoubleSupplier m_robotYawRate;
  private final DoubleSupplier m_targetFacing;

  private boolean m_targeting = true;
  private double m_target = 0;

  private boolean m_circumnavigating = false;
  private double m_circumnavigationTarget;

  // my head is spinning
  // newton's approximation
  // shows the way to look

  public Turret(
      DoubleSupplier driveGyroRateSupplier, DoubleSupplier trajectorySolverFacingSupplier) {
    m_robotYawRate = driveGyroRateSupplier;
    m_targetFacing = trajectorySolverFacingSupplier;

    configureMotors();
    configureCANCoder();

    SmartDashboard.putData("Turret/SyncTurretToEncoder", syncCommand().ignoringDisable(true));
    SmartDashboard.putData("Turret/Aim", aim());
    SmartDashboard.putData("Turret/Start Targeting", startTargeting());
    SmartDashboard.putData("Turret/Stop Targeting", stopTargeting());
    SmartDashboard.putData(
        "Turret/CalibrateEncoderZero", calibrateEncoderCommand().ignoringDisable(true));

    p_turretPID.addChangeHandler((Double unused) -> configureMotors());
    p_forwardLimit.addChangeHandler((Double unused) -> configureMotors());
    p_reverseLimit.addChangeHandler((Double unused) -> configureMotors());

    m_CANcoder.setPosition(m_CANcoder.getAbsolutePosition().getValue());

    sync();
    CommandScheduler.getInstance().schedule(syncCommand().ignoringDisable(true));
  }

  private void configureMotors() {
    TalonFXConfiguration turretCfg = new TalonFXConfiguration();
    turretCfg.MotionMagic.MotionMagicCruiseVelocity = p_turretPID.getMaxVelocity().getValue();
    turretCfg.MotionMagic.MotionMagicAcceleration = p_turretPID.getMaxAcceleration().getValue();
    turretCfg.Slot0.kP = p_turretPID.getKP().getValue();
    turretCfg.Slot0.kI = p_turretPID.getKI().getValue();
    turretCfg.Slot0.kD = p_turretPID.getKD().getValue();
    turretCfg.Slot0.kV = p_turretPID.getKV().getValue();
    turretCfg.Slot0.kS = p_turretPID.getKS().getValue();
    turretCfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        turretFacingToFalconEncoderPosition(p_forwardLimit.getValue());
    turretCfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turretCfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        turretFacingToFalconEncoderPosition(p_reverseLimit.getValue());
    turretCfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    turretCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_turret.getConfigurator().apply(turretCfg);

    TalonFXConfiguration retractomaticCfg = new TalonFXConfiguration();
    retractomaticCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_retractomatic.getConfigurator().apply(retractomaticCfg);
  }

  private void configureCANCoder() {
    CANcoderConfiguration canCoderCfg = new CANcoderConfiguration();

    canCoderCfg.MagnetSensor.MagnetOffset = p_CANcoderOffset.getValue();

    m_CANcoder.getConfigurator().apply(canCoderCfg);
  }

  private void sync() {
    m_turret.setPosition(turretFacingToFalconEncoderPosition(getCANCoderFacing()));
  }

  private void calibrateEncoder() {
    // This is only necessary if the CANcoders are moved or adjusted.
    // The turret must be physically moved to its center position.
    // WARNING - doing this when the turret isn't in the "zero"
    // position could cause the turret to move to unsafe positions.
    double newOffset =
        -m_CANcoder.getAbsolutePosition().getValueAsDouble() + p_CANcoderOffset.getValue();
    if (newOffset > 1.0) {
      newOffset -= 1.0;
    } else if (newOffset < -1.0) {
      newOffset += 1.0;
    }

    p_CANcoderOffset.setValue(newOffset);
    configureCANCoder();
    CommandScheduler.getInstance().schedule(syncCommand().ignoringDisable(true));
  }

  @AutoLogOutput
  private Voltage getTurretVoltage() {
    return m_turret.getMotorVoltage().getValue();
  }

  @AutoLogOutput
  private Current getTurretCurrent() {
    return m_turret.getTorqueCurrent().getValue();
  }

  @AutoLogOutput
  private AngularVelocity getTurretVelocity() {
    return m_turret.getVelocity().getValue();
  }

  @AutoLogOutput
  private Angle getTurretRawPosition() {
    return m_turret.getPosition().getValue();
  }

  @AutoLogOutput
  private Voltage getRetractomaticVoltage() {
    return m_retractomatic.getMotorVoltage().getValue();
  }

  @AutoLogOutput
  private Current getRetractomaticCurrent() {
    return m_retractomatic.getTorqueCurrent().getValue();
  }

  @AutoLogOutput
  private AngularVelocity getRetractomaticVelocity() {
    return m_retractomatic.getVelocity().getValue();
  }

  @AutoLogOutput
  private Angle getRetractomaticPosition() {
    return m_turret.getPosition().getValue();
  }

  @AutoLogOutput
  private double getCANCoderFacing() {
    return turretEncoderPositionToFacing(
        m_CANcoder.getAbsolutePosition().getValueAsDouble() * 100.0 * (7.0 / 5.0));
  }

  @AutoLogOutput
  private double getCANCoderPosition() {
    return m_CANcoder.getAbsolutePosition().getValueAsDouble();
  }

  @AutoLogOutput
  public boolean isHealthy() {
    return encodersHealthy() && motorsHealthy();
  }

  private boolean encodersHealthy() {
    return m_CANcoder.isConnected()
        && m_CANcoder.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Red
        && m_CANcoder.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Invalid;
  }

  private boolean motorsHealthy() {
    return m_turret.isConnected()
        && m_turret.isAlive()
        && m_retractomatic.isConnected()
        && m_retractomatic.isAlive();
  }

  private void retractomatic() {
    double currentFacingAngleRelative =
        getFacing() + 10.0; // TODO: find facing that is minimum tether length
    double currentVelocity = getFacingOmega();
    double targetCurrent = 0.0; // POSITIVE OUT

    // CCW side of minimum tether length, negative (clockwise) velocity, pull in
    if (currentFacingAngleRelative > 0.0 && currentVelocity < 0.0) { // CCW side of 0; going in
      targetCurrent = p_goingInCurrent.getValue();
    } else if (currentFacingAngleRelative > 0.0
        && currentVelocity > 0.0) { // CCW side of 0; going out
      targetCurrent = p_goingOutCurrent.getValue();
    } else if (currentFacingAngleRelative < 0.0
        && currentVelocity < 0.0) { // CW side of 0; going out
      targetCurrent = p_goingOutCurrent.getValue();
    } else if (currentFacingAngleRelative < 0.0
        && currentVelocity > 0.0) { // CW side of 0; going in
      targetCurrent = p_goingInCurrent.getValue();
    } else if (currentFacingAngleRelative == 0.0
        || currentVelocity == 0.0) { // At min length or not moving
      targetCurrent = 0.0;
    } else { // should never find this state
      targetCurrent = 0.0;
      System.out.println(
          "Strange Retractomatic State" + currentFacingAngleRelative + currentVelocity);
    }

    m_retractomatic.setControl(torqueReq.withOutput(targetCurrent));
  }

  private void aimAtTarget() {
    goToFacing(m_targeting ? getTargetFacing() : 0.0);
  }

  private void goToFacing(double target) {
    goToFacing(target, false);
  }

  private void goToFacing(double target, boolean spinCompensation) {
    m_target = target;

    if (isPositionSafe(target)) {
      m_circumnavigating = false;
      goToPosition(turretFacingToFalconEncoderPosition(target), spinCompensation);
    } else if (m_circumnavigating && !isFacingSafe(target)) {
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
    if (motorsHealthy()) {
      if (spinCompensation) {
        m_turret.setControl(
            motionMagicReq
                .withPosition(position)
                .withFeedForward(p_spinCompensation.getValue() * m_robotYawRate.getAsDouble()));
      } else {
        m_turret.setControl(motionMagicReq.withPosition(position));
      }
      // run the retractomatic whenever we move the turret
      retractomatic();
    } else { // if both motors aren't healthy, don't move
      m_turret.stopMotor();
      m_retractomatic.stopMotor();
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
  private double getFacing() {
    return turretEncoderPositionToFacing(getTurretPosition());
  }

  @AutoLogOutput(key = "Turret/Velocity")
  private double getFacingOmega() {
    return turretEncoderPositionToFacing(m_turret.getVelocity().getValueAsDouble());
  }

  @AutoLogOutput
  private double getTurretPosition() {
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
    return isPositionSafe(getTurretPosition());
  }

  @AutoLogOutput
  private boolean isTracking() {
    return m_targeting;
  }

  @AutoLogOutput
  private boolean isNotMoving() {
    return Math.abs(getFacingOmega() * 10.) < 45.;
  }

  @AutoLogOutput
  private double getTarget() {
    return m_target;
  }

  private double turretEncoderPositionToFacing(double turretPosition) {
    return (turretPosition / (7.0 * (68.0 / 12.0)) * 360.0);
  }

  private double turretFacingToFalconEncoderPosition(double degrees) {
    return (degrees / 360.0) * (7.0 * (68.0 / 12.0));
  }

  @AutoLogOutput
  public boolean onTarget() {
    return m_targeting
        && !m_circumnavigating
        && Math.abs(getFacing() - m_target) < 7.0; // TODO: Lower 5.0 threshold
  }

  public Command calibrateEncoderCommand() {
    return new InstantCommand(() -> calibrateEncoder(), this);
  }

  public Command syncCommand() {
    return new InstantCommand(() -> sync(), this);
  }

  public Command aim() {
    return new RunCommand(() -> aimAtTarget(), this);
  }

  public Command stop() {
    return new RunCommand(
        () -> {
          m_turret.stopMotor();
          m_retractomatic.stopMotor();
        },
        this);
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
          "Turret/Constant", getFacing() / m_CANcoder.getAbsolutePosition().getValueAsDouble());
    }
  }
}
