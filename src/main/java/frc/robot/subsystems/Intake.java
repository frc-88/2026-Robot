package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
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
import org.littletonrobotics.junction.Logger;

// hey kids! do The Thing!
// put your arms out, pull them back
// it's The Thing! oh yeah!

public class Intake extends SubsystemBase {
  // motors & devices
  private final TalonFX intakePivot = new TalonFX(Constants.INTAKE_PIVOT, CANBus.roboRIO());
  private final TalonFX intakeRoller = new TalonFX(Constants.INTAKE_ROLLER, CANBus.roboRIO());
  private final TalonFX intakeInnerRoller =
      new TalonFX(Constants.INTAKE_PIVOT_ROLLER, CANBus.roboRIO());

  // output requests
  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0.0);
  private final DynamicMotionMagicVoltage pivotRequestDynamic =
      new DynamicMotionMagicVoltage(0.0, 0.1, 10.0);
  private final VelocityVoltage rollerRequest = new VelocityVoltage(0.0);
  private final VelocityVoltage pivotRollerRequest = new VelocityVoltage(0.0);

  // preferences
  private final MotionMagicPIDPreferenceConstants intakePivotConfigConstants =
      new MotionMagicPIDPreferenceConstants(
          "Intake/IntakePivotMotor", 50., 1000., 0., 0., 0., 0., 0.11, 0., 0.);
  private final MotionMagicPIDPreferenceConstants intakeRollerConfigConstants =
      new MotionMagicPIDPreferenceConstants(
          "Intake/IntakeRollerMotor", 50., 1000., 0., 0.5, 0., 0., 0.098, 0., 0.);
  private final MotionMagicPIDPreferenceConstants intakePivotRollerConfigConstants =
      new MotionMagicPIDPreferenceConstants(
          "Intake/IntakePivotRollerMotor", 50., 1000., 0., 0.5, 0., 0., 0.098, 0., 0.);
  private final DoublePreferenceConstant targetPosition =
      new DoublePreferenceConstant("Intake/PivotTarget", 0.);
  private final DoublePreferenceConstant speed =
      new DoublePreferenceConstant("Intake/Speed", 100.0);
  private final DoublePreferenceConstant pivotRollerSpeed =
      new DoublePreferenceConstant("Intake/PivotRollerSpeed", 78.0);
  private final DoublePreferenceConstant deployPositionRotations =
      new DoublePreferenceConstant("Intake/DeployPosition", 27.6);

  private boolean isShooting = false;
  private boolean paused = false;
  private int stallCounter = 0;
  private int stopCounter = 0;

  DoubleSupplier m_drivespeed;

  public Intake(DoubleSupplier speed) {
    m_drivespeed = speed;
    configureTalons();
    configureSmartDashboardButtons();

    intakePivot.setPosition(0.0);
  }

  private void configureTalons() {
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0.kP = intakePivotConfigConstants.getKP().getValue();
    pivotConfig.Slot0.kI = intakePivotConfigConstants.getKI().getValue();
    pivotConfig.Slot0.kD = intakePivotConfigConstants.getKD().getValue();
    pivotConfig.Slot0.kV = intakePivotConfigConstants.getKV().getValue();
    pivotConfig.Slot0.kS = intakePivotConfigConstants.getKS().getValue();
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity =
        intakePivotConfigConstants.getMaxVelocity().getValue();
    pivotConfig.MotionMagic.MotionMagicAcceleration =
        intakePivotConfigConstants.getMaxAcceleration().getValue();
    pivotConfig.MotionMagic.MotionMagicJerk = intakePivotConfigConstants.getMaxJerk().getValue();
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // TODO: figure out limits and enable them
    // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = <forward limit>
    // config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = <reverse limit>
    // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 40.0;

    intakePivot.getConfigurator().apply(pivotConfig);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.Slot0.kP = intakeRollerConfigConstants.getKP().getValue();
    rollerConfig.Slot0.kI = intakeRollerConfigConstants.getKI().getValue();
    rollerConfig.Slot0.kD = intakeRollerConfigConstants.getKD().getValue();
    rollerConfig.Slot0.kV = intakeRollerConfigConstants.getKV().getValue();
    rollerConfig.Slot0.kS = intakeRollerConfigConstants.getKS().getValue();

    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    intakeRoller.getConfigurator().apply(rollerConfig);

    TalonFXConfiguration innerRollerConfig = new TalonFXConfiguration();
    innerRollerConfig.Slot0.kP = intakePivotRollerConfigConstants.getKP().getValue();
    innerRollerConfig.Slot0.kI = intakePivotRollerConfigConstants.getKI().getValue();
    innerRollerConfig.Slot0.kD = intakePivotRollerConfigConstants.getKD().getValue();
    innerRollerConfig.Slot0.kV = intakePivotRollerConfigConstants.getKV().getValue();
    innerRollerConfig.Slot0.kS = intakePivotRollerConfigConstants.getKS().getValue();

    innerRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    innerRollerConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    innerRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    intakeInnerRoller.getConfigurator().apply(innerRollerConfig);
  }

  private void configureSmartDashboardButtons() {
    SmartDashboard.putData("Intake/Retract", retractIntake());
    SmartDashboard.putData("Intake/Deploy", deployIntake());
    SmartDashboard.putData(
        "Intake/SetDeployed",
        new InstantCommand(() -> intakePivot.setPosition(27.6)).ignoringDisable(true));
    SmartDashboard.putData(
        "Intake/SetZero",
        new InstantCommand(() -> intakePivot.setPosition(0.0)).ignoringDisable(true));
  }

  public void periodic() {
    Logger.recordOutput("Intake/IsShooting", isShooting);
  }

  @AutoLogOutput
  public boolean isHealthy() {
    return intakePivot.isConnected()
        && intakePivot.isAlive()
        && intakeRoller.isConnected()
        && intakeRoller.isAlive();
  }

  @AutoLogOutput
  private Current getPivotCurrent() {
    return intakePivot.getStatorCurrent().getValue();
  }

  @AutoLogOutput
  private double getPivotPosition() {
    return intakePivot.getPosition().getValueAsDouble();
  }

  @AutoLogOutput
  private double getPivotAngle() {
    return intakePivotRotationsToAngle(getPivotPosition());
  }

  @AutoLogOutput
  private Current getRollerCurrent() {
    return intakeRoller.getStatorCurrent().getValue();
  }

  @AutoLogOutput
  private double getRollerVoltage() {
    return intakeRoller.getMotorVoltage().getValueAsDouble();
  }

  @AutoLogOutput
  private double getRollerVelocity() {
    return intakeRoller.getVelocity().getValueAsDouble();
  }

  @AutoLogOutput
  private Current getPivotRollerCurrent() {
    return intakeInnerRoller.getStatorCurrent().getValue();
  }

  @AutoLogOutput
  private double getPivotRollerVoltage() {
    return intakeInnerRoller.getMotorVoltage().getValueAsDouble();
  }

  @AutoLogOutput
  private double getPivotRollerVelocity() {
    return intakeInnerRoller.getVelocity().getValueAsDouble();
  }

  @AutoLogOutput
  private boolean isStalled() {
    return intakeRoller.getStatorCurrent().getValueAsDouble() > 20.0
        && intakeRoller.getVelocity().getValueAsDouble() < 0.1;
  }

  private double intakePivotAngleDegreesToRotations(double pivotAngle) {
    return (pivotAngle / 360.0) * Constants.INTAKE_PIVOT_MOTOR_ROTATIONS_TO_ROTATIONS;
  }

  private double intakePivotRotationsToAngle(double minionRotations) { // inverse of ^^
    return (minionRotations * 360) / Constants.INTAKE_PIVOT_MOTOR_ROTATIONS_TO_ROTATIONS;
  }

  private void goToRotations(double minionRotations) {
    intakePivot.setControl(pivotRequest.withPosition(minionRotations));
  }

  private void setRollerSpeed(DoubleSupplier speed) {
    if (isStalled()) {
      stallCounter++;
    } else if (!paused) {
      stallCounter = 0;
    }

    if (paused) {
      stopCounter++;
    } else {
      stopCounter = 0;
    }

    if (stallCounter > 50 && stopCounter > 10) {
      stallCounter = 0;
      paused = false;
    } else if (stallCounter > 50) {
      paused = true;
    }

    intakeRoller.setControl(rollerRequest.withVelocity(paused ? 0.0 : speed.getAsDouble()));
  }

  private void stopRoller() {
    intakeRoller.stopMotor();
  }

  private void setPivotRollerSpeed(DoubleSupplier speed) {
    intakeInnerRoller.setControl(pivotRollerRequest.withVelocity(speed.getAsDouble()));
  }

  private void rollerSpit() {
    setRollerSpeed(() -> -70.0);
  }

  private void pivotRollerSpit() {
    setPivotRollerSpeed(() -> -70.0);
  }

  private void stopPivotRoller() {
    intakeInnerRoller.stopMotor();
  }

  private void setPosition(double angle) {
    intakePivot.setControl(pivotRequest.withPosition(intakePivotAngleDegreesToRotations(angle)));
  }

  private void intakeOut() {
    goToRotations(deployPositionRotations.getValue());
    setRollerSpeed();
    setPivotRollerSpeed(() -> pivotRollerSpeed.getValue());
  }

  public void setRollerSpeed() {
    setRollerSpeed(
        () ->
            MathUtil.clamp(
                (m_drivespeed.getAsDouble() / (Math.PI * Units.inchesToMeters(1))), 90.0, 120.0));
  }

  public void intakeIn() {
    goToRotations(0.0);
    stopRoller();
    stopPivotRoller();
  }

  public void intakeSpit() {
    goToRotations(deployPositionRotations.getValue());
    rollerSpit();
    pivotRollerSpit();
  }

  private void theThing() {
    intakePivot.setControl(
        pivotRequestDynamic.withAcceleration(100.0).withVelocity(2.0).withPosition(5.0));
    setRollerSpeed(() -> speed.getValue() / 10.0);
    setPivotRollerSpeed(() -> pivotRollerSpeed.getValue());
  }

  private void justIntakeOut() {
    if (isShooting) {
      goToRotations(deployPositionRotations.getValue());
      setPivotRollerSpeed(() -> pivotRollerSpeed.getValue());
      setRollerSpeed();
    } else {
      goToRotations(deployPositionRotations.getValue());
      stopPivotRoller();
      stopRoller();
    }
  }

  private void antiJam() {
    setRollerSpeed(() -> -speed.getValue());
  }

  public Command setNotShooting() {
    return new InstantCommand(() -> isShooting = false);
  }

  public Command setShooting() {
    return new InstantCommand(() -> isShooting = true);
  }

  public Command antiJamIntake() {
    return new InstantCommand(() -> antiJam());
  }

  public Command runIntake() {
    return new RunCommand(
        () ->
            setRollerSpeed(
                () -> (m_drivespeed.getAsDouble() / (Math.PI * Units.inchesToMeters(1)))),
        this);
  }

  public Command stopIntake() {
    return new RunCommand(() -> stopRoller(), this);
  }

  public Command pivotGoToPosition() {
    return new RunCommand(() -> setPosition(targetPosition.getValue()), this);
  }

  public Command pivotGoToRotations() {
    return new RunCommand(() -> goToRotations(targetPosition.getValue()), this);
  }

  public Command retractIntake() {
    return new RunCommand(() -> intakeIn(), this);
  }

  public Command intakeSpitCommand() {
    return new RunCommand(() -> intakeSpit(), this);
  }

  public Command deployIntake() {
    return new RunCommand(() -> intakeOut(), this);
  }

  public Command deployJustIntake() {
    return new RunCommand(() -> justIntakeOut(), this);
  }

  public Command doTheThing() {
    return new RunCommand(() -> theThing(), this);
  }
}
