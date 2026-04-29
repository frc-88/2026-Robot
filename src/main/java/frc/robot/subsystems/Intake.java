package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
  private final TalonFX intakeRollerMainLeft =
      new TalonFX(Constants.INTAKE_ROLLER_MAIN_LEFT, CANBus.roboRIO());
  private final TalonFX intakeRollerFollowerRight =
      new TalonFX(Constants.INTAKE_ROLLER_FOLLOWER_RIGHT, CANBus.roboRIO());
  private final TalonFX intakeInnerRoller =
      new TalonFX(Constants.INTAKE_INNER_ROLLER, CANBus.roboRIO());

  // output requests
  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0.0);
  private final VoltageOut theThingRequest = new VoltageOut(0.0);
  private final VelocityVoltage rollerRequest = new VelocityVoltage(0.0);
  private final VelocityVoltage pivotRollerRequest = new VelocityVoltage(0.0);

  // preferences
  private final MotionMagicPIDPreferenceConstants intakePivotConfigConstants =
      new MotionMagicPIDPreferenceConstants(
          "Intake/IntakePivotMotor", 50., 1000., 0., 1., 0., 0., 0.11, 0., 0.);
  private final MotionMagicPIDPreferenceConstants intakeRollerConfigConstants =
      new MotionMagicPIDPreferenceConstants(
          "Intake/IntakeRollerMotor", 50., 1000., 0., 0.5, 0., 0., 0.098, 0., 0.);
  private final MotionMagicPIDPreferenceConstants intakeInnerRollerConfigConstants =
      new MotionMagicPIDPreferenceConstants(
          "Intake/IntakePivotRollerMotor", 50., 1000., 0., 0.5, 0., 0., 0.098, 0., 0.);
  private final DoublePreferenceConstant targetPosition =
      new DoublePreferenceConstant("Intake/PivotTarget", 0.);
  private final DoublePreferenceConstant speed = new DoublePreferenceConstant("Intake/Speed", 80.0);
  private final DoublePreferenceConstant pivotRollerSpeed =
      new DoublePreferenceConstant("Intake/PivotRollerSpeed", 78.0);
  private final DoublePreferenceConstant deployPositionRotations =
      new DoublePreferenceConstant("Intake/DeployPosition", 27.06);

  private boolean isShooting = false;
  private boolean paused = false;
  private int stallCounter = 0;
  private int stopCounter = 0;

  DoubleSupplier m_drivespeed;
  private Debouncer stallDebouncer = new Debouncer(3.0, DebounceType.kRising);

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

    intakeRollerFollowerRight.getConfigurator().apply(rollerConfig);

    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeRollerMainLeft.getConfigurator().apply(rollerConfig);

    TalonFXConfiguration innerRollerConfig = new TalonFXConfiguration();
    innerRollerConfig.Slot0.kP = intakeInnerRollerConfigConstants.getKP().getValue();
    innerRollerConfig.Slot0.kI = intakeInnerRollerConfigConstants.getKI().getValue();
    innerRollerConfig.Slot0.kD = intakeInnerRollerConfigConstants.getKD().getValue();
    innerRollerConfig.Slot0.kV = intakeInnerRollerConfigConstants.getKV().getValue();
    innerRollerConfig.Slot0.kS = intakeInnerRollerConfigConstants.getKS().getValue();

    innerRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    innerRollerConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    innerRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    intakeInnerRoller.getConfigurator().apply(innerRollerConfig);

    intakeRollerFollowerRight.setControl(
        new Follower(Constants.INTAKE_ROLLER_MAIN_LEFT, MotorAlignmentValue.Opposed));
    intakeRollerFollowerRight.getMotorVoltage().setUpdateFrequency(500);
  }

  private void configureSmartDashboardButtons() {
    SmartDashboard.putData("Intake/Retract", retractIntake());
    SmartDashboard.putData("Intake/Deploy", deployIntake());
    SmartDashboard.putData(
        "Intake/SetDeployed",
        new InstantCommand(() -> intakePivot.setPosition(deployPositionRotations.getValue()))
            .ignoringDisable(true));
    SmartDashboard.putData(
        "Intake/SetZero",
        new InstantCommand(() -> intakePivot.setPosition(0.0)).ignoringDisable(true));

    SmartDashboard.putData("Intake/SetTooHigh", setTooHigh());
  }

  public void periodic() {
    Logger.recordOutput("Intake/IsShooting", isShooting);
    // if(isStalledPivot()) {
    //   intakePivot.setPosition(deployPositionRotations.getValue());
    // }
  }

  @AutoLogOutput
  public boolean isHealthy() {
    return intakePivot.isConnected()
        && intakePivot.isAlive()
        && intakeRollerMainLeft.isConnected()
        && intakeRollerMainLeft.isAlive()
        && intakeRollerFollowerRight.isConnected()
        && intakeRollerFollowerRight.isAlive();
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
  private Current getRollerMainCurrent() {
    return intakeRollerMainLeft.getStatorCurrent().getValue();
  }

  @AutoLogOutput
  private Current getRollerFollowerCurrent() {
    return intakeRollerFollowerRight.getStatorCurrent().getValue();
  }

  @AutoLogOutput
  private double getRollerMainVoltage() {
    return intakeRollerMainLeft.getMotorVoltage().getValueAsDouble();
  }

  @AutoLogOutput
  private double getRollerFollowerVoltage() {
    return intakeRollerFollowerRight.getMotorVoltage().getValueAsDouble();
  }

  @AutoLogOutput
  private double getRollerMainVelocity() {
    return intakeRollerMainLeft.getVelocity().getValueAsDouble();
  }

  @AutoLogOutput
  private double getRollerFollowerVelocity() {
    return intakeRollerFollowerRight.getVelocity().getValueAsDouble();
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
  private boolean isStalledRoller() {
    return intakeRollerMainLeft.getStatorCurrent().getValueAsDouble() > 20.0
        && intakeRollerMainLeft.getVelocity().getValueAsDouble() < 8.0;
  }

  @AutoLogOutput
  private boolean isStalledPivot() {
    return stallDebouncer.calculate(
        intakePivot.getPosition().getValueAsDouble() > 10.0
            && intakePivot.getStatorCurrent().getValueAsDouble() > 4.0
            && intakePivot.getVelocity().getValueAsDouble() < 8.0);
  }

  private double intakePivotAngleDegreesToRotations(double pivotAngle) {
    return (pivotAngle / 360.0) * Constants.INTAKE_PIVOT_MOTOR_ROTATIONS_TO_ROTATIONS;
  }

  private double intakePivotRotationsToAngle(double minionRotations) { // inverse of ^^
    return (minionRotations * 360) / Constants.INTAKE_PIVOT_MOTOR_ROTATIONS_TO_ROTATIONS;
  }

  private void goToRotations(double minionRotations) {
    if (isStalledPivot() && minionRotations == deployPositionRotations.getValue()) {
      intakePivot.setPosition(deployPositionRotations.getValue());
    }
    intakePivot.setControl(pivotRequest.withPosition(minionRotations));
  }

  private void setRollerSpeed(DoubleSupplier speed) {
    if (isStalledRoller()) {
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

    intakeRollerMainLeft.setControl(rollerRequest.withVelocity(paused ? 0.0 : speed.getAsDouble()));
  }

  private void stopRoller() {
    intakeRollerMainLeft.stopMotor();
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
    if (intakePivot.getPosition().getValueAsDouble() > 5.0) {
      intakePivot.setControl(theThingRequest.withOutput(-3.0));
    } else {
      intakePivot.stopMotor();
    }
    setRollerSpeed(() -> speed.getValue() * 1.0);
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

  public Command setTooHigh() {
    return new InstantCommand(
            () -> intakePivot.setPosition(deployPositionRotations.getValue() - 1.0))
        .ignoringDisable(true);
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
