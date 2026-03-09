package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
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

public class Intake extends SubsystemBase {
  // motors & devices
  private final TalonFX intakePivot = new TalonFX(Constants.INTAKE_PIVOT, CANBus.roboRIO());
  private final TalonFX intakeRoller = new TalonFX(Constants.INTAKE_ROLLER, CANBus.roboRIO());

  // output requests
  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0.0);
  private final DutyCycleOut calibrationRequest = new DutyCycleOut(0.0);
  private final DutyCycleOut rollerRequest = new DutyCycleOut(0);

  // preferences
  private final MotionMagicPIDPreferenceConstants intakePivotConfigConstants =
      new MotionMagicPIDPreferenceConstants(
          "Intake/IntakePivotMotor", 50., 1000., 0., 0., 0., 0., 0.11, 0., 0.);
  private final MotionMagicPIDPreferenceConstants intakeRollerConfigConstants =
      new MotionMagicPIDPreferenceConstants(
          "Intake/IntakeRollerMotor", 50., 1000., 0., 0., 0., 0., 0.11, 0., 0.);
  private final DoublePreferenceConstant targetPos =
      new DoublePreferenceConstant("Intake/PivotTarget", 0.);
  private final DoublePreferenceConstant speed = new DoublePreferenceConstant("Intake/Speed", 0.8);

  private boolean isDoingTheThing = false;
  private double lastTimestamp = 0.0;

  public Intake() {
    configureTalons();
    configureSmartDashboardButtons();
    // TODO CHANGE THIS THING
    // TODO CHANGE THIS THING
    // TODO CHANGE THIS THING
    // TODO CHANGE THIS THING
    // TODO CHANGE THIS THING
    intakePivot.setPosition(23.0);
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

    intakePivot.getConfigurator().apply(pivotConfig);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    // rollerConfig.Slot0.kP = intakeRollerConfigConstants.getKP().getValue();
    // rollerConfig.Slot0.kI = intakeRollerConfigConstants.getKI().getValue();
    // rollerConfig.Slot0.kD = intakeRollerConfigConstants.getKD().getValue();
    // rollerConfig.Slot0.kV = intakeRollerConfigConstants.getKV().getValue();
    // rollerConfig.Slot0.kS = intakeRollerConfigConstants.getKS().getValue();
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeRoller.getConfigurator().apply(rollerConfig);
  }

  private void configureSmartDashboardButtons() {
    SmartDashboard.putData("Intake/Calibrate", calibrateIntake());
    SmartDashboard.putData("Intake/JustPivot", deployJustIntake());
    SmartDashboard.putData("Intake/SetPosition", pivotGoToPosition());
    SmartDashboard.putData("Intake/SetRotations", pivotGoToRotations());
    SmartDashboard.putData("Intake/Retract", retractIntake());
    SmartDashboard.putData("Intake/Deploy", deployIntake());
    SmartDashboard.putData("Intake/Zero", zeroIntake().ignoringDisable(true));
  }

  public void periodic() {
    SmartDashboard.putNumber(
        "Intake/Setpoint", intakePivotAngleDegreesToRotations(targetPos.getValue()));
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
  private double getRollerVelocity() {
    return intakeRoller.getVelocity().getValueAsDouble();
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

  private void setSpinnerSpeed(DoubleSupplier speed) {
    intakeRoller.setControl(rollerRequest.withOutput(speed.getAsDouble()));
  }

  private void stopSpinner() {
    intakeRoller.stopMotor();
    ;
  }

  private void setPosition(double angle) {
    intakePivot.setControl(pivotRequest.withPosition(intakePivotAngleDegreesToRotations(angle)));
  }

  private void setCalibrate() {
    intakePivot.setControl(calibrationRequest.withOutput(-0.1).withIgnoreSoftwareLimits(true));
    if (intakePivot.getStatorCurrent().getValueAsDouble() > 20.0) {
      intakePivot.setPosition(intakePivotAngleDegreesToRotations(0));
    }
  }

  private void intakeOut() {
    goToRotations(23.0); // TODO
    setSpinnerSpeed(() -> speed.getValue());
  }

  private void intakeIn() {
    goToRotations(0.6);
    stopSpinner();
  }

  private void theThing() {
    double toUse = 0.0;
    if (isDoingTheThing) {
      toUse = lastTimestamp;
    } else {
      isDoingTheThing = true;
      lastTimestamp = Timer.getFPGATimestamp();
      toUse = lastTimestamp;
    }

    double setpoint =
        (((21.0 + 0.6) / 2.0) / 2.0)
                * Math.sin(Math.PI * 2.0 * (Timer.getFPGATimestamp() - toUse - (Math.PI / 2.0)))
            + ((21.0 + 0.6) / 2.0);
    goToRotations(setpoint);
    setSpinnerSpeed(() -> speed.getValue());
  }

  private void justIntakeOut() {
    goToRotations(23.0); // TODO
    stopSpinner();
  }

  public Command zeroIntake() {
    return new InstantCommand(() -> intakePivot.setPosition(0.0));
  }

  public Command runIntake() {
    return new RunCommand(() -> setSpinnerSpeed(() -> speed.getValue()), this);
  }

  public Command stopIntake() {
    return new RunCommand(() -> stopSpinner(), this);
  }

  public Command calibrateIntake() {
    return new RunCommand(() -> setCalibrate(), this)
        .until(() -> intakePivot.getStatorCurrent().getValueAsDouble() > 30.0)
        .andThen(stopIntake());
  }

  public Command pivotGoToPosition() {
    return new RunCommand(() -> setPosition(targetPos.getValue()), this);
  }

  public Command pivotGoToRotations() {
    return new RunCommand(() -> goToRotations(targetPos.getValue()), this);
  }

  public Command retractIntake() {
    // TODO: determine proper retract angle, put it here
    return new RunCommand(() -> intakeIn(), this);
  }

  public Command deployIntake() {
    // TODO: determine proper deploy angle, put it here
    return new RunCommand(() -> intakeOut(), this);
  }

  public Command deployJustIntake() {
    // TODO: determine proper deploy angle, put it here
    return new RunCommand(() -> justIntakeOut(), this);
  }

  public Command doTheThing() {
    return new RunCommand(() -> theThing(), this).finallyDo(() -> isDoingTheThing = false);
  }
}
