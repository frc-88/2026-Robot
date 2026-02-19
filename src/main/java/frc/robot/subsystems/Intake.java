package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;

public class Intake extends SubsystemBase {

  private final TalonFXS intakePivot = new TalonFXS(Constants.INTAKE_MAIN, CANBus.roboRIO());
  private final MotionMagicPIDPreferenceConstants intakeConfigConstants =
      new MotionMagicPIDPreferenceConstants("Intake/IntakePivotMotor");
  private MotionMagicVoltage request = new MotionMagicVoltage(0.0);
  private DoublePreferenceConstant targetPos =
      new DoublePreferenceConstant("Intake/PivotTarget", 0);
  private PWM intakeSpinner = new PWM(0);

  private DoublePreferenceConstant speed = new DoublePreferenceConstant("Intake/Speed", 0.8);

  public Intake() {
    configureTalons();
    configureSmartDashboardButtons();
  }

  private void configureTalons() {
    TalonFXSConfiguration config = new TalonFXSConfiguration();
    config.Slot0.kP = intakeConfigConstants.getKP().getValue();
    config.Slot0.kI = intakeConfigConstants.getKI().getValue();
    config.Slot0.kD = intakeConfigConstants.getKD().getValue();
    config.Slot0.kV = intakeConfigConstants.getKV().getValue();
    config.Slot0.kS = intakeConfigConstants.getKS().getValue();
    config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    config.MotorOutput.Inverted =
        InvertedValue.CounterClockwise_Positive; // this might not work yet

    // TODO: figure out limits and enable them
    // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = <forward limit>
    // config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = <reverse limit>
    // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    intakePivot.getConfigurator().apply(config);
  }

  private void configureSmartDashboardButtons() {
    SmartDashboard.putData("Intake/Calibrate", calibrateIntake());
    SmartDashboard.putData("Intake/SetPosition", setPositionThing());
    SmartDashboard.putData("Intake/Retract", retractIntake());
    SmartDashboard.putData("Intake/Deploy", deployIntake());
  }

  public void periodic() {
    SmartDashboard.putNumber("Intake/Current", intakePivot.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "Intake/Setpoint", intakePivotAngleDegreesToRotations(targetPos.getValue()));
    SmartDashboard.putNumber(
        "Intake/CurrentPosition", intakePivot.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(
        "Intake/CurrentAngle",
        intakePivotRotationsToAngle(intakePivot.getPosition().getValueAsDouble()));
  }

  private double intakePivotAngleDegreesToRotations(double hoodAngle) {
    // TODO: determine actual conversion,this is from Hood
    return (hoodAngle / 360.0) * (287.0 / 18.0) * (36.0 / 12.0);
  }

  private double intakePivotRotationsToAngle(double minionRotations) { // inverse of ^^
    // TODO: determine actual conversion,this is from Hood
    return (minionRotations * 360.0) / ((287.0 / 18.0) * (36.0 / 12.0));
  }

  // private void setSpeed(double speed) {
  //   intake.setControl(requestcycle.withOutput(speed));
  // }

  // private void stopMotors() {
  //   intake.stopMotor();
  // }

  private void setSpeed(double speed) {
    intakeSpinner.setSpeed(speed);
  }

  private void stopMotors() {
    intakeSpinner.setSpeed(0.0);
  }

  public Command runIntake() {
    return new RunCommand(() -> setSpeed(speed.getValue()), this);
  }

  public Command stopIntake() {
    return new RunCommand(() -> stopMotors(), this);
  }

  private void setPosition(double angle) {
    intakePivot.setControl(request.withPosition(intakePivotAngleDegreesToRotations(angle)));
  }

  private void setCalibrate() {
    // TODO: convert this (from Hood) to Intake and figure out proper calibration method
    // intakePivot.setControl(request.withOutput(-0.16).withIgnoreSoftwareLimits(true));
    // if (intakePivot.getStatorCurrent().getValueAsDouble() > 20.0) {
    //   intakePivot.setPosition(hoodAngleDegreesToRotationsOfMinion(13.5));

  }

  public Command calibrateIntake() {
    // TODO: this is from Hood, it may need to be adapted for Intake
    return new RunCommand(() -> setCalibrate(), this)
        .until(() -> intakePivot.getStatorCurrent().getValueAsDouble() > 60.0)
        .andThen(stopIntake());
  }

  public Command setPositionThing() {
    return new RunCommand(() -> setPosition(targetPos.getValue()), this);
  }

  public Command retractIntake() {
    // TODO: determine proper retract angle, put it here
    return new RunCommand(() -> setPosition(0.0), this);
  }

  public Command deployIntake() {
    // TODO: determine proper deploy angle, put it here
    return new RunCommand(() -> setPosition(0.0), this);
  }
}
