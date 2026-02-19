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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;

public class Hood extends SubsystemBase {
  private TalonFXS hood = new TalonFXS(Constants.HOOD, CANBus.roboRIO());

  private final MotionMagicPIDPreferenceConstants hoodConfigConstants =
      new MotionMagicPIDPreferenceConstants("Hood/HoodMotor");
  private DoublePreferenceConstant targetPos = new DoublePreferenceConstant("Hood/Target", 0);
  private MotionMagicVoltage request = new MotionMagicVoltage(0.0);

  public Hood() {
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

    hoodConfig.MotionMagic.MotionMagicCruiseVelocity =
        hoodConfigConstants.getMaxVelocity().getValue();
    hoodConfig.MotionMagic.MotionMagicAcceleration =
        hoodConfigConstants.getMaxAcceleration().getValue();
    hood.getConfigurator().apply(hoodConfig);
  }

  private void configureSmartDashboardButtons() {
    // SmartDashboard.putNumber("Hood/Position", hood.getPosition().getValueAsDouble());
    SmartDashboard.putData("Hood/Calibrate", calibrate());
    SmartDashboard.putNumber("Hood/Current", hood.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putData("Hood/SetPosition", setPositionThing());
    SmartDashboard.putData("Hood/GoToZero", goToZero());
  }

  public void periodic() {
    SmartDashboard.putNumber(
        "Hood/Setpoint", hoodAngleDegreesToRotationsOfMinion(targetPos.getValue()));

      }

  private double hoodAngleDegreesToRotationsOfMinion(double hoodAngle) {
    return (hoodAngle / 360.0) * (287.0 / 18.0) * (36.0 / 12.0);
  }

  private void setPosition(double angle) {
    hood.setControl(request.withPosition(hoodAngleDegreesToRotationsOfMinion(angle)));
  }

  private void setCalibrate() {
    hood.setControl(new DutyCycleOut(-0.16));
  }

  private void stopHoodMotor() {
    hood.stopMotor();
  }

  public Command stopHood() {
    return new RunCommand(() -> stopHoodMotor(), this);
  }

  public Command calibrate() {
    return new SequentialCommandGroup(
            new RunCommand(() -> setCalibrate(), this)
                .until(() -> hood.getStatorCurrent().getValueAsDouble() > 60.0),
            new InstantCommand(
                () -> hood.setPosition(hoodAngleDegreesToRotationsOfMinion(13.5)), this))
        .andThen(stopHood());
  }

  public Command setPositionThing() {
    return new RunCommand(() -> setPosition(targetPos.getValue()), this);
  }

  public Command goToZero() {
    return new RunCommand(() -> setPosition(0.0), this);
  }
}
