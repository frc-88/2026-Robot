package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;

public class Hood extends SubsystemBase {
  private TalonFXS hood = new TalonFXS(Constants.HOOD, CANBus.roboRIO());

  private final MotionMagicPIDPreferenceConstants hoodConfigConstants =
      new MotionMagicPIDPreferenceConstants("Hood/HoodMotor");

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
    hood.getConfigurator().apply(hoodConfig);
  }

  private void configureSmartDashboardButtons() {
    SmartDashboard.putNumber("Hood/Position", hood.getPosition().getValueAsDouble());
    SmartDashboard.putData("Hood/Calibrate", calibrate());
  }

  private void setCalibrate() {
    hood.setControl(new DutyCycleOut(0.16));
  }

  private void stopHoodMotor() {
    hood.stopMotor();
  }

  public Command stopHood() {
    return new RunCommand(() -> stopHoodMotor());
  }

  public Command calibrate() {
    return new SequentialCommandGroup(
            new RunCommand(() -> setCalibrate(), this)
                .until(() -> hood.getStatorCurrent().getValueAsDouble() > 40.0),
            new InstantCommand(() -> hood.setPosition(0.0), this))
        .andThen(new RunCommand(() -> hood.setControl(new DutyCycleOut(0.0)), this));
  }
}
