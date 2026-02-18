package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;

public class Hood {
    private TalonFXS hood = new TalonFXS(16, CANBus.roboRIO());

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
    }

    private void calibrate() {

    }
}
