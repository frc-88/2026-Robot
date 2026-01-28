package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;

public class Shooter extends SubsystemBase {
    private TalonFX shooterMain = new TalonFX(12, CANBus.roboRIO()); //forward +
    private TalonFX shooterFollower = new TalonFX(2, CANBus.roboRIO()); //forward -

    private VelocityDutyCycle requestShooter = new VelocityDutyCycle(0.0);
    private DoublePreferenceConstant shootSpeed = new DoublePreferenceConstant("Shooter/ShootSpeed", 0.0);

    private MotionMagicPIDPreferenceConstants shooterConfigConstants = new MotionMagicPIDPreferenceConstants("TurretMainMotor");


    public Shooter() {
        configureTalons();
    }

    private void configureTalons() {
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

        shooterConfig.Slot0.kP = shooterConfigConstants.getKP().getValue();
        shooterConfig.Slot0.kI = shooterConfigConstants.getKI().getValue();
        shooterConfig.Slot0.kD = shooterConfigConstants.getKD().getValue();
        shooterConfig.Slot0.kV = shooterConfigConstants.getKV().getValue();
        shooterMain.getConfigurator().apply(shooterConfig);
        // shooterFollower.getConfigurator().apply(shooterConfig);
        shooterFollower.setControl(new Follower(12, MotorAlignmentValue.Opposed));
    }
    public void periodic() {
        SmartDashboard.putNumber("Shooter/ShooterVelocity", shooterMain.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/ShooterVoltage", shooterMain.getMotorVoltage().getValueAsDouble());
    }

    private void setShooterSpeed(DoubleSupplier speed) {
        shooterMain.setControl(requestShooter.withVelocity(speed.getAsDouble()));
    }

    private void stopShooterMotors() {
        shooterMain.stopMotor();
        // shooterFollower.stopMotor();
    }

    public Command runShooter() {
        return new RunCommand (() -> setShooterSpeed(() -> shootSpeed.getValue()), this);
    }

    public Command stopShooter() {
        return new RunCommand(() -> stopShooterMotors(), this);
    }

}