package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;

public class Feeder extends SubsystemBase {

    private TalonFX shooterFeeder = new TalonFX(18, CANBus.roboRIO());

    private VelocityDutyCycle requestFeeder = new VelocityDutyCycle(0.0);

    private DoublePreferenceConstant feedSpeed = new DoublePreferenceConstant("Feeder/FeedSpeed", 0.0);

    private MotionMagicPIDPreferenceConstants feederConfigConstants = new MotionMagicPIDPreferenceConstants("FeederMotor");

    public Feeder() {
        configureTalons();
    }

    private void configureTalons() {
        TalonFXConfiguration feederConfig = new TalonFXConfiguration();

        feederConfig.Slot0.kP = feederConfigConstants.getKP().getValue();
        feederConfig.Slot0.kI = feederConfigConstants.getKI().getValue();
        feederConfig.Slot0.kD = feederConfigConstants.getKD().getValue();
        feederConfig.Slot0.kV = feederConfigConstants.getKV().getValue();
        shooterFeeder.getConfigurator().apply(feederConfig);
    }

    public void periodic() {
        SmartDashboard.putNumber("Turret/FeederVelocity", shooterFeeder.getVelocity().getValueAsDouble());
    }

    private void setFeederSpeed(DoubleSupplier speed) {
        shooterFeeder.setControl(requestFeeder.withVelocity(speed.getAsDouble()));
    }

    private void stopOnlyFeederMotors() {
        shooterFeeder.stopMotor();
    }

    public Command runOnlyFeeder() {
        return new RunCommand (() -> setFeederSpeed(() -> feedSpeed.getValue()), this);
    }

    public Command stopOnlyFeeder() {
        return new RunCommand(() -> stopOnlyFeederMotors(), this);
    }
}
