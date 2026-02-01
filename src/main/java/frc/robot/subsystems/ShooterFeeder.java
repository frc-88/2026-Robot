package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;

public class ShooterFeeder extends SubsystemBase {

    private TalonFX feeder = new TalonFX(19, CANBus.roboRIO());

    private VelocityDutyCycle request = new VelocityDutyCycle(0.0);

    private DoublePreferenceConstant feedSpeed = new DoublePreferenceConstant("Feeder/FeedSpeed", 0.0);

    private MotionMagicPIDPreferenceConstants feederConfigConstants = new MotionMagicPIDPreferenceConstants("FeederMotor");

    public ShooterFeeder() {
        configureTalons();
    }

    private void configureTalons() {
        TalonFXConfiguration feederConfig = new TalonFXConfiguration();

        feederConfig.Slot0.kP = feederConfigConstants.getKP().getValue();
        feederConfig.Slot0.kI = feederConfigConstants.getKI().getValue();
        feederConfig.Slot0.kD = feederConfigConstants.getKD().getValue();
        feederConfig.Slot0.kV = feederConfigConstants.getKV().getValue();
        feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        feeder.getConfigurator().apply(feederConfig);
    }

    public void periodic() {
        SmartDashboard.putNumber("Feeder/FeederVelocity", feeder.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/FeederVoltage", feeder.getMotorVoltage().getValueAsDouble());
    }

    private void setFeederSpeed(DoubleSupplier speed) {
        feeder.setControl(request.withVelocity(speed.getAsDouble()));
    }

    private void stopFeederMotors() {
        feeder.stopMotor();
    }

    public Command runFeeder() {
        return new RunCommand (() -> setFeederSpeed(() -> feedSpeed.getValue()), this);
    }

    public Command stopFeeder() {
        return new RunCommand(() -> stopFeederMotors(), this);
    }
}
